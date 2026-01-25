#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <future>
#include <tuple>
#include <type_traits>
#include <thread>
#include <cctype>

#include "timer.hpp"
#include "dgp/termcolor.hpp"
// #include "mighty/mighty_type.hpp"
#include <mighty/utils.hpp>
// #include <mighty/mighty_node.hpp>
#include <mighty/mighty.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// #include "dgp/dgp_manager.hpp"
// #include "mighty/lbfgs_solver.hpp"
// #include <mighty/gurobi_solver.hpp>
#include <decomp_rviz_plugins/data_ros_utils.hpp>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>


namespace fs = std::filesystem;
using namespace std::chrono;

using namespace mighty;

using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Matrix<double, 3, 1>;

struct BenchResult
{
    std::string file;
    bool success{false};
    bool gurobi_error{false};

    double per_opt_runtime_ms{0.0};   // from GRB Runtime * 1000
    double total_opt_runtime_ms{0.0}; // end-to-end wall time for this case
    double total_traj_time_sec{0.0};
    std::string status;

    Vec3d start{0, 0, 0};
    Vec3d goal{0, 0, 0};

    double factor_used{0.0};
    double cost_value{-100000.0};

    // -------- Constraint violation metrics (sampled at dt=dc) --------
    // Safe corridor (union of polytopes): for each sample, compute min_i max(A_i p - b_i).
    // max over samples of that value is reported here. <= 0 means "inside at least one polytope".
    double corridor_max_min_violation{0.0}; // max_t min_poly max(Ap-b) (positive means outside corridor union)
    double corridor_t_at_max{0.0};
    Vec3d corridor_p_at_max{0, 0, 0};
    int corridor_best_poly_idx_at_max{-1};

    // Dynamic constraints
    double v_max_observed{0.0};
    double v_max_excess{0.0}; // max( ||v|| - v_max ) (axis-wise in current code)
    double v_t_at_max{0.0};

    double a_max_observed{0.0};
    double a_max_excess{0.0}; // max( ||a|| - a_max )
    double a_t_at_max{0.0};

    double j_max_observed{0.0};
    double j_max_excess{0.0}; // max( ||j|| - j_max )
    double j_t_at_max{0.0};

    // Convenience flags (derived from the above using tolerances)
    bool corridor_violated{false};
    bool v_violated{false};
    bool a_violated{false};
    bool j_violated{false};

    // -------- Smoothness metrics (sampled at dt=dc; jerk via finite-diff of accel) --------
    // S_jerk    = ∫_0^T ||j(t)|| dt
    // Sbar_jerk = sqrt( (1/T) ∫_0^T ||j(t)||^2 dt )   (RMS jerk; time-normalized)
    double jerk_smoothness_l1{0.0};
    double jerk_rms{0.0};

    double traj_length_m{0.0}; // ∫ ||v(t)|| dt

    // Visualization payloads (cached)
    decomp_ros_msgs::msg::PolyhedronArray poly_msg;
    visualization_msgs::msg::MarkerArray opt_traj_ma;    // committed colored
    visualization_msgs::msg::MarkerArray global_path_ma; // marker array path
};

// ------------------------ helpers ------------------------

static void writeU32(std::ofstream &ofs, uint32_t v)
{
    ofs.write(reinterpret_cast<const char *>(&v), sizeof(v));
}
static void writeD(std::ofstream &ofs, double v)
{
    ofs.write(reinterpret_cast<const char *>(&v), sizeof(v));
}
static uint32_t readU32(std::ifstream &ifs)
{
    uint32_t v;
    ifs.read(reinterpret_cast<char *>(&v), sizeof(v));
    if (!ifs)
        throw std::runtime_error("Corrupt .mysco2 (u32).");
    return v;
}
static double readD(std::ifstream &ifs)
{
    double v;
    ifs.read(reinterpret_cast<char *>(&v), sizeof(v));
    if (!ifs)
        throw std::runtime_error("Corrupt .mysco2 (double).");
    return v;
}

static void fillStateFromPos(state &s, const Vec3d &p)
{
    s.pos = p;
    s.vel = Eigen::Vector3d::Zero();
    s.accel = Eigen::Vector3d::Zero();
}

// Build Polyhedron<3> from A x <= b
static Polyhedron<3> polyFromHalfspacesSeeded(
    Eigen::MatrixXd A,
    Eigen::VectorXd b,
    const Vec3f &seed,
    double eps,
    bool *did_flip_any = nullptr)
{
    if (did_flip_any)
        *did_flip_any = false;

    // Ensure seed satisfies all inequalities by flipping rows that violate it.
    for (int r = 0; r < A.rows(); ++r)
    {
        const double v = A.row(r).dot(seed) - b(r);
        if (v > eps)
        {
            A.row(r) *= -1.0;
            b(r) *= -1.0;
            if (did_flip_any)
                *did_flip_any = true;
        }
    }

    Polyhedron<3> poly;
    for (int r = 0; r < A.rows(); ++r)
    {
        Vec3f a = A.row(r).transpose();
        const double norm = a.norm();
        if (norm < 1e-12)
            continue;

        // Inequality: a^T x <= b
        const Vec3f n = a / norm;
        const double d = b(r) / norm;
        const Vec3f p0 = n * d;

        // Hyperplane(point, normal)
        poly.add(Hyperplane<3>(p0, n));
    }

    return poly;
}

// Load one .mysco2 file and reconstruct:
// - path (vec_Vecf<3>)
// - seg_end_times
// - poly_out (vec_E<Polyhedron<3>>)
// - l_constraints (std::vector<LinearConstraint3D>)
static void loadMysco2(
    const fs::path &file,
    Vec3d &start,
    Vec3d &goal,
    vec_Vecf<3> &path,
    std::vector<double> &seg_end_times,
    vec_E<Polyhedron<3>> &poly_out,
    std::vector<LinearConstraint3D> &l_constraints,
    double poly_seed_eps, bool debug_poly_check)
{
    std::ifstream ifs(file, std::ios::binary);
    if (!ifs)
        throw std::runtime_error("Failed to open: " + file.string());

    char magic[8];
    ifs.read(magic, 8);
    if (!ifs)
        throw std::runtime_error("Corrupt .mysco2 (magic): " + file.string());

    const std::string m(magic, magic + 8);
    if (m.rfind("MYSCO2", 0) != 0)
        throw std::runtime_error("Bad magic in: " + file.string());

    const uint32_t version = readU32(ifs);
    if (version != 1)
        throw std::runtime_error("Unsupported .mysco2 version in: " + file.string());

    const double a0 = readD(ifs);
    const double a1 = readD(ifs);
    const double a2 = readD(ifs);

    const double g0 = readD(ifs);
    const double g1 = readD(ifs);
    const double g2 = readD(ifs);

    start = Vec3d(a0, a1, a2);
    goal = Vec3d(g0, g1, g2);

    const uint32_t num_path_pts = readU32(ifs);
    path.clear();
    path.reserve(num_path_pts);
    for (uint32_t i = 0; i < num_path_pts; ++i)
    {
        Vec3f p;
        const double px0 = readD(ifs);
        const double px1 = readD(ifs);
        const double px2 = readD(ifs);
        p << px0, px1, px2;
        path.push_back(p);
    }

    const uint32_t num_seg = readU32(ifs);
    seg_end_times.resize(num_seg);
    for (uint32_t i = 0; i < num_seg; ++i)
        seg_end_times[i] = readD(ifs);

    if (path.size() < 2 || (path.size() - 1) != num_seg)
        throw std::runtime_error("File inconsistent: path.size()-1 != num_seg in " + file.string());

    poly_out.clear();
    poly_out.resize(num_seg);

    l_constraints.clear();
    l_constraints.resize(num_seg);

    for (uint32_t si = 0; si < num_seg; ++si)
    {
        const uint32_t mplanes = readU32(ifs);

        Eigen::MatrixXd A(mplanes, 3);
        Eigen::VectorXd b(mplanes);

        for (uint32_t r = 0; r < mplanes; ++r)
            for (int c = 0; c < 3; ++c)
                A(r, c) = readD(ifs);

        for (uint32_t r = 0; r < mplanes; ++r)
            b(r) = readD(ifs);

        const Vec3f pt_inside = (path[si] + path[si + 1]) / 2.0;

        // Make seed-consistent halfspaces
        Eigen::MatrixXd A_fix = A;
        Eigen::VectorXd b_fix = b;
        for (int r = 0; r < A_fix.rows(); ++r)
        {
            const double v = A_fix.row(r).dot(pt_inside) - b_fix(r);
            if (v > poly_seed_eps)
            {
                A_fix.row(r) *= -1.0;
                b_fix(r) *= -1.0;
            }
        }

        poly_out[si] = polyFromHalfspacesSeeded(A_fix, b_fix, pt_inside, poly_seed_eps, nullptr);

        LinearConstraint3D lc;
        lc.A_ = A_fix;
        lc.b_ = b_fix;
        l_constraints[si] = lc;

        const double max_v = (lc.A_ * pt_inside - lc.b_).maxCoeff();
        if (max_v > 1e-4)
        {
            std::cerr << "[SFC LOAD] WARNING seg=" << si
                      << " seed violates A_fix x<=b_fix by " << max_v
                      << " file=" << file.string() << std::endl;
        }
    }
}

// ------------------------ constraint checks (sample at dt=dc) ------------------------

template <typename T, typename = void>
struct has_member_pos : std::false_type
{
};
template <typename T>
struct has_member_pos<T, std::void_t<decltype(std::declval<const T &>().pos)>> : std::true_type
{
};

template <typename T, typename = void>
struct has_member_vel : std::false_type
{
};
template <typename T>
struct has_member_vel<T, std::void_t<decltype(std::declval<const T &>().vel)>> : std::true_type
{
};

template <typename T, typename = void>
struct has_member_accel : std::false_type
{
};
template <typename T>
struct has_member_accel<T, std::void_t<decltype(std::declval<const T &>().accel)>> : std::true_type
{
};

static inline Vec3d getPosSafe(const state &s)
{
    if constexpr (has_member_pos<state>::value)
        return s.pos;
    return Vec3d::Zero();
}

static inline Vec3d getVelSafe(const state &s)
{
    if constexpr (has_member_vel<state>::value)
        return s.vel;
    return Vec3d::Zero();
}

static inline Vec3d getAccelSafe(const state &s)
{
    if constexpr (has_member_accel<state>::value)
        return s.accel;
    return Vec3d::Zero();
}

static inline double maxHalfspaceViolation(const LinearConstraint3D &lc, const Vec3f &p)
{
    if (lc.A_.rows() == 0)
        return 0.0;
    Eigen::VectorXd d = lc.A_ * p - lc.b_;
    return d.maxCoeff();
}

struct ConstraintReport
{
    double corridor_max_min_violation{0.0};
    double corridor_t_at_max{0.0};
    Vec3d corridor_p_at_max{0, 0, 0};
    int corridor_best_poly_idx_at_max{-1};

    double v_max_observed{0.0};
    double v_max_excess{0.0};
    double v_t_at_max{0.0};

    double a_max_observed{0.0};
    double a_max_excess{0.0};
    double a_t_at_max{0.0};

    double j_max_observed{0.0};
    double j_max_excess{0.0};
    double j_t_at_max{0.0};

    bool corridor_violated{false};
    bool v_violated{false};
    bool a_violated{false};
    bool j_violated{false};

    // -------- Smoothness metrics (sampled at dt=dc; jerk via finite-diff of accel) --------
    // S_jerk    = ∫_0^T ||j(t)|| dt
    // Sbar_jerk = sqrt( (1/T) ∫_0^T ||j(t)||^2 dt )   (RMS jerk; time-normalized)
    double jerk_smoothness_l1{0.0};
    double jerk_rms{0.0};

    double traj_length_m{0.0}; // ∫ ||v(t)|| dt
};

static ConstraintReport analyzeConstraintsSampled(
    const std::vector<state> &samples,
    const std::vector<LinearConstraint3D> &l_constraints,
    double dc,
    double v_max, double a_max, double j_max,
    double corridor_tol = 1e-6,
    double dyn_tol = 1e-6)
{
    ConstraintReport rep;

    if (samples.empty())
        return rep;

    const int num_poly = static_cast<int>(l_constraints.size());
    const bool have_polys = (num_poly > 0);

    auto maxAbsComponent = [](const Vec3d &x) -> double
    {
        return x.cwiseAbs().maxCoeff();
    };

    // Corridor union
    if (have_polys)
    {
        double worst = -std::numeric_limits<double>::infinity();
        double worst_t = 0.0;
        Vec3d worst_p(0, 0, 0);
        int worst_best_idx = -1;

        for (size_t k = 0; k < samples.size(); ++k)
        {
            const double t = static_cast<double>(k) * dc;
            const Vec3d p_d = getPosSafe(samples[k]);
            const Vec3f p = p_d.cast<double>();

            double best = std::numeric_limits<double>::infinity();
            int best_idx = -1;

            for (int i = 0; i < num_poly; ++i)
            {
                const double v = maxHalfspaceViolation(l_constraints[i], p);
                if (v < best)
                {
                    best = v;
                    best_idx = i;
                }
            }

            if (best > worst)
            {
                worst = best;
                worst_t = t;
                worst_p = p_d;
                worst_best_idx = best_idx;
            }
        }

        rep.corridor_max_min_violation = worst;
        rep.corridor_t_at_max = worst_t;
        rep.corridor_p_at_max = worst_p;
        rep.corridor_best_poly_idx_at_max = worst_best_idx;
        rep.corridor_violated = (worst > corridor_tol);
    }

    // Build v,a,j
    std::vector<Vec3d> vel(samples.size(), Vec3d::Zero());
    std::vector<Vec3d> acc(samples.size(), Vec3d::Zero());
    std::vector<Vec3d> jerk(samples.size(), Vec3d::Zero());

    if constexpr (has_member_vel<state>::value)
    {
        for (size_t k = 0; k < samples.size(); ++k)
            vel[k] = samples[k].vel;
    }
    else
    {
        if (samples.size() >= 2 && dc > 0.0)
        {
            vel[0] = Vec3d::Zero();
            for (size_t k = 1; k < samples.size(); ++k)
                vel[k] = (getPosSafe(samples[k]) - getPosSafe(samples[k - 1])) / dc;
        }
    }

    double length_m = 0.0;

    if (samples.size() >= 2 && dc > 0.0)
    {
        // If you have velocity vectors vel[k]
        for (size_t k = 1; k < samples.size(); ++k)
        {
            const double s0 = vel[k - 1].norm();
            const double s1 = vel[k].norm();
            length_m += 0.5 * (s0 + s1) * dc; // trapezoidal rule
        }
    }

    // store
    rep.traj_length_m = length_m;

    if constexpr (has_member_accel<state>::value)
    {
        for (size_t k = 0; k < samples.size(); ++k)
            acc[k] = samples[k].accel;
    }
    else
    {
        if (samples.size() >= 2 && dc > 0.0)
        {
            acc[0] = Vec3d::Zero();
            for (size_t k = 1; k < samples.size(); ++k)
                acc[k] = (vel[k] - vel[k - 1]) / dc;
        }
    }

    if constexpr (has_member_accel<state>::value == false)
    {
        for (size_t k = 1; k < samples.size(); ++k)
            jerk[k] = samples[k].jerk;
    }
    else
    {
        if (samples.size() >= 2 && dc > 0.0)
        {
            jerk[0] = Vec3d::Zero();
            for (size_t k = 1; k < samples.size(); ++k)
                jerk[k] = (acc[k] - acc[k - 1]) / dc;
        }
    }

    constexpr double buf = 1e-6;

    double vmax_obs = 0.0, vmax_ex = -std::numeric_limits<double>::infinity(), vmax_t = 0.0;
    double amax_obs = 0.0, amax_ex = -std::numeric_limits<double>::infinity(), amax_t = 0.0;
    double jmax_obs = 0.0, jmax_ex = -std::numeric_limits<double>::infinity(), jmax_t = 0.0;

    for (size_t k = 0; k < samples.size(); ++k)
    {
        const double t = static_cast<double>(k) * dc;

        const double vcomp_max = maxAbsComponent(vel[k]);
        vmax_obs = std::max(vmax_obs, vcomp_max);
        const double vex = vcomp_max - (v_max + buf);
        if (vex > vmax_ex)
        {
            vmax_ex = vex;
            vmax_t = t;
        }

        const double acomp_max = maxAbsComponent(acc[k]);
        amax_obs = std::max(amax_obs, acomp_max);
        const double aex = acomp_max - (a_max + buf);
        if (aex > amax_ex)
        {
            amax_ex = aex;
            amax_t = t;
        }

        const double jcomp_max = maxAbsComponent(jerk[k]);
        jmax_obs = std::max(jmax_obs, jcomp_max);
        const double jex = jcomp_max - (j_max + buf);
        if (jex > jmax_ex)
        {
            jmax_ex = jex;
            jmax_t = t;
        }
    }

    rep.v_max_observed = vmax_obs;
    rep.v_max_excess = std::max(0.0, vmax_ex);
    rep.v_t_at_max = vmax_t;
    rep.v_violated = (vmax_ex > dyn_tol);

    rep.a_max_observed = amax_obs;
    rep.a_max_excess = std::max(0.0, amax_ex);
    rep.a_t_at_max = amax_t;
    rep.a_violated = (amax_ex > dyn_tol);

    rep.j_max_observed = jmax_obs;
    rep.j_max_excess = std::max(0.0, jmax_ex);
    rep.j_t_at_max = jmax_t;
    rep.j_violated = (jmax_ex > dyn_tol);

    double jerk_l1 = 0.0;     // ∫ ||j|| dt   (approx)
    double jerk_l2_int = 0.0; // ∫ ||j||^2 dt (approx)

    if (samples.size() >= 2 && dc > 0.0)
    {
        for (size_t k = 1; k < samples.size(); ++k)
        {
            // Smoothness metrics use Euclidean norm.
            const double jnorm = jerk[k].norm();
            jerk_l1 += jnorm * dc;
            jerk_l2_int += (jnorm * jnorm) * dc;
        }
    }

    // Smoothness results.
    // Use the sampled trajectory duration implied by samples + dc for numerical consistency.
    const double T = (samples.size() >= 2 && dc > 0.0) ? (static_cast<double>(samples.size() - 1) * dc) : 0.0;
    rep.jerk_smoothness_l1 = jerk_l1;
    rep.jerk_rms = (T > 0.0) ? std::sqrt(jerk_l2_int / T) : 0.0;

    return rep;
}

auto applyConstraintReport = [](BenchResult &out, const ConstraintReport &rep)
{
    out.corridor_max_min_violation = rep.corridor_max_min_violation;
    out.corridor_t_at_max = rep.corridor_t_at_max;
    out.corridor_p_at_max = rep.corridor_p_at_max;
    out.corridor_best_poly_idx_at_max = rep.corridor_best_poly_idx_at_max;

    out.v_max_observed = rep.v_max_observed;
    out.v_max_excess = rep.v_max_excess;
    out.v_t_at_max = rep.v_t_at_max;

    out.a_max_observed = rep.a_max_observed;
    out.a_max_excess = rep.a_max_excess;
    out.a_t_at_max = rep.a_t_at_max;

    out.j_max_observed = rep.j_max_observed;
    out.j_max_excess = rep.j_max_excess;
    out.j_t_at_max = rep.j_t_at_max;

    out.corridor_violated = rep.corridor_violated;
    out.v_violated = rep.v_violated;
    out.a_violated = rep.a_violated;
    out.j_violated = rep.j_violated;

    out.jerk_smoothness_l1 = rep.jerk_smoothness_l1;
    out.jerk_rms = rep.jerk_rms;

    out.traj_length_m = rep.traj_length_m;
};
// ------------------------ trajectory dump helpers (NEW) ------------------------

static inline bool ensureDir(const fs::path &p)
{
    std::error_code ec;
    if (fs::exists(p, ec))
        return fs::is_directory(p, ec);
    return fs::create_directories(p, ec);
}

static inline std::string sanitizeFilename(std::string s)
{
    for (char &c : s)
    {
        const unsigned char uc = static_cast<unsigned char>(c);
        if (!(std::isalnum(uc) || c == '-' || c == '_' || c == '.'))
            c = '_';
    }
    return s;
}

// Dump goal_setpoints (sampled at dt=dc) to CSV.
// If traj_dump_dt > dc, downsample by stride = round(traj_dump_dt/dc), clamped >= 1.
static void dumpTrajectoryCsvV1(
    const fs::path &out_csv,
    const std::string &planner_name,
    const std::string &case_file_basename,
    const std::string &frame_id,
    const std::vector<state> &samples,
    double dc,
    double traj_dump_dt_requested,
    double factor_used,
    double cost_value,
    double total_traj_time_sec)
{
    if (samples.empty())
        return;

    if (dc <= 0.0)
        dc = 0.01;

    int stride = 1;
    if (traj_dump_dt_requested > 0.0)
    {
        const double ratio = traj_dump_dt_requested / dc;
        const long long s = llround(ratio);
        stride = static_cast<int>(std::max<long long>(1LL, s));
    }
    const double dump_dt = stride * dc;

    std::ofstream ofs(out_csv);
    if (!ofs)
        throw std::runtime_error("Failed to open traj csv for write: " + out_csv.string());

    ofs << "# traj_format: mighty_local_traj_csv_v1\n";
    ofs << "# planner_name: " << planner_name << "\n";
    ofs << "# case_file: " << case_file_basename << "\n";
    ofs << "# frame_id: " << frame_id << "\n";
    ofs << "# dc_sec: " << std::fixed << std::setprecision(9) << dc << "\n";
    ofs << "# dump_dt_sec: " << std::fixed << std::setprecision(9) << dump_dt << "\n";
    ofs << "# factor_used: " << std::fixed << std::setprecision(9) << factor_used << "\n";
    ofs << "# cost_value: " << std::fixed << std::setprecision(9) << cost_value << "\n";
    ofs << "# total_traj_time_sec: " << std::fixed << std::setprecision(9) << total_traj_time_sec << "\n";
    ofs << "t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz\n";

    ofs << std::fixed << std::setprecision(9);

    Vec3d prev_a = getAccelSafe(samples.front());
    for (size_t k = 0; k < samples.size(); k += static_cast<size_t>(stride))
    {
        const double t = static_cast<double>(k) * dc;

        const Vec3d p = getPosSafe(samples[k]);
        const Vec3d v = getVelSafe(samples[k]);
        const Vec3d a = getAccelSafe(samples[k]);

        Vec3d j = Vec3d::Zero();
        if (k > 0 && dc > 0.0)
            j = (a - prev_a) / dc;

        // Update prev_a for next jerk computation (even if downsampling)
        prev_a = a;

        ofs << t << ","
            << p.x() << "," << p.y() << "," << p.z() << ","
            << v.x() << "," << v.y() << "," << v.z() << ","
            << a.x() << "," << a.y() << "," << a.z() << ","
            << j.x() << "," << j.y() << "," << j.z()
            << "\n";
    }

    ofs.flush();
}

// ------------------------ node ------------------------

class LocalTrajBenchmarkNode final : public rclcpp::Node
{
public:
    LocalTrajBenchmarkNode() : Node("local_traj_benchmark_node")
    {

        // Declar parameters
        // I/O + visualization
        declare_parameter<std::string>("sfc_dir", "/home/kkondo/code/mighty_ws/src/mighty/data");
        declare_parameter<std::string>("file_ext", ".mysco2");
        declare_parameter<std::string>("frame_id", "map");

        declare_parameter<std::string>("poly_topic", "/NX01/poly_safe");
        declare_parameter<std::string>("traj_committed_topic", "/NX01/traj_committed_colored");
        declare_parameter<std::string>("dgp_path_topic", "/NX01/dgp_path_marker");

        declare_parameter<bool>("visualize", true);
        declare_parameter<double>("playback_period_sec", 0.1);
        declare_parameter<bool>("latched", true);

        // NEW: trajectory dump settings
        declare_parameter<bool>("traj_dump_enable", true);
        declare_parameter<std::string>("traj_dump_root_dir", "");
        declare_parameter<double>("traj_dump_dt", -1.0);

        // Solver control
        declare_parameter<double>("assumed_last_replan_time_sec", 0.05);
        declare_parameter<double>("factor_constant_step_size", 0.1);
        declare_parameter<double>("max_gurobi_comp_time_sec", 5.0);
        declare_parameter<double>("jerk_smooth_weight", -1.0e+1);
        declare_parameter<double>("goal_pull_weight", 0.0);
        declare_parameter<double>("goal_pull_time_buffer", 1.5);


        declare_parameter<double>("poly_seed_eps", 1e-6);
        declare_parameter<bool>("debug_poly_check", true);

        declare_parameter<std::vector<std::string>>("planner_names", std::vector<std::string>{});
        declare_parameter<std::vector<int64_t>>("num_N_list", std::vector<int64_t>{});
        declare_parameter<std::vector<double>>("factor_initial_list", std::vector<double>{2.0, 1.0, 1.0});
        declare_parameter<std::vector<double>>("factor_final_list", std::vector<double>{4.0, 3.0, 2.0});

        declare_parameter<std::string>("planner_name", "mighty");
        declare_parameter<bool>("use_single_threaded", false);

        // ---------------- Vehicle / mode ----------------
        this->declare_parameter<std::string>("vehicle_type", "UAV");
        this->declare_parameter<bool>("provide_goal_in_global_frame", true);
        this->declare_parameter<bool>("use_hardware", false);

        this->declare_parameter<std::string>("flight_mode", "AUTO");
        this->declare_parameter<int>("visual_level", 1);

        // ---------------- Global planner ----------------
        this->declare_parameter<std::string>("global_planner", "dgp");
        this->declare_parameter<bool>("global_planner_verbose", false);
        this->declare_parameter<double>("global_planner_huristic_weight", 1.0);
        this->declare_parameter<double>("factor_dgp", 1.0);
        this->declare_parameter<double>("inflation_dgp", 0.3);

        declare_parameter<double>("x_min", -100.0);
        declare_parameter<double>("x_max", 100.0);
        declare_parameter<double>("y_min", -100.0);
        declare_parameter<double>("y_max", 100.0);
        declare_parameter<double>("z_min", 0.0);
        declare_parameter<double>("z_max", 5.0);

        this->declare_parameter<int>("dgp_timeout_duration_ms", 2000);
        this->declare_parameter<bool>("use_free_start", false);
        this->declare_parameter<double>("free_start_factor", 1.0);
        this->declare_parameter<bool>("use_free_goal", false);
        this->declare_parameter<double>("free_goal_factor", 1.0);

        this->declare_parameter<int>("num_N", 6);
        this->declare_parameter<double>("max_dist_vertexes", 5.0);
        this->declare_parameter<double>("w_unknown", 1.0);
        this->declare_parameter<double>("w_align", 1.0);
        this->declare_parameter<double>("decay_len_cells", 3.0);
        this->declare_parameter<double>("w_side", 1.0);

        // ---------------- LOS ----------------
        this->declare_parameter<int>("los_cells", 5);
        this->declare_parameter<double>("min_len", 0.2);
        this->declare_parameter<double>("min_turn", 0.1);

        // ---------------- Visualization ----------------
        this->declare_parameter<bool>("use_state_update", false);
        this->declare_parameter<bool>("use_random_color_for_global_path", true);
        this->declare_parameter<bool>("use_path_push_for_visualization", true);

        // ---------------- Decomposition ----------------
        this->declare_parameter<std::vector<double>>("local_box_size", {5.0, 5.0, 3.0});
        this->declare_parameter<double>("min_dist_from_agent_to_traj", 0.5);
        this->declare_parameter<bool>("use_shrinked_box", false);
        this->declare_parameter<double>("shrinked_box_size", 0.8);

        // ---------------- Map ----------------
        this->declare_parameter<double>("map_buffer", 2.0);
        this->declare_parameter<double>("center_shift_factor", 0.5);
        this->declare_parameter<double>("initial_wdx", 5.0);
        this->declare_parameter<double>("initial_wdy", 5.0);
        this->declare_parameter<double>("initial_wdz", 3.0);
        this->declare_parameter<double>("min_wdx", 2.0);
        this->declare_parameter<double>("min_wdy", 2.0);
        this->declare_parameter<double>("min_wdz", 1.0);
        this->declare_parameter<double>("mighty_map_res", 0.2);

        // ---------------- Comm delay ----------------
        this->declare_parameter<bool>("use_comm_delay_inflation", false);
        this->declare_parameter<double>("comm_delay_inflation_alpha", 0.5);
        this->declare_parameter<double>("comm_delay_inflation_max", 1.0);
        this->declare_parameter<double>("comm_delay_filter_alpha", 0.5);

        // ---------------- Simulation ----------------
        this->declare_parameter<double>("depth_camera_depth_max", 10.0);
        this->declare_parameter<double>("fov_visual_depth", 10.0);
        this->declare_parameter<double>("fov_visual_x_deg", 90.0);
        this->declare_parameter<double>("fov_visual_y_deg", 60.0);

        // ---------------- Initial guess ----------------
        this->declare_parameter<bool>("use_multiple_initial_guesses", false);
        this->declare_parameter<int>("num_perturbation_for_ig", 5);
        this->declare_parameter<double>("r_max_for_ig", 0.5);

        // ---------------- Optimization ----------------
        this->declare_parameter<double>("horizon", 5.0);
        this->declare_parameter<double>("dc", 0.05);
        declare_parameter<double>("v_max", 1.0);
        declare_parameter<double>("a_max", 2.0);
        declare_parameter<double>("j_max", 3.0);

        this->declare_parameter<bool>("closed_form_traj_verbose", false);
        this->declare_parameter<double>("jerk_weight", 1.0);
        this->declare_parameter<double>("dynamic_weight", 1.0);
        this->declare_parameter<double>("time_weight", 1.0);
        this->declare_parameter<double>("pos_anchor_weight", 1.0);
        this->declare_parameter<double>("stat_weight", 1.0);

        this->declare_parameter<double>("dyn_constr_bodyrate_weight", 1.0);
        this->declare_parameter<double>("dyn_constr_tilt_weight", 1.0);
        this->declare_parameter<double>("dyn_constr_thrust_weight", 1.0);
        this->declare_parameter<double>("dyn_constr_vel_weight", 1.0);
        this->declare_parameter<double>("dyn_constr_acc_weight", 1.0);
        this->declare_parameter<double>("dyn_constr_jerk_weight", 1.0);

        this->declare_parameter<int>("num_dyn_obst_samples", 5);
        this->declare_parameter<double>("planner_Co", 1.0);
        this->declare_parameter<double>("planner_Cw", 1.0);

        this->declare_parameter<std::vector<double>>("drone_bbox", {0.6, 0.6, 0.3});
        this->declare_parameter<double>("goal_radius", 0.3);
        this->declare_parameter<double>("goal_seen_radius", 1.0);
        this->declare_parameter<double>("init_turn_bf", 0.5);
        this->declare_parameter<int>("integral_resolution", 20);

        this->declare_parameter<double>("hinge_mu", 1.0);
        this->declare_parameter<double>("omega_max", 2.0);
        this->declare_parameter<double>("tilt_max_rad", 0.6);
        this->declare_parameter<double>("f_min", 0.0);
        this->declare_parameter<double>("f_max", 20.0);
        this->declare_parameter<double>("mass", 1.5);
        this->declare_parameter<double>("g", 9.81);

        this->declare_parameter<double>("fopt_threshold", 1e6);

        // ---------------- LBFGS ----------------
        this->declare_parameter<double>("f_dec_coeff", 1e-4);
        this->declare_parameter<double>("cautious_factor", 0.9);
        this->declare_parameter<int>("past", 10);
        this->declare_parameter<int>("max_linesearch", 40);
        this->declare_parameter<int>("max_iterations", 200);
        this->declare_parameter<double>("g_epsilon", 1e-6);
        this->declare_parameter<double>("delta", 1e-6);

        // ---------------- Dynamic obstacles ----------------
        this->declare_parameter<double>("traj_lifetime", 5.0);

        // ---------------- k-value ----------------
        this->declare_parameter<int>("num_replanning_before_adapt", 5);
        this->declare_parameter<int>("default_k_value", 3);
        this->declare_parameter<double>("alpha_k_value_filtering", 0.5);
        this->declare_parameter<double>("k_value_factor", 1.0);

        // ---------------- Yaw ----------------
        this->declare_parameter<double>("alpha_filter_dyaw", 0.5);
        this->declare_parameter<double>("w_max", 2.0);
        this->declare_parameter<int>("yaw_spinning_threshold", 10);
        this->declare_parameter<double>("yaw_spinning_dyaw", 1.0);

        // ---------------- Simulation env ----------------
        this->declare_parameter<bool>("force_goal_z", false);
        this->declare_parameter<double>("default_goal_z", 1.0);

        // ---------------- Debug ----------------
        this->declare_parameter<bool>("debug_verbose", false);

        // ----------------- Use occlusion cost --------------
        this->declare_parameter<bool>("use_occ_cost", true);


        ///////////////////////////////////////////////////////////////////////////



        // poly_seed_eps_ = get_parameter("poly_seed_eps").as_double();
        // debug_poly_check_ = get_parameter("debug_poly_check").as_bool();

        // Create map subscription (static map)
        // Synchronize the occupancy grid and unknown grid
        this->cb_group_map_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options_map;
        options_map.callback_group = this->cb_group_map_;
        occup_grid_sub_.subscribe(this, "occupancy_grid", rmw_qos_profile_sensor_data, options_map);
        unknown_grid_sub_.subscribe(this, "unknown_grid", rmw_qos_profile_sensor_data, options_map);
        sync_.reset(new Sync(MySyncPolicy(10), occup_grid_sub_, unknown_grid_sub_));
        sync_->registerCallback(std::bind(&LocalTrajBenchmarkNode::mapCallback, this, std::placeholders::_1, std::placeholders::_2));

        // To run the planning/solving
        planning_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&LocalTrajBenchmarkNode::solveAll, this));

    }

private:

    // Read in the pointcloud and update the map (this should only occur once since 
    // the benchmarking is for a single trajectory per polytope in a static snapshot)
    void mapCallback(
        const sensor_msgs::msg::PointCloud2::ConstPtr &map_msg,
        const sensor_msgs::msg::PointCloud2::ConstPtr &unk_msg)
    {
        // Wait until parameters and pointer for solver (mighty) were set
        if (!solvers_initialized_){
            RCLCPP_WARN(get_logger(), "updateMap called before MIGHTY initialized");
            return;
        }
        // If the map was already updated, no longer need to keep executing callback
        if (map_ready_){
            return;
        }
        // use PCL’s own Ptr (boost::shared_ptr)
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*map_msg, *map_pc);

        pcl::PointCloud<pcl::PointXYZ>::Ptr unk_pc(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*unk_msg, *unk_pc);
        RCLCPP_INFO(get_logger(), "going to update map.");
        mighty_ptr_->updateMap(map_pc, unk_pc);
        RCLCPP_INFO(get_logger(), "updated map.");
        map_ready_ = true;
        RCLCPP_INFO(get_logger(), "Map received and stored.");
    }

    void solveAll(){

        if (!solvers_initialized_){
            initializeSolvers();
        }
        if (!map_ready_){
            // RCLCPP_INFO(get_logger(), "map not received");
            return;
        }
        if (planning_started_){
            return;
        }
        planning_started_ = true;
        // Read params
        std::vector<int64_t> num_N_list_64 = this->get_parameter("num_N_list").as_integer_array();
        std::vector<std::string> planner_names = this->get_parameter("planner_names").as_string_array();
        if (planner_names.empty())
            planner_names.push_back(this->get_parameter("planner_name").as_string());

        std::vector<int> num_N_list;
        num_N_list.reserve(num_N_list_64.size());
        for (auto v : num_N_list_64)
            num_N_list.push_back(static_cast<int>(v));

        if (num_N_list.empty())
            num_N_list.push_back(this->get_parameter("num_N").as_int());

        std::vector<double> factor_initial_list = this->get_parameter("factor_initial_list").as_double_array();
        std::vector<double> factor_final_list = this->get_parameter("factor_final_list").as_double_array();

        use_single_threaded_ = get_parameter("use_single_threaded").as_bool();
        sfc_dir_ = get_parameter("sfc_dir").as_string();
        file_ext_ = get_parameter("file_ext").as_string();
        frame_id_ = get_parameter("frame_id").as_string();

        poly_topic_ = get_parameter("poly_topic").as_string();
        traj_committed_topic_ = get_parameter("traj_committed_topic").as_string();
        dgp_path_topic_ = get_parameter("dgp_path_topic").as_string();

        visualize_ = get_parameter("visualize").as_bool();
        playback_period_sec_ = get_parameter("playback_period_sec").as_double();
        latched_ = get_parameter("latched").as_bool();

        assumed_last_replan_time_sec_ = get_parameter("assumed_last_replan_time_sec").as_double();

        // Dump params (NEW)
        traj_dump_enable_ = get_parameter("traj_dump_enable").as_bool();
        traj_dump_root_dir_ = get_parameter("traj_dump_root_dir").as_string();
        traj_dump_dt_ = get_parameter("traj_dump_dt").as_double();
        for (const auto &planner_name : planner_names)
        {
            for (int idx = 0; idx < (int)num_N_list.size(); ++idx)
            {
                int num_N = num_N_list[idx];
                planner_name_ = planner_name;
                par_.num_N = num_N;

                std::string occ_identifier;
                if (use_occ_cost_==true){
                    occ_identifier = "with_occ_";
                }
                else{
                    occ_identifier = "";
                }
                std::string thread_string = use_single_threaded_ ? "single_thread" : "multi_thread";
                csv_out_ = "/home/kkondo/code/mighty_ws/src/mighty/benchmark_data/" + thread_string + "/" +
                           planner_name_ + "_" + std::to_string(par_.num_N) + occ_identifier + "_benchmark.csv";

                // NEW: derive dump directory for this run
                if (traj_dump_enable_)
                {
                    fs::path root;
                    if (!traj_dump_root_dir_.empty())
                        root = fs::path(traj_dump_root_dir_);
                    else
                        root = fs::path(csv_out_).parent_path() / "traj_dump";

                    std::string occ_identifier;
                    if (use_occ_cost_){
                        occ_identifier = "with_occ_";
                    }
                    else{
                        occ_identifier = "";
                    }

                    traj_dump_run_dir_ = (root / (planner_name_ + "_N" + std::to_string(par_.num_N) + occ_identifier)).string();

                    if (!ensureDir(fs::path(traj_dump_run_dir_)))
                    {
                        RCLCPP_WARN(get_logger(), "Failed to create traj dump dir: %s (disabling dump for this run)",
                                    traj_dump_run_dir_.c_str());
                        traj_dump_enable_this_run_ = false;
                    }
                    else
                    {
                        traj_dump_enable_this_run_ = true;
                        RCLCPP_INFO(get_logger(), "Trajectory dump enabled. dir=%s dt_req=%.4f (dc=%.4f)",
                                    traj_dump_run_dir_.c_str(), traj_dump_dt_, par_.dc);
                    }
                }
                else
                {
                    traj_dump_enable_this_run_ = false;
                }
                // Publishers
                rclcpp::QoS qos(rclcpp::KeepLast(1));
                qos.reliable();
                if (latched_)
                    qos.transient_local();
                pub_poly_ = create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(poly_topic_, qos);
                pub_traj_committed_colored_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                    traj_committed_topic_, qos); // qos was originally 10 
                pub_dgp_path_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                    dgp_path_topic_, qos);
                // Load + solve
                loadAll();
                solvePlanner(planner_name); // run the optimization and save results
                writeCsv();
                if (visualize_ && !results_.empty())
                {
                    playback_timer_ = create_wall_timer(
                        std::chrono::duration<double>(std::max(0.05, playback_period_sec_)),
                        std::bind(&LocalTrajBenchmarkNode::publishNext, this));
                }
                const fs::path out_dir = fs::path(traj_dump_run_dir_);
                std::cout << "Trajectories dumped to directory: " << out_dir.string();

            }
        }

    }

    void loadAll()
    {
        results_.clear();

        if (!fs::exists(sfc_dir_))
            throw std::runtime_error("sfc_dir does not exist: " + sfc_dir_);

        std::vector<fs::path> files;
        for (const auto &ent : fs::directory_iterator(sfc_dir_))
        {
            if (!ent.is_regular_file())
                continue;
            const auto p = ent.path();
            if (p.extension() == file_ext_)
                files.push_back(p);
        }

        std::sort(files.begin(), files.end());

        for (const auto &f : files)
        {
            BenchResult r;
            r.file = f.string();
            results_.push_back(std::move(r));
        }
    }

    void maybeDumpTrajectory(const std::string &case_fname,
                             const std::vector<state> &goal_setpoints,
                             const BenchResult &r)
    {
        if (!traj_dump_enable_this_run_)
            return;

        const std::string base = sanitizeFilename("traj_" + planner_name_ + "_N" + std::to_string(par_.num_N) + "__" + case_fname);
        
        const fs::path out_csv = fs::path(traj_dump_run_dir_) / (base + ".csv");

        dumpTrajectoryCsvV1(out_csv,
                            planner_name_,
                            case_fname,
                            frame_id_,
                            goal_setpoints,
                            par_.dc,
                            traj_dump_dt_,
                            r.factor_used,
                            r.cost_value,
                            r.total_traj_time_sec);
    }

    void solvePlanner(std::string solver_name){
        if (solver_name == "mighty"){
            // loop over all the safety corridors from the saved files
            for (auto &r : results_)
            {
                auto t0 = std::chrono::high_resolution_clock::now();
                const std::string fname = fs::path(r.file).filename().string();
                bool ok = solveMighty(r);
                auto t1 = std::chrono::high_resolution_clock::now();
                r.success = ok;
            }

        }
    }

    void initializeSolvers(){
        if (solvers_initialized_){
            return;
        }

        std::vector<std::string> planner_names = this->get_parameter("planner_names").as_string_array();
        if (planner_names.empty())
            planner_names.push_back(this->get_parameter("planner_name").as_string());

        for (const auto &name: planner_names){
            if (name == "mighty"){
                initializeMightySolver();
            }
        }
        solvers_initialized_ = true;
    }
    void initializeMightySolver(){

        // Set the parameters
        // Vehicle type (UAV, Wheeled Robit, or Quadruped)
        par_.vehicle_type = this->get_parameter("vehicle_type").as_string();
        par_.provide_goal_in_global_frame = this->get_parameter("provide_goal_in_global_frame").as_bool();
        par_.use_hardware = this->get_parameter("use_hardware").as_bool();

        // Flight mode
        par_.flight_mode = this->get_parameter("flight_mode").as_string();

        // Visual level
        par_.visual_level = this->get_parameter("visual_level").as_int();

        // Global Planner parameters
        par_.global_planner = this->get_parameter("global_planner").as_string();
        par_.global_planner_verbose = this->get_parameter("global_planner_verbose").as_bool();
        par_.global_planner_huristic_weight = this->get_parameter("global_planner_huristic_weight").as_double();
        par_.factor_dgp = this->get_parameter("factor_dgp").as_double();
        par_.inflation_dgp = this->get_parameter("inflation_dgp").as_double();
        par_.x_min = this->get_parameter("x_min").as_double();
        par_.x_max = this->get_parameter("x_max").as_double();
        par_.y_min = this->get_parameter("y_min").as_double();
        par_.y_max = this->get_parameter("y_max").as_double();
        par_.z_min = this->get_parameter("z_min").as_double();
        par_.z_max = this->get_parameter("z_max").as_double();
        par_.dgp_timeout_duration_ms = this->get_parameter("dgp_timeout_duration_ms").as_int();
        par_.use_free_start = this->get_parameter("use_free_start").as_bool();
        par_.free_start_factor = this->get_parameter("free_start_factor").as_double();
        par_.use_free_goal = this->get_parameter("use_free_goal").as_bool();
        par_.free_goal_factor = this->get_parameter("free_goal_factor").as_double();
        par_.num_N = this->get_parameter("num_N").as_int();
        par_.max_dist_vertexes = this->get_parameter("max_dist_vertexes").as_double();
        par_.w_unknown = this->get_parameter("w_unknown").as_double();
        par_.w_align = this->get_parameter("w_align").as_double();
        par_.decay_len_cells = this->get_parameter("decay_len_cells").as_double();
        par_.w_side = this->get_parameter("w_side").as_double();

        // LOS post processing parameters
        par_.los_cells = this->get_parameter("los_cells").as_int();
        par_.min_len = this->get_parameter("min_len").as_double();
        par_.min_turn = this->get_parameter("min_turn").as_double();

        // Path push visualization parameters
        par_.use_state_update = this->get_parameter("use_state_update").as_bool();
        par_.use_random_color_for_global_path = this->get_parameter("use_random_color_for_global_path").as_bool();
        par_.use_path_push_for_visualization = this->get_parameter("use_path_push_for_visualization").as_bool();

        // Static obstacle push parameters

        // Decomposition parameters
        par_.local_box_size = this->get_parameter("local_box_size").as_double_array();
        par_.min_dist_from_agent_to_traj = this->get_parameter("min_dist_from_agent_to_traj").as_double();
        par_.use_shrinked_box = this->get_parameter("use_shrinked_box").as_bool();
        par_.shrinked_box_size = this->get_parameter("shrinked_box_size").as_double();

        // Map parameters
        par_.map_buffer = this->get_parameter("map_buffer").as_double();
        par_.center_shift_factor = this->get_parameter("center_shift_factor").as_double();
        par_.initial_wdx = this->get_parameter("initial_wdx").as_double();
        par_.initial_wdy = this->get_parameter("initial_wdy").as_double();
        par_.initial_wdz = this->get_parameter("initial_wdz").as_double();
        par_.min_wdx = this->get_parameter("min_wdx").as_double();
        par_.min_wdy = this->get_parameter("min_wdy").as_double();
        par_.min_wdz = this->get_parameter("min_wdz").as_double();
        par_.res = this->get_parameter("mighty_map_res").as_double();

        // Communication delay parameters
        par_.use_comm_delay_inflation = this->get_parameter("use_comm_delay_inflation").as_bool();
        par_.comm_delay_inflation_alpha = this->get_parameter("comm_delay_inflation_alpha").as_double();
        par_.comm_delay_inflation_max = this->get_parameter("comm_delay_inflation_max").as_double();
        par_.comm_delay_filter_alpha = this->get_parameter("comm_delay_filter_alpha").as_double();

        // Simulation parameters
        par_.depth_camera_depth_max = this->get_parameter("depth_camera_depth_max").as_double();
        par_.fov_visual_depth = this->get_parameter("fov_visual_depth").as_double();
        par_.fov_visual_x_deg = this->get_parameter("fov_visual_x_deg").as_double();
        par_.fov_visual_y_deg = this->get_parameter("fov_visual_y_deg").as_double();

        // Initial guess parameters
        par_.use_multiple_initial_guesses = this->get_parameter("use_multiple_initial_guesses").as_bool();
        par_.num_perturbation_for_ig = this->get_parameter("num_perturbation_for_ig").as_int();
        par_.r_max_for_ig = this->get_parameter("r_max_for_ig").as_double();

        // Optimization parameters
        par_.horizon = this->get_parameter("horizon").as_double();
        par_.dc = this->get_parameter("dc").as_double();
        par_.v_max = this->get_parameter("v_max").as_double();
        par_.a_max = this->get_parameter("a_max").as_double();
        par_.j_max = this->get_parameter("j_max").as_double();
        par_.closed_form_traj_verbose = this->get_parameter("closed_form_traj_verbose").as_bool();
        par_.jerk_weight = this->get_parameter("jerk_weight").as_double();
        par_.dynamic_weight = this->get_parameter("dynamic_weight").as_double();
        par_.time_weight = this->get_parameter("time_weight").as_double();
        par_.pos_anchor_weight = this->get_parameter("pos_anchor_weight").as_double();
        par_.stat_weight = this->get_parameter("stat_weight").as_double();
        par_.dyn_constr_bodyrate_weight = this->get_parameter("dyn_constr_bodyrate_weight").as_double();
        par_.dyn_constr_tilt_weight = this->get_parameter("dyn_constr_tilt_weight").as_double();
        par_.dyn_constr_thrust_weight = this->get_parameter("dyn_constr_thrust_weight").as_double();
        par_.dyn_constr_vel_weight = this->get_parameter("dyn_constr_vel_weight").as_double();
        par_.dyn_constr_acc_weight = this->get_parameter("dyn_constr_acc_weight").as_double();
        par_.dyn_constr_jerk_weight = this->get_parameter("dyn_constr_jerk_weight").as_double();
        par_.num_dyn_obst_samples = this->get_parameter("num_dyn_obst_samples").as_int();
        par_.planner_Co = this->get_parameter("planner_Co").as_double();
        par_.planner_Cw = this->get_parameter("planner_Cw").as_double();
        // verbose_computation_time_ = this->get_parameter("verbose_computation_time").as_bool();
        par_.drone_bbox = this->get_parameter("drone_bbox").as_double_array();
        par_.drone_radius = par_.drone_bbox[0] / 2.0;
        par_.goal_radius = this->get_parameter("goal_radius").as_double();
        par_.goal_seen_radius = this->get_parameter("goal_seen_radius").as_double();
        par_.init_turn_bf = this->get_parameter("init_turn_bf").as_double();
        par_.integral_resolution = this->get_parameter("integral_resolution").as_int();
        par_.hinge_mu = this->get_parameter("hinge_mu").as_double();
        par_.omega_max = this->get_parameter("omega_max").as_double();
        par_.tilt_max_rad = this->get_parameter("tilt_max_rad").as_double();
        par_.f_min = this->get_parameter("f_min").as_double();
        par_.f_max = this->get_parameter("f_max").as_double();
        par_.mass = this->get_parameter("mass").as_double();
        par_.g = this->get_parameter("g").as_double();
        par_.fopt_threshold = this->get_parameter("fopt_threshold").as_double();

        // L-BFGS parameters
        par_.f_dec_coeff = this->get_parameter("f_dec_coeff").as_double();
        par_.cautious_factor = this->get_parameter("cautious_factor").as_double();
        par_.past = this->get_parameter("past").as_int();
        par_.max_linesearch = this->get_parameter("max_linesearch").as_int();
        par_.max_iterations = this->get_parameter("max_iterations").as_int();
        par_.g_epsilon = this->get_parameter("g_epsilon").as_double();
        par_.delta = this->get_parameter("delta").as_double();

        // Dynamic obstacles parameters
        par_.traj_lifetime = this->get_parameter("traj_lifetime").as_double();

        // Dynamic k_value parameters
        par_.num_replanning_before_adapt = this->get_parameter("num_replanning_before_adapt").as_int();
        par_.default_k_value = this->get_parameter("default_k_value").as_int();
        par_.alpha_k_value_filtering = this->get_parameter("alpha_k_value_filtering").as_double();
        par_.k_value_factor = this->get_parameter("k_value_factor").as_double();

        // Yaw-related parameters
        par_.alpha_filter_dyaw = this->get_parameter("alpha_filter_dyaw").as_double();
        par_.w_max = this->get_parameter("w_max").as_double();
        par_.yaw_spinning_threshold = this->get_parameter("yaw_spinning_threshold").as_int();
        par_.yaw_spinning_dyaw = this->get_parameter("yaw_spinning_dyaw").as_double();

        // Simulation env parameters
        par_.force_goal_z = this->get_parameter("force_goal_z").as_bool();
        par_.default_goal_z = this->get_parameter("default_goal_z").as_double();

        if (par_.default_goal_z <= par_.z_min)
        {
            RCLCPP_ERROR(this->get_logger(), "Default goal z is lower than the ground level");
        }

        if (par_.default_goal_z >= par_.z_max)
        {
            RCLCPP_ERROR(this->get_logger(), "Default goal z is higher than the max level");
        }

        // Debug flag
        par_.debug_verbose = this->get_parameter("debug_verbose").as_bool();

        // Set up the planner parameters (TODO: move to parameters)
        planner_params_.verbose = false;                                 // enable verbose output
        planner_params_.V_max = par_.v_max;                              // max velocity
        planner_params_.A_max = par_.a_max;                              // max acceleration
        planner_params_.J_max = par_.j_max;                              // max jerk
        planner_params_.num_perturbation = par_.num_perturbation_for_ig; // number of perturbations for initial guesses
        planner_params_.r_max = par_.r_max_for_ig;                       // perturbation radius for initial guesses
        planner_params_.time_weight = par_.time_weight;                  // weight for time cost
        planner_params_.pos_anchor_weight = par_.pos_anchor_weight;
        planner_params_.dyn_weight = par_.dynamic_weight;
        planner_params_.stat_weight = par_.stat_weight;
        planner_params_.jerk_weight = par_.jerk_weight;
        planner_params_.dyn_constr_vel_weight = par_.dyn_constr_vel_weight;
        planner_params_.dyn_constr_acc_weight = par_.dyn_constr_acc_weight;
        planner_params_.dyn_constr_jerk_weight = par_.dyn_constr_jerk_weight;
        planner_params_.dyn_constr_bodyrate_weight = par_.dyn_constr_bodyrate_weight;
        planner_params_.dyn_constr_tilt_weight = par_.dyn_constr_tilt_weight;
        planner_params_.dyn_constr_thrust_weight = par_.dyn_constr_thrust_weight;
        planner_params_.num_dyn_obst_samples = par_.num_dyn_obst_samples; // Number of dynamic obstacle samples
        planner_params_.Co = par_.planner_Co;                             // for static obstacle avoidance
        planner_params_.Cw = par_.planner_Cw;                             // for dynamic obstacle avoidance
        planner_params_.BIG = 1e8;
        planner_params_.dc = par_.dc;                                             // descretiation constant
        planner_params_.init_turn_bf = par_.init_turn_bf;
        
        use_occ_cost_ = this->get_parameter("use_occ_cost").as_bool();
        RCLCPP_INFO(get_logger(), "useing occ cost %d", use_occ_cost_);
        mighty_ptr_ = std::make_shared<MIGHTY>(par_);
    }
    



    // this is for a single corridor
    bool solveMighty(BenchResult &r)
    {
        // -----------------------------
        // 1. Load corridors
        // -----------------------------
        Vec3d start, goal;
        vec_Vecf<3> path;
        std::vector<double> seg_end_times;
        vec_E<Polyhedron<3>> poly_out;
        std::vector<LinearConstraint3D> l_constraints;

        loadMysco2(
            fs::path(r.file),
            start,
            goal,
            path,
            seg_end_times,
            poly_out,
            l_constraints,
            poly_seed_eps_,
            debug_poly_check_);

        const std::string fname = fs::path(r.file).filename().string();

        // Cache corridor polyhedra for RViz
        {
            auto msg = DecompROS::polyhedron_array_to_ros(poly_out);
            msg.header.frame_id = frame_id_;
            msg.header.stamp = now();
            msg.lifetime = rclcpp::Duration::from_seconds(1.0);
            r.poly_msg = msg;
        }

        // Global path marker array
        {
            r.global_path_ma.markers.clear();
            vectorOfVectors2MarkerArray(path, &r.global_path_ma, color(RED));
        }

        // Sanity
        if (path.size() < 2 || l_constraints.size() != path.size() - 1)
        {
            RCLCPP_ERROR(get_logger(), "Invalid corridor in %s", r.file.c_str());
            return false;
        }

        // -----------------------------
        // 2. Initial and goal states
        // -----------------------------
        state local_A, local_E;

        local_A.setZero();
        local_A.pos = start;

        local_E.setZero();
        local_E.pos = goal;

        // -----------------------------
        // 3. Create solver
        // -----------------------------
        auto solver = std::make_shared<lbfgs::SolverLBFGS>();
        solver->initializeSolver(planner_params_);
        solver->setUseOccCost(use_occ_cost_);
        // RCLCPP_INFO(get_logger(), "Use occlusion cost %d: ", use_occ_cost_);
        solver->setMapUtil(mighty_ptr_->getMapUtilShared().get());
        auto map_util = mighty_ptr_->getMapUtilShared();

        if (!map_util) {
        RCLCPP_ERROR(get_logger(),
        "MapUtil is null! Map callback has not run yet.");
        }
        // RCLCPP_INFO(get_logger(), "reached 1433");

        std::vector<std::shared_ptr<dynTraj>> no_obstacles;

        double initial_guess_time_ms = 0.0;

        // -----------------------------
        // 4. Prepare solver
        // -----------------------------
        solver->prepareSolverForReplan(
            /*t0=*/0.0,
            path,
            l_constraints,
            no_obstacles,
            local_A,
            local_E,
            initial_guess_time_ms,
            /*use_for_safe_path=*/false,
            /*use_multiple_initial_guesses=*/false);

        // -----------------------------
        // 5. Optimize
        // -----------------------------
        auto list_z0 = solver->getInitialGuesses();
        if (list_z0.empty())
        {
            RCLCPP_ERROR(get_logger(), "No initial guess for %s", r.file.c_str());
            return false;
        }

        Eigen::VectorXd zopt_;
        double fopt_ = 0.0;

        lbfgs::lbfgs_parameter_t lbfgs_params;
        lbfgs_params.mem_size = 256;

        auto t0 = std::chrono::high_resolution_clock::now();
        // RCLCPP_INFO(get_logger(), "reached 1470");

        int status = solver->optimize(list_z0[0], zopt_, fopt_, lbfgs_params);
        // RCLCPP_INFO(get_logger(), "reached 1473");
        auto t1 = std::chrono::high_resolution_clock::now();

        double solve_time_ms =
            std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (status < 0 || fopt_ > par_.fopt_threshold)
        {
            if (status < 0){
            RCLCPP_ERROR(get_logger(),
                        "Optimization failed due to status < 0 (%s): status=%d fopt=%.3f",
                        r.file.c_str(), status, fopt_);                
            }

            if (fopt_ > par_.fopt_threshold){
            RCLCPP_ERROR(get_logger(),
                        "Optimization failed due to high fopt(%s): status=%d fopt=%.3f, fopt_threshold=%.3f",
                        r.file.c_str(), status, fopt_, par_.fopt_threshold);
            }
            r.success = false;
            r.status = "opt_failed";
            RCLCPP_ERROR(get_logger(),
                        "Optimization failed (%s): status=%d fopt=%.3f",
                        r.file.c_str(), status, fopt_);
            return false;
        }
        // RCLCPP_INFO(get_logger(),
        //                 "Optimization success (%s): status=%d fopt=%.3f",
        //                 r.file.c_str(), status, fopt_);
  

        // -----------------------------
        // 6. Extract trajectory
        // -----------------------------
        solver->reconstructPVATCPopt(zopt_);
        std::vector<state> goal_setpoints;
        solver->getGoalSetpoints(goal_setpoints);

        const auto crep = analyzeConstraintsSampled(
                            goal_setpoints, l_constraints, par_.dc,
                            par_.v_max, par_.a_max, par_.j_max);
        applyConstraintReport(r, crep);

        r.opt_traj_ma = stateVector2ColoredMarkerArray(goal_setpoints, /*type=*/1, par_.v_max, this->now());
        r.success = true;
        r.status = "success";

        maybeDumpTrajectory(fname, goal_setpoints, r);
        r.start = start;
        r.goal  = goal;

        return true;
    }

    void writeCsv() const
    {
        std::ofstream ofs(csv_out_);
        if (!ofs)
        {
            RCLCPP_WARN(get_logger(), "Failed to open csv_out: %s", csv_out_.c_str());
            return;
        }

        ofs << "planner_name,file,success,status,gurobi_error,per_opt_runtime_ms,total_opt_runtime_ms,factor_used,cost_value,total_traj_time_sec,"
               "corridor_max_min_violation,corridor_t_at_max,corridor_px,corridor_py,corridor_pz,corridor_best_poly_idx,corridor_violated,"
               "v_max_observed,v_max_excess,v_t_at_max,v_violated,"
               "a_max_observed,a_max_excess,a_t_at_max,a_violated,"
               "j_max_observed,j_max_excess,j_t_at_max,j_violated,"
               "jerk_smoothness_l1,jerk_rms,"
               "traj_length_m,"
               "start_x,start_y,start_z,goal_x,goal_y,goal_z\n";

        for (const auto &r : results_)
        {
            ofs << planner_name_ << ","
                << fs::path(r.file).filename().string() << ","
                << (r.success ? 1 : 0) << ","
                << "\"" << r.status << "\"" << ","
                << (r.gurobi_error ? 1 : 0) << ","
                << std::fixed << std::setprecision(3)
                << r.per_opt_runtime_ms << ","
                << r.total_opt_runtime_ms << ","
                << r.factor_used << ","
                << r.cost_value << ","
                << r.total_traj_time_sec << ","
                << std::setprecision(9)
                << r.corridor_max_min_violation << ","
                << r.corridor_t_at_max << ","
                << r.corridor_p_at_max.x() << "," << r.corridor_p_at_max.y() << "," << r.corridor_p_at_max.z() << ","
                << r.corridor_best_poly_idx_at_max << ","
                << (r.corridor_violated ? 1 : 0) << ","
                << r.v_max_observed << ","
                << r.v_max_excess << ","
                << r.v_t_at_max << ","
                << (r.v_violated ? 1 : 0) << ","
                << r.a_max_observed << ","
                << r.a_max_excess << ","
                << r.a_t_at_max << ","
                << (r.a_violated ? 1 : 0) << ","
                << r.j_max_observed << ","
                << r.j_max_excess << ","
                << r.j_t_at_max << ","
                << (r.j_violated ? 1 : 0) << ","
                << r.jerk_smoothness_l1 << ","
                << r.jerk_rms << ","
                << r.traj_length_m << ","
                << std::setprecision(3)
                << r.start.x() << "," << r.start.y() << "," << r.start.z() << ","
                << r.goal.x() << "," << r.goal.y() << "," << r.goal.z()
                << "\n";
        }

        ofs.flush();
        RCLCPP_INFO(get_logger(), "Wrote CSV: %s", csv_out_.c_str());
    }

    void publishNext()
    {
        // RCLCPP_INFO(this->get_logger(), "publishing next for viz");
        auto &r = results_[play_idx_];
        auto stamp = now();

        // Clear the trajectory topic
        {
            visualization_msgs::msg::MarkerArray clear_msg;
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = stamp;
            m.action = visualization_msgs::msg::Marker::DELETEALL;
            clear_msg.markers.push_back(m);
            pub_traj_committed_colored_->publish(clear_msg);
            pub_dgp_path_marker_->publish(clear_msg);
        }

        // Publish global path markers
        for (auto &mk : r.global_path_ma.markers)
        {
            mk.header.frame_id = frame_id_;
            mk.header.stamp = stamp;
        }
        pub_dgp_path_marker_->publish(r.global_path_ma);

        // Publish committed traj markers (if success)
        if (r.success)
        {
            for (auto &mk : r.opt_traj_ma.markers)
            {
                mk.header.frame_id = frame_id_;
                mk.header.stamp = stamp;
            }
            pub_traj_committed_colored_->publish(r.opt_traj_ma);
        }

        // publish corridor polyhedra
        r.poly_msg.header.stamp = stamp;
        pub_poly_->publish(r.poly_msg);

        play_idx_ = (play_idx_ + 1) % results_.size();
    }

private:
    std::string planner_name_{"dynus"};

    // I/O
    std::string sfc_dir_;
    std::string file_ext_;
    std::string frame_id_;
    std::string poly_topic_;
    std::string traj_committed_topic_;
    std::string dgp_path_topic_;
    std::string csv_out_;

    bool use_single_threaded_{false};

    bool visualize_{true};
    double playback_period_sec_{1.0};
    bool latched_{true};

    double assumed_last_replan_time_sec_{0.05};

    parameters par_;
    // This is for mighty
    lbfgs::planner_params_t planner_params_;

    std::vector<BenchResult> results_;
    size_t play_idx_{0};

    std::vector<double> factors_;
    // std::vector<std::shared_ptr<SolverGurobi>> whole_traj_solver_ptrs_;
    std::vector<std::shared_ptr<lbfgs::SolverLBFGS>> whole_traj_solver_ptrs_;

    double poly_seed_eps_{1e-6};
    bool debug_poly_check_{true};

    std::vector<EllipsoidDecomp3D> ellip_workers_;

    // NEW: trajectory dump control
    bool traj_dump_enable_{true};
    std::string traj_dump_root_dir_;
    double traj_dump_dt_{-1.0};

    bool traj_dump_enable_this_run_{false};
    std::string traj_dump_run_dir_;

    // ROS
    rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_committed_colored_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dgp_path_marker_;
    rclcpp::TimerBase::SharedPtr playback_timer_;

    // For map
    std::shared_ptr<MIGHTY> mighty_ptr_;
    std::mutex map_mutex_;
    bool use_occ_cost_;
    bool solvers_initialized_{false};
    bool map_ready_{false};
    bool planning_started_{false};
    rclcpp::TimerBase::SharedPtr planning_timer_;
    // std::shared_ptr<mighty::VoxelMapUtil> map_util;
    rclcpp::CallbackGroup::SharedPtr cb_group_map_;
    // Time synchronizer
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> occup_grid_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> unknown_grid_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalTrajBenchmarkNode>());
    rclcpp::shutdown();
    return 0;
}
