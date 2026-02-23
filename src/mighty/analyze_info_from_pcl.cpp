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

#include <dynus_interfaces/msg/dyn_traj.hpp>
#include <dynus_interfaces/msg/dyn_traj_array.hpp>
#include "dynus_interfaces/msg/state.hpp"
#include "dynus_interfaces/msg/goal.hpp"
#include "dynus_interfaces/msg/yaw_output.hpp"
#include "dynus_interfaces/msg/pn_adaptation.hpp"

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <iomanip>
#include <cstddef> // Required for size_t
#include <cmath> // Required for atan()

namespace fs = std::filesystem;
using namespace mighty;

struct TrajPoint
{
    double t;

    double x, y, z;
    double vx, vy, vz;
    double ax, ay, az;
    double jx, jy, jz;
};

using GlobalVoxel = Eigen::Vector3i;
struct GlobalVoxelHash
{
  std::size_t operator()(const GlobalVoxel& v) const noexcept
  {
    std::size_t hx = std::hash<int>()(v.x());
    std::size_t hy = std::hash<int>()(v.y());
    std::size_t hz = std::hash<int>()(v.z());

    return hx ^ (hy << 1) ^ (hz << 2);
  }
};

struct GlobalVoxelEqual
{
  bool operator()(const GlobalVoxel& a,
                  const GlobalVoxel& b) const noexcept
  {
    return a.x() == b.x() &&
           a.y() == b.y() &&
           a.z() == b.z();
  }
};

class OcclusionAnalysisPCLNode final : public rclcpp::Node
{
public:
    OcclusionAnalysisPCLNode() : Node("occlusion_analysis_pcl_node")
    {
        
        // Declare parameters
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

        declare_parameter<std::string>("traj_directory", "");
        get_parameter("traj_directory", traj_directory_);

        this->declare_parameter<std::string>("trajectory_csv_path", "/home/kkondo/code/mighty_ws/src/mighty/benchmark_data/multi_thread/traj_dump/mighty_N5/traj_mighty_N5__sfc_g000.mysco2.csv");
        std::string traj_path;
        this->get_parameter("trajectory_csv_path", traj_path);
        loadTrajectory(traj_path);

        progress_file_.open("exploration_progress.csv");
        progress_file_ << std::fixed << std::setprecision(10);
        progress_file_ << "time_sec,percent_unknown\n";

        // ------------------------------------------------------------------------------------------


        if (traj_directory_.empty()) {
            RCLCPP_FATAL(get_logger(), "traj_directory parameter not set.");
            rclcpp::shutdown();
            return;
        }

        // loadTrajectories(traj_directory_);

        pub_occ_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "occlusion_markers", 10);

        // Qos policy settings
        rclcpp::QoS critical_qos(rclcpp::KeepLast(10));
        critical_qos.reliable().durability_volatile();
        pub_goal_ = this->create_publisher<dynus_interfaces::msg::Goal>("/NX01/goal", critical_qos);

        // ----------------------------
        // KEEP SYNCHRONIZED MAP SETUP
        // ----------------------------
        cb_group_map_ =
            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options;
        options.callback_group = cb_group_map_;

        occup_grid_sub_.subscribe(this, "/NX01/occupancy_grid",
                                  rmw_qos_profile_sensor_data, options);
        unknown_grid_sub_.subscribe(this, "/NX01/unknown_grid",
                                    rmw_qos_profile_sensor_data, options);

        sync_.reset(new Sync(MySyncPolicy(10),
                             occup_grid_sub_,
                             unknown_grid_sub_));

        sync_->registerCallback(
            std::bind(&OcclusionAnalysisPCLNode::mapCallback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));


    }

private:

    // ===============================
    // CSV LOADING
    // ===============================
    void loadTrajectory(const std::string& path)
    {
        std::ifstream file(path);
        
        RCLCPP_INFO(get_logger(), "Working dir: %s",
                    std::filesystem::current_path().c_str());

        RCLCPP_INFO(get_logger(), "Trying to open: %s",
                    path.c_str());

        std::ifstream test(path);
        RCLCPP_INFO(get_logger(), "File open success: %d",
                    test.is_open());

        if (!file.is_open())
        {
            RCLCPP_ERROR(get_logger(), "Failed to open file: %s", path.c_str());
            return;
        }

        std::string line;

        while (std::getline(file, line))
        {
            // Skip empty lines
            if (line.empty())
                continue;

            // Skip comments
            if (line[0] == '#')
                continue;

            // Skip header
            if (line.rfind("t,", 0) == 0)
                continue;

            std::stringstream ss(line);
            std::string value;
            TrajPoint pt;

            std::getline(ss, value, ','); pt.t  = std::stod(value);

            std::getline(ss, value, ','); pt.x  = std::stod(value);
            std::getline(ss, value, ','); pt.y  = std::stod(value);
            std::getline(ss, value, ','); pt.z  = std::stod(value);

            std::getline(ss, value, ','); pt.vx = std::stod(value);
            std::getline(ss, value, ','); pt.vy = std::stod(value);
            std::getline(ss, value, ','); pt.vz = std::stod(value);

            std::getline(ss, value, ','); pt.ax = std::stod(value);
            std::getline(ss, value, ','); pt.ay = std::stod(value);
            std::getline(ss, value, ','); pt.az = std::stod(value);

            std::getline(ss, value, ','); pt.jx = std::stod(value);
            std::getline(ss, value, ','); pt.jy = std::stod(value);
            std::getline(ss, value, ','); pt.jz = std::stod(value);

            trajectory_.push_back(pt);
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu trajectory points",
                    trajectory_.size());
    }

    void loadTrajectories(const std::string& dir)
    {
        RCLCPP_INFO(get_logger(), "Loading trajectories from: %s", dir.c_str());

        for (const auto& entry : fs::directory_iterator(dir))
        {
            if (entry.path().extension() != ".csv")
                continue;

            std::ifstream file(entry.path());
            if (!file.is_open())
                continue;

            std::vector<Eigen::Vector3d> traj;
            std::string line;

            while (std::getline(file, line))
            {
                if (line.empty() || line[0] == '#')
                    continue;

                if (line.rfind("t,", 0) == 0)
                    continue;

                std::stringstream ss(line);
                std::string token;
                std::vector<double> values;

                while (std::getline(ss, token, ','))
                    values.push_back(std::stod(token));

                if (values.size() >= 4)
                {
                    traj.emplace_back(values[1], values[2], values[3]);
                }
            }

            if (!traj.empty())
                trajectories_.push_back(traj);
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu trajectories.",
                    trajectories_.size());
    }

    // ============================================
    // MAP CALLBACK
    // ============================================
    void mapCallback(
        const sensor_msgs::msg::PointCloud2::ConstPtr &map_msg,
        const sensor_msgs::msg::PointCloud2::ConstPtr &unk_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr unk_pc(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*unk_msg, *unk_pc);

        if (!initial_unknown_saved_) {
            for (const auto& pt : unk_pc->points) {
                GlobalVoxel gv = worldToGlobalVoxel(Vec3f(pt.x, pt.y, pt.z));
                original_unknown_.insert(gv);
                remaining_unknown_.insert(gv);
            }

            original_unknown_num_ = original_unknown_.size();
            initial_unknown_saved_ = true;

            RCLCPP_INFO(get_logger(),
                "Saved %zu original unknown voxels",
                original_unknown_num_);

            return;
        }


        if (trajectory_.empty()){
            RCLCPP_INFO(get_logger(), "traj empty");
            return;
        }
        
        if (current_index_ >= trajectory_.size()){
            // RCLCPP_INFO(get_logger(), "Trajectory Complete. Remaining unknown: %zu", remaining_original_unknown_.size());
            return;
        }

        const auto& pt = trajectory_[current_index_];

        dynus_interfaces::msg::Goal quadGoal;

        quadGoal.header.stamp = this->now();
        quadGoal.header.frame_id = "map";

        quadGoal.p.x = pt.x;
        quadGoal.p.y = pt.y;
        quadGoal.p.z = pt.z;

        quadGoal.v.x = pt.vx;
        quadGoal.v.y = pt.vy;
        quadGoal.v.z = pt.vz;

        quadGoal.a.x = pt.ax;
        quadGoal.a.y = pt.ay;
        quadGoal.a.z = pt.az;

        quadGoal.j.x = pt.jx;
        quadGoal.j.y = pt.jy;
        quadGoal.j.z = pt.jz;
        if (std::sqrt(pt.vx*pt.vx + pt.vy*pt.vy) > 0.001){
            quadGoal.yaw  = std::atan2(pt.vy, pt.vx); //0.0;   // Not in CSV
        }
        else{
            quadGoal.yaw = old_yaw_;
        }

        old_yaw_ = quadGoal.yaw;

        quadGoal.dyaw = 0.0;

        pub_goal_->publish(quadGoal);

        // Build current unknown set
        std::unordered_set<GlobalVoxel, GlobalVoxelHash> current_unknown;

        for (const auto& pt : unk_pc->points) {
            GlobalVoxel gv = worldToGlobalVoxel(Vec3f(pt.x, pt.y, pt.z));
            current_unknown.insert(gv);
        }

        // Check which original unknown voxels disappeared
        for (auto it = remaining_unknown_.begin();
            it != remaining_unknown_.end(); )
        {
            if (current_unknown.find(*it) == current_unknown.end()) {
                // This voxel was originally unknown and is no longer unknown -> converted
                converted_.insert(*it);
                it = remaining_unknown_.erase(it);
            } else {
                ++it;
            }
        }

        double percent_converted =
            double(converted_.size()) / original_unknown_num_;

        RCLCPP_INFO(get_logger(),
            "Converted %zu / %zu (%.3f%%)",
            converted_.size(),
            original_unknown_num_,
            100.0 * percent_converted);

        size_t time_index = std::min(current_index_ - 1, trajectory_.size() - 1);

        double traj_time = trajectory_[time_index].t;
        
        progress_file_ << traj_time << "," << 100.0 * percent_converted << "\n";

        current_index_++;

    }


    // HELPER FUNCTION
    inline GlobalVoxel worldToGlobalVoxel(const Vec3f& pt) const
    {
    return GlobalVoxel(
        static_cast<int>(std::floor(pt.x() / res_)),
        static_cast<int>(std::floor(pt.y() / res_)),
        static_cast<int>(std::floor(pt.z() / res_))
    );
    }


    // ===============================
    // MEMBERS
    // ===============================

    std::string traj_directory_;
    std::vector<std::vector<Eigen::Vector3d>> trajectories_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_occ_;
    rclcpp::CallbackGroup::SharedPtr cb_group_map_;
    rclcpp::TimerBase::SharedPtr planning_timer_;

    // Map sync
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> occup_grid_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> unknown_grid_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2> MySyncPolicy;

    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    bool map_ready_{false};

    std::shared_ptr<MIGHTY> mighty_;
    DGPManager dgp_manager_;


    parameters par_;

    std::vector<TrajPoint> trajectory_;
    size_t current_index_ = 0;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Publisher<dynus_interfaces::msg::Goal>::SharedPtr pub_goal_;
    std::ofstream progress_file_;

    std::unordered_set<
    GlobalVoxel,
    GlobalVoxelHash,
    GlobalVoxelEqual
    > remaining_original_unknown_;

    size_t conversion_count_ = 0;
    rclcpp::TimerBase::SharedPtr compare_timer_;
    bool initial_unknown_saved_map_{false};
    bool initial_unknown_saved_{false};
    double res_ = 0.15; // TODO: MAKE SURE THIS MATCHES WITH THE DEFAULT RESOLUTION VALUE
    size_t original_unknown_num_;
    float old_yaw_ = 0.0;
    std::mutex map_mutex_;
    std::unordered_set<GlobalVoxel, GlobalVoxelHash> original_unknown_;
    std::unordered_set<GlobalVoxel, GlobalVoxelHash> remaining_unknown_;
    std::unordered_set<GlobalVoxel, GlobalVoxelHash> converted_;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OcclusionAnalysisPCLNode>());
    rclcpp::shutdown();
    return 0;
}
