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
using namespace mighty;

class OcclusionVizNode final : public rclcpp::Node
{
public:
    OcclusionVizNode() : Node("occlusion_viz_node")
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
        this->declare_parameter<double>("initial_wdx", 30.0);
        this->declare_parameter<double>("initial_wdy", 30.0);
        this->declare_parameter<double>("initial_wdz", 3.0);
        this->declare_parameter<double>("min_wdx", 30.0);
        this->declare_parameter<double>("min_wdy", 30.0);
        this->declare_parameter<double>("min_wdz", 3.0);
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
        declare_parameter<std::string>("traj_directory", "");
        get_parameter("traj_directory", traj_directory_);

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



        if (traj_directory_.empty()) {
            RCLCPP_FATAL(get_logger(), "traj_directory parameter not set.");
            rclcpp::shutdown();
            return;
        }

        loadTrajectories(traj_directory_);

        pub_occ_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "occlusion_markers", 10);

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
            std::bind(&OcclusionVizNode::mapCallback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        planning_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&OcclusionVizNode::process, this),
            cb_group_map_);
    }

private:

    // ===============================
    // CSV LOADING
    // ===============================
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
    // KEEP THIS MAP CALLBACK EXACTLY AS DESIGNED
    // ============================================
    void mapCallback(
        const sensor_msgs::msg::PointCloud2::ConstPtr &map_msg,
        const sensor_msgs::msg::PointCloud2::ConstPtr &unk_msg)
    {
        if (map_ready_)
            return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*map_msg, *map_pc);

        pcl::PointCloud<pcl::PointXYZ>::Ptr unk_pc(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*unk_msg, *unk_pc);

        if (!mighty_)
            mighty_ = std::make_shared<MIGHTY>(par_);
            // dgp_manager_.setParameters(par_);

        mighty_->updateMap(map_pc, unk_pc);
        
        map_ready_ = true;
        RCLCPP_INFO(get_logger(), "Map received and stored.");
    }

    // ===============================
    // PROCESS AFTER MAP IS READY
    // ===============================
    void process()
    {
        if (!map_ready_ || visualized_)
            return;

        auto map_util = mighty_->getMapUtilShared().get();
        map_util->info();
        if (!map_util) {
            RCLCPP_ERROR(get_logger(), "Map util not available.");
            return;
        }

        int num_unknown = map_util->countUnknownCells();
        RCLCPP_INFO(get_logger(), "%d is number unknown cells", num_unknown);
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        double radius = map_util->getRes();
        int min_num = 1;

        for (const auto& traj : trajectories_)
        {
            for (const auto& pt : traj)
            {
                Vecf<3> pt_f;
                pt_f << static_cast<float>(pt.x()),
                        static_cast<float>(pt.y()),
                        static_cast<float>(pt.z());
                auto occ = map_util->detectOcclusionAt(pt_f, radius, min_num);

                if (occ.is_occlusion)
                {
                    // RCLCPP_INFO(get_logger(), "Found an occlusion");
                    visualization_msgs::msg::Marker m;
                    m.header.frame_id = "map";
                    m.header.stamp = rclcpp::Time(0);//now();
                    m.ns = "occlusions";
                    m.id = id++;
                    m.type = visualization_msgs::msg::Marker::SPHERE;
                    m.action = visualization_msgs::msg::Marker::ADD;

                    m.pose.position.x = pt.x();
                    m.pose.position.y = pt.y();
                    m.pose.position.z = pt.z();

                    m.scale.x = 0.12;
                    m.scale.y = 0.12;
                    m.scale.z = 0.12;

                    m.color.r = 1.0;
                    m.color.g = 1.0;
                    m.color.b = 0.0;
                    m.color.a = 1.0;

                    marker_array.markers.push_back(m);
                }
                
            }
        }

        visualization_msgs::msg::Marker traj_marker;
        traj_marker.header.frame_id = "map";
        traj_marker.header.stamp = rclcpp::Time(0);
        traj_marker.ns = "trajectory_debug";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        traj_marker.action = visualization_msgs::msg::Marker::ADD;

        traj_marker.scale.x = 0.05;  // line width

        traj_marker.color.a = 1.0;  // not used for LINE_LIST with per-vertex colors

        for (const auto& traj : trajectories_)
        {
            for (size_t i = 0; i + 1 < traj.size(); ++i)
            {
                const auto& p0 = traj[i];
                const auto& p1 = traj[i + 1];

                // Midpoint classification
                Eigen::Vector3d mid = 0.5 * (p0 + p1);

                Vecf<3> mid_f;
                mid_f << static_cast<float>(mid.x()),
                        static_cast<float>(mid.y()),
                        static_cast<float>(mid.z());
                if (map_util->isOutside(map_util->floatToInt(mid_f)))
                {
                    // std::cout << "Outside map: " << mid.transpose() << std::endl;
                }

                std_msgs::msg::ColorRGBA color;
                color.a = 1.0;

                if (map_util->isUnknown(mid_f))
                {
                    // Blue
                    color.r = 0.0;
                    color.g = 0.0;
                    color.b = 1.0;
                }
                else if (map_util->isFree(mid_f))
                {
                    // Green
                    color.r = 0.0;
                    color.g = 1.0;
                    color.b = 0.0;
                }
                else if (map_util->isOccupied(mid_f))
                {
                    // Occupied (optional red)
                    color.r = 1.0;
                    color.g = 0.0;
                    color.b = 0.0;
                }
                else{
                    color.r = 1.0;
                    color.g = 0.0;
                    color.b = 1.0;
                }

                geometry_msgs::msg::Point pt0, pt1;
                pt0.x = p0.x();
                pt0.y = p0.y();
                pt0.z = p0.z();

                pt1.x = p1.x();
                pt1.y = p1.y();
                pt1.z = p1.z();

                traj_marker.points.push_back(pt0);
                traj_marker.points.push_back(pt1);

                traj_marker.colors.push_back(color);
                traj_marker.colors.push_back(color);
            }
        }

        marker_array.markers.push_back(traj_marker);

        RCLCPP_INFO(get_logger(),
            "Publishing %zu occlusion markers",
            marker_array.markers.size());

        pub_occ_->publish(marker_array);
        visualized_ = true;

        RCLCPP_INFO(get_logger(), "Occlusion markers published.");
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
    bool visualized_{false};

    std::shared_ptr<MIGHTY> mighty_;
    DGPManager dgp_manager_;


    parameters par_;
    

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OcclusionVizNode>());
    rclcpp::shutdown();
    return 0;
}
