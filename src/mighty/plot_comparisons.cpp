#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>

namespace fs = std::filesystem;

class TrajectoryDirectoryVisualizer : public rclcpp::Node
{
public:
    TrajectoryDirectoryVisualizer(
        const std::string & nominal_dir,
        const std::string & modified_dir)
    : Node("trajectory_directory_visualizer"),
      nominal_dir_(nominal_dir),
      modified_dir_(modified_dir)
    {
        nominal_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "nominal_paths", 10);

        modified_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "modified_paths", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TrajectoryDirectoryVisualizer::publish_all, this));

        RCLCPP_INFO(this->get_logger(), "Trajectory visualizer started.");
    }

private:

    std::vector<geometry_msgs::msg::Point>
    load_trajectory(const std::string & filepath)
    {
        std::vector<geometry_msgs::msg::Point> points;
        std::ifstream file(filepath);

        if (!file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Failed to open: %s", filepath.c_str());
            return points;
        }

        std::string line;
        bool header_skipped = false;

        while (std::getline(file, line)) {

            if (line.empty())
                continue;

            // Skip metadata comments
            if (line[0] == '#')
                continue;

            // Skip column header line (t,x,y,z,...)
            if (!header_skipped) {
                header_skipped = true;
                continue;
            }

            std::stringstream ss(line);
            std::string token;

            std::vector<double> values;

            while (std::getline(ss, token, ',')) {
                try {
                    values.push_back(std::stod(token));
                }
                catch (const std::exception &) {
                    // Skip malformed line safely
                    values.clear();
                    break;
                }
            }

            // We expect at least t,x,y,z
            if (values.size() >= 4) {
                geometry_msgs::msg::Point p;
                p.x = values[1];
                p.y = values[2];
                p.z = values[3];
                points.push_back(p);
            }
        }

    return points;
    }


    visualization_msgs::msg::MarkerArray
    create_marker_array_from_directory(
        const std::string & directory,
        float r, float g, float b,
        const std::string & ns,
        double z_offset)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        int id = 0;

        for (const auto & entry : fs::directory_iterator(directory)) {

            if (entry.path().extension() != ".csv")
                continue;

            auto points = load_trajectory(entry.path().string());

            if (points.empty())
                continue;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = ns;
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.scale.x = 0.02;   // Thin line

            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 0.9;

            for (auto & p : points) {
                p.z += z_offset;
                marker.points.push_back(p);
            }

            marker_array.markers.push_back(marker);
        }

        return marker_array;
    }

    void publish_all()
    {
        if (!fs::exists(nominal_dir_) || !fs::exists(modified_dir_)) {
            RCLCPP_ERROR(this->get_logger(), "Directory does not exist.");
            return;
        }

        auto nominal_markers = create_marker_array_from_directory(
            nominal_dir_,
            0.0f, 0.0f, 1.0f,   // Blue
            "nominal",
            0.0);

        auto modified_markers = create_marker_array_from_directory(
            modified_dir_,
            1.0f, 0.0f, 0.0f,   // Red
            "modified",
            0.01);              // Small lift

        nominal_pub_->publish(nominal_markers);
        modified_pub_->publish(modified_markers);
    }

    std::string nominal_dir_;
    std::string modified_dir_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr nominal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr modified_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
    if (argc < 3) {
        std::cout << "Usage: ros2 run your_package trajectory_directory_visualizer "
                  << "<nominal_dir> <modified_dir>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrajectoryDirectoryVisualizer>(
        argv[1],
        argv[2]);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
