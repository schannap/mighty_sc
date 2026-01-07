
#include <iostream>
#include "dgp/data_type.hpp"
#include <omp.h>
#include "dgp/map_util.hpp"
#include <pcl/kdtree/kdtree.h>
#include <Eigen/StdVector>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

// It should also publish visualization for the "direction of occlusion" arrows
using OcclusionInfo = mighty::MapUtil<3>::OcclusionInfo;
using traj_occlusion_info = mighty::MapUtil<3>::traj_occlusion_info;

class TestableMapUtil : public mighty::MapUtil<3>
{
public:
  using mighty::MapUtil<3>::MapUtil;
  
  void initTestMap(const Veci<3>& dim,
                   float res,
                   const Vecf<3>& origin)
  {
    dim_ = dim;
    res_ = res;
    origin_d_ = origin;
    total_size_ = dim[0] * dim[1] * dim[2];
    map_.assign(total_size_, val_unknown_);
  }

  void setFree(const Veci<3>& p)
  {
    map_[getIndex(p)] = val_free_;
  }

  void setOccupied(const Veci<3>& p)
  {
    map_[getIndex(p)] = val_occ_;
  }
  float getRes() const { return res_; }
  Vecf<3> getOrigin() const { return origin_d_; }
  Veci<3> getDim() const { return dim_; }
  const std::vector<int>& getMapData() const { return map_; }
  int8_t getValFree() const { return val_free_;}
  int8_t getValUnknown() const { return val_unknown_;}
  int8_t getValOcc() const { return val_occ_;}
};




//TODO: add more tests for detecting occlusions if there are intersections with trajectories

//TODO: Check comment in occlusion detection function in map_util.hpp. Should make the likelihood of being an 
// occlusion decay with the distance from the query unknown space to the free space.



class OcclusionVizNode : public rclcpp::Node
{
public:
  OcclusionVizNode() : Node("occlusion_viz_node")
  {
    // Create markers to display the map (color dependent on free, unknown or occupied)
    map_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("map_marker", 10);
    // Create markers to display the initial trajectory
    traj_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("traj_marker", 10);
    // Create markers to display the occlusion norm direction
    occ_norm_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("occ_norm_marker", 10);

    // Create marker to display the velocity vector of a trajectory at an occlusion site
    occ_vel_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("occ_vel_marker", 10);
    run_tests();
    

  }
private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occ_norm_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occ_vel_marker_pub_;

  std::vector<state> traj_;
  // std::vector<traj_occlusion_info> traj_occlusions_;

  // create a function that takes as input a TestableMapUtil and visualizes it
  void publishMap(const TestableMapUtil& map)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "map";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = map.getRes();
    marker.scale.y = map.getRes();
    marker.scale.z = map.getRes();

    marker.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point p;

    for (int x = 0; x < map.getDim()[0]; ++x)
    {
      for (int y = 0; y < map.getDim()[1]; ++y)
      {
        for (int z = 0; z < map.getDim()[2]; ++z)
        {
          Veci<3> idx(x,y,z);
          int v = map.map_[map.getIndex(idx)];

          const float res = static_cast<float>(map.getRes());
          Vecf<3> ori = map.getOrigin(); 
          Vecf<3> res_vec((idx[0]+0.5)*res, (idx[1]+0.5)*res, (idx[2]+0.5)*res);
          Vecf<3> pos = ori + res_vec;
          p.x = pos.x();
          p.y = pos.y();
          p.z = pos.z();
          marker.points.push_back(p);

          std_msgs::msg::ColorRGBA c;
          c.a = 0.8f;

          
          if (v == map.getValFree())        { c.r = 0.0; c.g = 1.0; c.b = 0.0; }
          else if (v == map.getValOcc())    { c.r = 1.0; c.g = 0.0; c.b = 0.0; }
          else                           { c.r = 1.0; c.g = 1.0; c.b = 0.0; }

          marker.colors.push_back(c);
        }
      }
    }

    map_marker_pub_->publish(marker);
  }
 // create a function that takes as input a trajectory and visualizes it
  void publishTrajectory(
    const std::vector<state>& traj
  )
  {
    // -------- Trajectory --------
    visualization_msgs::msg::Marker traj_marker;
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = now();
    traj_marker.ns = "trajectory";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::msg::Marker::ADD;

    traj_marker.scale.x = 0.05;
    traj_marker.color.r = 0.0;
    traj_marker.color.g = 0.0;
    traj_marker.color.b = 1.0;
    traj_marker.color.a = 1.0;

    traj_marker.pose.orientation.w = 1.0;

    for (const auto& s : traj)
    {
      geometry_msgs::msg::Point p;
      p.x = s.pos.x();
      p.y = s.pos.y();
      p.z = s.pos.z();
      traj_marker.points.push_back(p);
    }

    traj_marker_pub_->publish(traj_marker);
  }

  void publishOcclusionNormals(
    const std::vector<traj_occlusion_info>& occlusions)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    const double arrow_length = 0.6;  // meters

    for (const auto& occ : occlusions)
    {
      // Skip invalid normals
      if (occ.normal.norm() < 1e-6)
        continue;

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now();
      m.ns = "occlusion_normals";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;

      // ---- Arrow geometry ----
      geometry_msgs::msg::Point p0, p1;

      p0.x = occ.position.x();
      p0.y = occ.position.y();
      p0.z = occ.position.z();

      Vecf<3> n_hat = occ.normal.normalized();

      p1.x = p0.x + arrow_length * n_hat.x();
      p1.y = p0.y + arrow_length * n_hat.y();
      p1.z = p0.z + arrow_length * n_hat.z();

      m.points.push_back(p0);
      m.points.push_back(p1);

      // ---- Arrow scale ----
      m.scale.x = 0.05;  // shaft diameter
      m.scale.y = 0.10;  // head diameter
      m.scale.z = 0.15;  // head length

      // ---- Color (purple) ----
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.color.a = 1.0;

      marker_array.markers.push_back(m);
    }

    occ_norm_marker_pub_->publish(marker_array);
  }

  void publishOcclusionVels(
    const std::vector<traj_occlusion_info>& occlusions)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    const double arrow_length = 0.6;  // meters

    for (const auto& occ : occlusions)
    {

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now();
      m.ns = "occlusion_velocities";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;

      // ---- Arrow geometry ----
      geometry_msgs::msg::Point p0, p1;

      p0.x = occ.position.x();
      p0.y = occ.position.y();
      p0.z = occ.position.z();

      Vecf<3> v_hat = occ.velocity.normalized();

      p1.x = p0.x + arrow_length * v_hat.x();
      p1.y = p0.y + arrow_length * v_hat.y();
      p1.z = p0.z + arrow_length * v_hat.z();

      m.points.push_back(p0);
      m.points.push_back(p1);

      // ---- Arrow scale ----
      m.scale.x = 0.05;  // shaft diameter
      m.scale.y = 0.10;  // head diameter
      m.scale.z = 0.15;  // head length

      // ---- Color ----
      m.color.r = 0.0;
      m.color.g = 1.0;
      m.color.b = 1.0;
      m.color.a = 1.0;

      marker_array.markers.push_back(m);

    }

    occ_vel_marker_pub_->publish(marker_array);
  }
  // Find the nearest free voxel center to a query
  Vecf<3> nearestFreeVoxelCenter(
    const TestableMapUtil& map,
    const Vecf<3>& query_pos)
  {
    float best_dist2 = std::numeric_limits<float>::infinity();
    Vecf<3> best_p = Vecf<3>::Zero();

    const float res = map.getRes();
    const Vecf<3>& origin = map.getOrigin();
    const Veci<3>& dim = map.getDim();

    for (int x = 0; x < dim[0]; ++x)
      for (int y = 0; y < dim[1]; ++y)
        for (int z = 0; z < dim[2]; ++z)
        {
          Veci<3> idx(x, y, z);
          if (map.getMapData()[map.getIndex(idx)] != map.getValFree())
            continue;

          Vecf<3> p =
            origin + Vecf<3>(
              (x + 0.5f) * res,
              (y + 0.5f) * res,
              (z + 0.5f) * res);

          float d2 = (p - query_pos).squaredNorm();
          if (d2 < best_dist2)
          {
            best_dist2 = d2;
            best_p = p;
          }
        }

    return best_p;
  }
  // Check if a occluded position is within the influence band of a free position
  // Within the influence band is a necessary condition for being considered an occlusion but
  // not a sufficient condition
  bool withinVoxelInfluenceBand(
    const Vecf<3>& occ_pos,
    const Vecf<3>& free_pos,
    float resolution,
    float neighbor_radius)
  {
    const float h = 0.5f * resolution;
    const float shrink = 2.0f * h; // occ + free half extents

    Vecf<3> delta = (occ_pos - free_pos).cwiseAbs();

    Vecf<3> surface_dist;
    for (int i = 0; i < 3; ++i)
      // gets max of delta[i] - shrink and 0.0
      surface_dist[i] = (delta[i] - shrink > 0.0f)
                  ? (delta[i] - shrink)
                  : 0.0f;

    return surface_dist.norm() <= neighbor_radius;
  }

  // make each test a function within here and call the appropriate publishers? Maybe better this way so that each tests is completely isolated
  void testFlatBoundaryOcclusion()
  {
    TestableMapUtil map(
      /*res=*/0.5,
      /*x_min=*/0, /*x_max=*/5,
      /*y_min=*/0, /*y_max=*/5,
      /*z_min=*/0, /*z_max=*/2,
      /*inflation=*/0.0
    );

    map.initTestMap(
      Veci<3>(10, 10, 3),
      0.5,
      Vecf<3>(0, 0, 0)
    );

    // Mark free region: x < 5
    for (int x = 0; x < 5; ++x)
      for (int y = 0; y < 10; ++y)
        for (int z = 0; z < 3; ++z)
          map.setFree(Veci<3>(x,y,z));

    Veci<3> p_unknown(5, 5, 1);

    auto occ = map.detectOcclusionAt(p_unknown, 1.0, 6);

    assert(occ.is_occlusion);
    assert(occ.normal.norm() > 0.9);

    std::cout << "Normal: " << occ.normal.transpose() << std::endl;
  }

  void testAllFreeMap()
  {
    TestableMapUtil map(
      0.5,   // res
      -5, 5, // x lims
      -5, 5, // y lims
      -5, 5, // z lims
      0.0   // inflation
    );

    map.initTestMap(
      Veci<3>(20, 20, 20),
      0.5,
      Vecf<3>(-5, -5, -5)
    );

    // Mark everything free
    for (int x = 0; x < 20; ++x)
      for (int y = 0; y < 20; ++y)
        for (int z = 0; z < 20; ++z)
          map.setFree(Veci<3>(x,y,z));

    publishMap(map);
    // Interior point
    auto occ1 = map.detectOcclusionAt(Veci<3>(10,10,10), 1.0, 6);
    assert(!occ1.is_occlusion);

    // Boundary point
    auto occ2 = map.detectOcclusionAt(Veci<3>(0,10,10), 1.0, 6);
    assert(!occ2.is_occlusion);

    std::cout << "[PASS] All-free map test\n";
  }

  void testPartialOcclusion()
  {
    TestableMapUtil map(
      0.5, // res
      -5, 5, // x lims
      -5, 5, // y lims
      -5, 5, // z lims
      0.0   // inflation
    );

    map.initTestMap(
      Veci<3>(20, 20, 20),
      0.5,
      Vecf<3>(-5, -5, -5)
    );

    // Let all x >= 0 be unknown
    // Free half-space x < 0
    for (int x = 0; x < 10; ++x)
      for (int y = 0; y < 20; ++y)
        for (int z = 0; z < 20; ++z)
          map.setFree(Veci<3>(x,y,z));

    publishMap(map);
    // Case 1: near boundary -> occluded (enough free neighbors)
    Veci<3> p1(10,10,10);
    auto occ1 = map.detectOcclusionAt(p1, 1.0, 6);
    assert(occ1.is_occlusion);
    // Case 2: deep unknown -> not occluded (not enough free neighbors)
    Veci<3> p2(15,10,10);
    auto occ2 = map.detectOcclusionAt(p2, 1.0, 6);
    assert(!occ2.is_occlusion);

    std::cout << "[PASS] Partial occlusion logic test\n";
  }

  void testOcclusionNormalPlanar()
  {
    TestableMapUtil map(
      0.1, // res
      -1, 1, // x lims
      -1, 1, // y lims
      -1, 1, // z lims
      0.0   // inflation
    );

    map.initTestMap(
      Veci<3>(20, 20, 20),
      0.1,
      Vecf<3>(-1, -1, -1)
    );

    // Let everything x < 0 be free and x >= 0 be unknown
    // Free x < 0
    for (int x = 0; x < 10; ++x)
      for (int y = 0; y < 20; ++y)
        for (int z = 0; z < 20; ++z)
          map.setFree(Veci<3>(x,y,z));

    publishMap(map);
    Veci<3> p(11,10,10);  // x ≈ 0.1
    auto occ = map.detectOcclusionAt(p, 0.3, 6);

    assert(occ.is_occlusion);

    Vecf<3> expected(-1, 0, 0);
    float cos_angle = occ.normal.normalized().dot(expected);
    assert(cos_angle > 0.9);

    std::cout << "[PASS] Planar occlusion normal test\n";
  }



  void testOcclusionNormalCorner()
  {
    TestableMapUtil map(
      0.1, // res
      -1, 1, // x lims
      -1, 1, // y lims
      -1, 1, // z lims
      0.0   // inflation
    );

    map.initTestMap(
      Veci<3>(20, 20, 20),
      0.1,
      Vecf<3>(-1, -1, -1)
    );

    // Let everything x < 0 AND y < 0 be free and rest unknown
    // Free x < 0 and y < 0
    for (int x = 0; x < 10; ++x)
      for (int y = 0; y < 10; ++y)
        for (int z = 0; z < 20; ++z)
          map.setFree(Veci<3>(x,y,z));

    publishMap(map);
    Veci<3> p(10,10,10);  // near (0,0,0)
    auto occ = map.detectOcclusionAt(p, 0.3, 6);

    assert(occ.is_occlusion);

    Vecf<3> expected(-1, -1, 0);
    float cos_angle = occ.normal.normalized().dot(expected);
    assert(cos_angle > 0.9);

    std::cout << "[PASS] Planar occlusion normal test\n";
  }


  void testExactKFreeNeighborsOcclusion()
  {
    constexpr int k = 4;

    TestableMapUtil map(
      0.5, // res
      -5, 5, // x lims
      -5, 5, // y lims
      -5, 5, // z lims
      0.0   // inflation
    );

    map.initTestMap(
      Veci<3>(10, 10, 3),
      0.5,
      Vecf<3>(-5, -5, -5)
    );

    publishMap(map);
    // Query voxel (unknown by default)
    Veci<3> q(5, 5, 1);

    // Exactly k free neighbors (6-connectivity)
    std::vector<Veci<3>> free_neighbors = {
      Veci<3>(4,5,1),
      Veci<3>(6,5,1),
      Veci<3>(5,4,1),
      Veci<3>(5,6,1)
    };

    for (const auto& p : free_neighbors)
      map.setFree(p);

    auto occ = map.detectOcclusionAt(q, 1.0, k);

    assert(occ.is_occlusion);

    // Direction should be near zero due to symmetry
    assert(occ.normal.norm() < 1e-3);

    std::cout << "[PASS] Exact-k free neighbors occlusion test\n";
  }

  std::vector<state> makeTrajectoryFromPositions(
    const std::vector<Vecf<3>>& positions,
    const Vecf<3>& vel = Vecf<3>::Zero()
  )
  {
    std::vector<state> traj;
    traj.reserve(positions.size());

    for (const auto& p : positions)
    {
      state s;
      s.pos = p;
      s.vel = vel;
      traj.push_back(s);
    }
    return traj;
  }

  void testTrajectoryNoOcclusion(){
    // Create a 3d map for testing with dimensions 10 x 10 x 10 and resolution 0.5. All dims go from -5 to 5
    TestableMapUtil map(
      0.5, // res
      -5, 5, // x lims
      -5, 5, // y lims
      -5, 5, // z lims
      0.0   // inflation
    );

    map.initTestMap(
      Veci<3>(20, 20, 20),
      0.5,
      Vecf<3>(-5, -5, -5)
    );

    // Make the map entirely free
    // Mark everything free
    for (int x = 0; x < 20; ++x)
      for (int y = 0; y < 20; ++y)
        for (int z = 0; z < 20; ++z)
          map.setFree(Veci<3>(x,y,z));

    publishMap(map);
    // Create a sampled trajectory from x = -5 to x = 5
    std::vector<Vecf<3>> positions;
    for (float x = -5; x <= 5; x += 0.1f)
      positions.emplace_back(x, 0, 0);

    auto traj = makeTrajectoryFromPositions(
      positions, Vecf<3>(1,0,0));

    publishTrajectory(traj);

    auto occlusions = map.trajectoryIntersectsOcclusion(
      traj, // trajectory
      1.0,  // neighbor radius
      6     // min free neighbors
    );

    publishOcclusionNormals(occlusions);
    // For this example, assert that the list of occlusions the trajectory intersects is empty
    assert(occlusions.empty());
    std::cout << "[PASS] Trajectory no occlusion test\n";
  }

  void testTrajectoryWithPlanarOcclusion()
  {
    TestableMapUtil map(
      0.5,        // resolution
      -5, 5,
      -5, 5,
      -5, 5,
      0.0
    );

    map.initTestMap(
      Veci<3>(20, 20, 20),
      0.5,
      Vecf<3>(-5, -5, -5)
    );

    // Mark free space for x < 0
    for (int x = 0; x < 10; ++x)        // x < 0
      for (int y = 0; y < 20; ++y)
        for (int z = 0; z < 20; ++z)
          map.setFree(Veci<3>(x, y, z));

    // x >= 0 remains unknown -> occlusion boundary at x = 0
    publishMap(map);
    // Trajectory crossing the boundary
    std::vector<Vecf<3>> positions;
    for (float x = -2.0f; x <= 2.0f; x += 0.1f)
      positions.emplace_back(x, 0.0f, 0.0f);

    auto traj = makeTrajectoryFromPositions(
      positions, Vecf<3>(1, 0, 0));  // moving +x

    publishTrajectory(traj);
    auto occlusions = map.trajectoryIntersectsOcclusion(
      traj,
      1.0f,  // neighbor radius
      6      // min free neighbors
    );

    // ---- Assertions ----
    assert(!occlusions.empty());
    publishOcclusionNormals(occlusions);
    publishOcclusionVels(occlusions);
    // At least one occlusion should be near x = 0
    bool found_near_boundary = false;

    for (const auto& occ : occlusions)
    {
      if (std::abs(occ.position.x()) < 0.6f)
      {
        found_near_boundary = true;

        // Normal should point roughly +x
        Vecf<3> nhat = occ.normal.normalized();
        assert(nhat.dot(Vecf<3>(1, 0, 0)) > 0.9);

        // Velocity should align with +x
        Vecf<3> vhat = occ.velocity.normalized();
        assert(vhat.dot(Vecf<3>(1, 0, 0)) > 0.9);
      }
    }

    assert(found_near_boundary);

    std::cout << "[PASS] Trajectory planar occlusion test\n";
  }
  bool near(const Vecf<3>& p, const Vecf<3>& q, float tol)
  {
    return (p - q).norm() < tol;
  }

  bool nearY(const Vecf<3>& p, float y, float tol)
  {
    return std::abs(p.y() - y) < tol;
  }

  void testTrajectoryMultiplePlanarOcclusions()
  {
    TestableMapUtil map(
      0.5,
      -5, 5,
      -5, 5,
      -5, 5,
      0.0
    );

    map.initTestMap(
      Veci<3>(20, 20, 20),
      0.5,
      Vecf<3>(-5, -5, -5)
    );

    // ----------------------------
    // Define free space
    // Free if x < 0 OR y < 2
    // Unknown otherwise
    // ----------------------------
    for (int x = 0; x < 20; ++x)
      for (int y = 0; y < 20; ++y)
        for (int z = 0; z < 20; ++z)
        {
          float wx = -5.0f + x * 0.5f;
          float wy = -5.0f + y * 0.5f;

          if (wx < 0.0f || wy < 2.0f)
            map.setFree(Veci<3>(x, y, z));
        }

    publishMap(map);

    // ----------------------------
    // Non-straight trajectory
    // ----------------------------
    std::vector<Vecf<3>> positions;

    // Line goes from y = -1, x = -2 to x = 1.5 and then x = 1.5, y = -1.0 to y = 3.5
    // Segment 1: cross x = 0
    for (float x = -2.0f; x <= 1.5f; x += 0.1f)
      positions.emplace_back(x, -1.0f, 0.0f);

    // Segment 2: turn and cross y = 2
    for (float y = -1.0f; y <= 3.5f; y += 0.1f)
      positions.emplace_back(1.5f, y, 0.0f);

    auto traj = makeTrajectoryFromPositions(
      positions,
      Vecf<3>(1, 0, 0)   // nominal velocity (direction checked locally)
    );

    publishTrajectory(traj);

    auto occlusions = map.trajectoryIntersectsOcclusion(
      traj,
      1.0f,   // neighbor radius
      6       // min free neighbors
    );

    publishOcclusionNormals(occlusions);
    publishOcclusionVels(occlusions);

    // ----------------------------
    // Assertions
    // ----------------------------
    assert(occlusions.size() >= 2);

    bool found_x_plane = false;
    bool found_y_plane = false;

    for (const auto& occ : occlusions)
    {
      // ---- Occlusion near x = 0 ----
      if (std::abs(occ.position.x()) < 0.6f &&
          std::abs(occ.position.y() + 1.0f) < 0.6f)
      {
        found_x_plane = true;

        Vecf<3> nhat = occ.normal.normalized();
        assert(nhat.dot(Vecf<3>(1, 0, 0)) > 0.9);

        Vecf<3> vhat = occ.velocity.normalized();
        assert(vhat.dot(Vecf<3>(1, 0, 0)) > 0.5);
      }

      // ---- Occlusion near y = 2 ----
      if (std::abs(occ.position.y() - 2.0f) < 0.6f &&
          std::abs(occ.position.x() - 1.5f) < 0.6f)
      {
        found_y_plane = true;

        Vecf<3> nhat = occ.normal.normalized();
        assert(nhat.dot(Vecf<3>(0, 1, 0)) > 0.9);

        Vecf<3> vhat = occ.velocity.normalized();
        assert(vhat.dot(Vecf<3>(0, 1, 0)) > 0.5);
      }
    }

    assert(found_x_plane);
    assert(found_y_plane);
    for (const auto& occ : occlusions) {
      std::cout << "Occlusion at: " << occ.position.transpose()
                << " normal: " << occ.normal.transpose() << std::endl;
    }

    std::cout << "[PASS] Multi-occlusion non-straight trajectory test\n";
  }

void testTrajectoryWithSinglePlanarOcclusion_OR_logic()
  {
    TestableMapUtil map(
      0.5,
      -5, 5,
      -5, 5,
      -5, 5,
      0.0
    );

    map.initTestMap(
      Veci<3>(20, 20, 20),
      0.5,
      Vecf<3>(-5, -5, -5)
    );

    // ----------------------------
    // Define free space
    // Free if x < 0 OR y < 2
    // Unknown otherwise
    // ----------------------------
    for (int x = 0; x < 20; ++x)
      for (int y = 0; y < 20; ++y)
        for (int z = 0; z < 20; ++z)
        {
          float wx = -5.0f + x * 0.5f;
          float wy = -5.0f + y * 0.5f;

          if (wx < 0.0f || wy < 2.0f)
            map.setFree(Veci<3>(x, y, z));
        }


    // Non-straight trajectory
    std::vector<Vecf<3>> positions;
    // Line goes from y = -1, x = -2 to x = 1.5 and then x = 1.5, y = -1.0 to y = 3.5
    // Segment 1: cross x = 0
    for (float x = -2.0f; x <= 1.5f; x += 0.1f)
      positions.emplace_back(x, -1.0f, 0.0f);

    // Segment 2: turn and cross y = 2
    for (float y = -1.0f; y <= 3.5f; y += 0.1f)
      positions.emplace_back(1.5f, y, 0.0f);

    auto traj = makeTrajectoryFromPositions(
      positions,
      Vecf<3>(1, 0, 0)   // nominal velocity (direction checked locally)
    );

    auto occlusions = map.trajectoryIntersectsOcclusion(
      traj,
      0.5f,
      6
    );

    assert(!occlusions.empty());

    // ---- Assertions ----
    bool found_y_plane = false;
    bool found_bad_region = false;

    for (const auto& occ : occlusions)
    {
      // Must be near y ≈ 2
      if (nearY(occ.position, 2.0f, 0.6f))
      {
        found_y_plane = true;

        Vecf<3> nhat = occ.normal.normalized();
        assert(nhat.dot(Vecf<3>(0, 1, 0)) > 0.9);
      }

      // Must NOT be near y ≈ -1
      if (nearY(occ.position, -1.0f, 0.6f))
      {
        found_bad_region = true;
      }
    }

    for (const auto& occ : occlusions) {
      std::cout << "Occlusion at: " << occ.position.transpose()
                << " normal: " << occ.normal.transpose() << std::endl;
                for (const auto& occ : occlusions)
      // Test if the nearest free neighbor is close enough for this to be considered an occlusion
      {
        Vecf<3> nearest_free =
          nearestFreeVoxelCenter(map, occ.position);

        bool valid =
          withinVoxelInfluenceBand(
            occ.position,
            nearest_free,
            map.getRes(),
            0.5f); // neighbor radius

        assert(valid && "Occlusion outside expected voxel influence band");
      }

    }
    assert(found_y_plane && "Expected occlusion at y ≈ 2 not found");
    assert(!found_bad_region && "Unexpected occlusion in fully free space");

    std::cout << "[PASS] Single planar occlusion (OR logic) test\n";
  }

  int run_tests()
  {
    std::cout << "Running occlusion detection test..." << std::endl;

    // testFlatBoundaryOcclusion(); 
    // testAllFreeMap(); // test occlusion detection
    // testPartialOcclusion(); // test occlusion detection
    // testOcclusionNormalPlanar(); // test occlusion direction
    // testOcclusionNormalCorner(); // test occlusion direction
    // testExactKFreeNeighborsOcclusion(); // test occlusion detection for exact k free neighbors
    // testTrajectoryNoOcclusion(); // test trajectory occlusion detection
    // testTrajectoryWithPlanarOcclusion();
    // testTrajectoryMultiplePlanarOcclusions();
    testTrajectoryWithSinglePlanarOcclusion_OR_logic();
    std::cout << "Test passed." << std::endl;
    return 0;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OcclusionVizNode>());
  rclcpp::shutdown();
  return 0;
}