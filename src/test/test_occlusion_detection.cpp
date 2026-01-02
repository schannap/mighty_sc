
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
#include <stdlib.h>

// TODO: Make a wrapper ros2 node for this. It should publish visualization for the
// map and trajectory. 
// It should also publish visualization for the "direction of occlusion" arrows

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
};


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

  // Create a sampled trajectory from x = -5 to x = 5
  std::vector<Vecf<3>> positions;
  for (float x = -5; x <= 5; x += 0.1f)
    positions.emplace_back(x, 0, 0);

  auto traj = makeTrajectoryFromPositions(
    positions, Vecf<3>(1,0,0));

  auto occlusions = map.trajectoryIntersectsOcclusion(
    traj, // trajectory
    1.0,  // neighbor radius
    6     // min free neighbors
  );
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

  // x >= 0 remains unknown → occlusion boundary at x = 0

  // Trajectory crossing the boundary
  std::vector<Vecf<3>> positions;
  for (float x = -2.0f; x <= 2.0f; x += 0.1f)
    positions.emplace_back(x, 0.0f, 0.0f);

  auto traj = makeTrajectoryFromPositions(
    positions, Vecf<3>(1, 0, 0));  // moving +x

  auto occlusions = map.trajectoryIntersectsOcclusion(
    traj,
    1.0f,  // neighbor radius
    6      // min free neighbors
  );

  // ---- Assertions ----
  assert(!occlusions.empty());

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

//TODO: add more tests for detecting occlusions if there are intersections with trajectories

//TODO: Check comment in occlusion detection function in map_util.hpp. Should make the likelihood of being an 
// occlusion decay with the distance from the query unknown space to the free space.

int main(int argc, char** argv)
{
  std::cout << "Running occlusion detection test..." << std::endl;

  testFlatBoundaryOcclusion(); 
  testAllFreeMap(); // test occlusion detection
  testPartialOcclusion(); // test occlusion detection
  testOcclusionNormalPlanar(); // test occlusion direction
  testOcclusionNormalCorner(); // test occlusion direction
  testExactKFreeNeighborsOcclusion(); // test occlusion detection for exact k free neighbors
  testTrajectoryNoOcclusion(); // test trajectory occlusion detection
  testTrajectoryWithPlanarOcclusion();
  std::cout << "Test passed." << std::endl;
  return 0;
}
