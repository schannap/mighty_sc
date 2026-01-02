
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

  std::cout << "Test passed." << std::endl;
  return 0;
}
