/**
 * Using this file to have some concrete, verifiable test cases for detecting an occlusion, 
 * computing its normal direction, and computing the cost associated with a trajectory travelling
 * through that occlusion.
 */

#include <cassert>
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

  void setUnknown(const Veci<3>& p)
  {
    map_[getIndex(p)] = val_unknown_;
  }
  float getRes() const { return res_; }
  float getx_min() const { return x_map_min_;}
  float getx_max() const { return x_map_max_;}
  float gety_min() const { return y_map_min_;}
  float gety_max() const { return y_map_max_;}
  float getz_min() const { return z_map_min_;}
  float getz_max() const { return z_map_max_;}
  Vecf<3> getOrigin() const { return origin_d_; }
  Veci<3> getDim() const { return dim_; }
  const std::vector<int>& getMapData() const { return map_; }
  int8_t getValFree() const { return val_free_;}
  int8_t getValUnknown() const { return val_unknown_;}
  int8_t getValOcc() const { return val_occ_;}
};

void printOccInfo(OcclusionInfo info){
    std::cout << "OcclusionInfo:\n"
      << "  is_occlusion: " << info.is_occlusion << "\n"
      << "  free_neighbor_count: " << info.free_neighbor_count << "\n"
      << "  normal: ["
      << info.normal.x() << ", "
      << info.normal.y() << ", "
      << info.normal.z() << "]\n";
}

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

double pointToVoxelCubeDistance(
    const Vecf<3>& p,
    const Vecf<3>& c,
    double res)
{
    const double h = 0.5 * res;

    const double dx = std::max(0.0, std::abs(p.x() - c.x()) - h);
    const double dy = std::max(0.0, std::abs(p.y() - c.y()) - h);
    const double dz = std::max(0.0, std::abs(p.z() - c.z()) - h);

    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

 /**
 * Test whether there are any occlusions detected for a map that is completely free space.
 * 
 */
void testAllFreeMap()
{
    float x_min = -5.0f, y_min = -5.0f, z_min = -5.0f;
    float x_max =  5.0f, y_max =  5.0f, z_max =  5.0f;


    // create a free map
    TestableMapUtil map(
        0.5,   // res
        x_min, x_max, // x lims
        y_min, y_max, // y lims
        z_min, z_max, // z lims
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

    // test all points on the map
    for (float x = x_min; x < x_max; x+=0.1)
        for (float y = y_min; y < y_max; y+=0.1)
            for (float z = z_min; z < z_max; z+=0.1){
                auto occ = map.detectOcclusionAt(Vecf<3>(x, y, z), map.getRes(), 6);
                // assert !occ.is_occlusion
                if (occ.is_occlusion){
                    std::cerr << "Invalid occlusion found on a free map" << std::endl;
                    std::abort();
                }
            }

    std::cout << "[PASS] All-free map test\n";
}

 /**
  * Test whether an occlusion is detected when a corner cell is unknown
  * As a test, try with radius of res and neighbors 6. -> should fail
  * Also try with radius 6*s or some large number and neighbors 6. -> should pass
  */
void testCornerOcclusions(){

    // create a map that is all free
    float x_min = -5.0f, y_min = -5.0f, z_min = -5.0f;
    float x_max =  5.0f, y_max =  5.0f, z_max =  5.0f;


    // create a free map
    TestableMapUtil map(
        0.5,   // res
        x_min, x_max, // x lims
        y_min, y_max, // y lims
        z_min, z_max, // z lims
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
        for (int z = 0; z < 20; ++z){
            map.setFree(Veci<3>(x,y,z));
        }

    // fill in the corner voxels as unknown
    map.setUnknown(Veci<3>(0, 0, 0));
    map.setUnknown(Veci<3>(20, 0, 0));
    map.setUnknown(Veci<3>(0, 20, 0));
    map.setUnknown(Veci<3>(0, 0, 20));
    map.setUnknown(Veci<3>(0, 20, 20));
    map.setUnknown(Veci<3>(20, 20, 0));
    map.setUnknown(Veci<3>(20, 0, 20));
    map.setUnknown(Veci<3>(20, 20, 20));

    auto bottomleft = Vecf<3>(x_min, y_min, z_min);
    // with a radius of the resolution and 6 neighbors, we expect this to fail due to too few neighbors
    auto occ1 = map.detectOcclusionAt(bottomleft, map.getRes(), 6);
    // assert(!occ1.is_occlusion);
    if (occ1.is_occlusion){
        std::cerr << "Invalid occlusion found despite too few neighbors" << std::endl;
        std::abort();
        }
    // with a radius of the resolution and 3 neighbors, we expect this to pass due to large enough radius and enough neighbors
    auto occ2 = map.detectOcclusionAt(bottomleft, map.getRes(), 3);
    // assert(occ2.is_occlusion);
    if (!occ2.is_occlusion){
        std::cerr << "Occlusion not identified" << std::endl;
        std::abort();
    }
    // with a radius of the resolution/2 and 6 neighbors, we expect this to fail due to too small radius
    auto occ3 = map.detectOcclusionAt(bottomleft, map.getRes()/2.0, 6);
    // assert(!occ3.is_occlusion);
    if (occ3.is_occlusion){
        std::cerr << "Occlusion found despite too small neighbor radius" << std::endl;
        std::abort();
    }
    
    std::cout << "[PASS] Corner occlusion test\n";
}
 /**
  * Test scenarios in which occlusions exist mid-map and a deep unknown (not occlusion)
  */
void testOccludedMap(){
    // create a map with unknown region and free region
    // create a map that is all free
    float x_min = -5.0f, y_min = -5.0f, z_min = -5.0f;
    float x_max =  5.0f, y_max =  5.0f, z_max =  5.0f;


    TestableMapUtil map(
        0.5,   // res
        x_min, x_max, // x lims
        y_min, y_max, // y lims
        z_min, z_max, // z lims
        0.0   // inflation
    );

    map.initTestMap(
        Veci<3>(20, 20, 20),
        0.5,
        Vecf<3>(-5, -5, -5)
    );

    // index value for x at boundary
    int boundary_x_i = 10;
    // from x indices 0 to 10 is free. Otherwise map is unknown
    for (int x = 0; x < boundary_x_i; ++x)
        for (int y = 0; y < 20; ++y)
            for (int z = 0; z < 20; ++z){
                map.setFree(Veci<3>(x,y,z));
        }

    // world value for x at boundary
    float boundary_x_f =  map.getx_min() + (boundary_x_i + 0.5) * map.getRes();

    // if in deep free, should not be occlusion
    auto free_vox = Vecf<3>(-2.0f, -2.0f, 0.0f);
    auto occ1 = map.detectOcclusionAt(free_vox, map.getRes(), 3);
    // assert(!occ1.is_occlusion);
    if (occ1.is_occlusion){
        std::cerr << "Invalid occlusion found in deep free space" << std::endl;
        std::abort();
    }
    // if in feep unknown space, should not be an occlusion
    auto unknown_deep = Vecf<3>(3.0f, 4.0f, 0.0f);
    auto occ2 = map.detectOcclusionAt(unknown_deep, map.getRes(), 3);
    // assert(!occ2.is_occlusion);
    if (occ2.is_occlusion){
        std::cerr << "Invalid occlusion found in deep unknown space" << std::endl;
        std::abort();
    }
    // if on boundary on unknown and free - should be an occlusion
    auto occluded_1 = Vecf<3>(boundary_x_f + map.getRes()/2.0, 2.3, 0.0);
    auto occ3 = map.detectOcclusionAt(occluded_1, map.getRes(), 3);
    
    // assert(occ3.is_occlusion);
    if (!occ3.is_occlusion){
        std::cerr << "Occlusion not identified 1" << std::endl;
        std::abort();
    }
    // if EXACTLY on boundary on unknown and free - should be an occlusion
    float radius = map.getRes();
    auto occluded_2 = Vecf<3>(boundary_x_f+radius-0.001, 2.3, 0.0);
    auto occ4 = map.detectOcclusionAt(occluded_2, radius, 3);
    // assert(occ4.is_occlusion);
    if (!occ4.is_occlusion){
        std::cerr << "Occlusion not identified 2" << std::endl;
        std::abort();
    }
    // if EXACTLY outside of boundary on unknown and free - should not be an occlusion
    auto occluded_3 = Vecf<3>(boundary_x_f+radius+0.001, 2.3, 0.0);
    auto occ5 = map.detectOcclusionAt(occluded_3, radius, 3);
    // assert(occ4.is_occlusion);
    if (occ5.is_occlusion){
        std::cerr << "Occlusion incorrectly identified 3" << std::endl;
        std::abort();
    }
    std::cout << "[PASS] Occluded Map Simple\n";


}

/**
 * Test whether an occlusion is identified with correct normal direction for a 
 * planar boundary of free and unknown
 */
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

    // Let everything x < index 10 be free and x >= index 10 be unknown
    int boundary_x_i = 10;
    for (int x = 0; x < 10; ++x)
        for (int y = 0; y < 20; ++y)
        for (int z = 0; z < 20; ++z)
            map.setFree(Veci<3>(x,y,z));

    // world value for x at boundary
    float boundary_x_f =  map.getx_min() + (boundary_x_i + 0.5) * map.getRes();

    Vecf<3> p(boundary_x_f, 0.0, 0.0);  // at the boundary
    auto occ = map.detectOcclusionAt(p, map.getRes(), 6);

    if (!occ.is_occlusion){
        std::cerr << "Occlusion not identified in testOcclusionNormalPlanar" << std::endl;
        std::abort();
    }

    Vecf<3> expected_norm(1, 0, 0);
    float cos_angle = occ.normal.normalized().dot(expected_norm);
    if (cos_angle <= 0.9){
        printOccInfo(occ);
        std::cerr << "Incorrect normal direction in testOcclusionNormalPlanar" << std::endl;
        std::abort();
    }

    std::cout << "[PASS] Planar occlusion normal test\n";
}

/**
 * Test whether an occlusion and the normal vectory are correctly identified for a map
 * with a corner of free space and everything else unknown 
 */
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

    int boundary_x_i = 10;
    int boundary_y_i = 10;
    // Let everything x < idx 10 AND y < idx 10 be free and rest unknown
    for (int x = 0; x < 10; ++x)
        for (int y = 0; y < 10; ++y)
        for (int z = 0; z < 20; ++z)
            map.setFree(Veci<3>(x,y,z));

    // world value for x and y at boundary
    float boundary_x_f =  map.getx_min() + (boundary_x_i + 0.5) * map.getRes();
    float boundary_y_f =  map.gety_min() + (boundary_y_i + 0.5) * map.getRes();

    Vecf<3> p(boundary_x_f, boundary_y_f, 0.0);  // near (0,0,0)
    auto occ = map.detectOcclusionAt(p, map.getRes(), 3);
    if (!occ.is_occlusion){
        printOccInfo(occ);
        std::cerr << "Occlusion not identified in testOcclusionNormalCorner" << std::endl;
        std::abort();
    }

    Vecf<3> expected(1, 1, 0);
    float cos_angle = occ.normal.normalized().dot(expected);
    if (cos_angle <= 0.9){
        printOccInfo(occ);
        std::cerr << "Incorrect normal direction in testOcclusionNormalCorner" << std::endl;
        std::abort();
    }

    std::cout << "[PASS] Planar occlusion corner normal test\n";
}

/**
 * Test whether an occlusion is identified when exactly k free neighbors are available
 * (edge case behavior)
 */
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
        Veci<3>(5,6,1),
        // Veci<3>(5,5,0), // Not free for this test
        // Veci<3>(5,5,2)
    };

    for (const auto& p : free_neighbors)
        map.setFree(p);

    Vecf<3> point = map.intToFloat(q);
    auto occ = map.detectOcclusionAt(point, map.getRes(), k);
    if (!occ.is_occlusion){
        printOccInfo(occ);
        std::cerr << "Occlusion not identified in testExactKFreeNeighborsOcclusion" << std::endl;
        std::abort();
    }
    
    // Direction should be near zero due to symmetry
    if (!(occ.normal.norm() < 1e-3)){
        printOccInfo(occ);
        std::cerr << "Incorrect normal direction in testExactKFreeNeighborsOcclusion" << std::endl;
        std::abort();
    }

    std::cout << "[PASS] Exact-k free neighbors occlusion test\n";
    }

/**
 * Test whether multiple samples at an occlusion have the same (similar) occlusion normal direction
 */
void testNormalSignConsistency()
{
    // Same planar setup as testOcclusionNormalPlanar

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

    // Let everything x < index 10 be free and x >= index 10 be unknown
    int boundary_x_i = 10;
    for (int x = 0; x < 10; ++x)
        for (int y = 0; y < 20; ++y)
        for (int z = 0; z < 20; ++z)
            map.setFree(Veci<3>(x,y,z));

    // world value for x at boundary
    float boundary_x_f =  map.getx_min() + (boundary_x_i + 0.5) * map.getRes();

    // Sample multiple y,z positions
    for (float y = -0.5; y <= 0.5; y += 0.1)
        for (float z = -0.5; z <= 0.5; z += 0.1)
        {
            Vecf<3> p(boundary_x_f, y, z);
            auto occ = map.detectOcclusionAt(p, map.getRes(), 6);

            Vecf<3> into_unknown(1, 0, 0);
            // assert(occ.normal.dot(into_unknown) > 0);
            if (occ.normal.dot(into_unknown) <= 0){
                printOccInfo(occ);
                std::cerr << "Incorrect normal sign in testNormalSignConsistency" << std::endl;
                std::abort();
            }
        }
        std::cout << "[PASS] Normal sign consistency test\n";

}

/**
 * Test multiple points epsilon appart and ensure their normal directions are very similar
 * using a planar boundary
 */
void testNormalStabilityAlongBoundary()
{
    TestableMapUtil map(
        0.1,
        -1, 1,
        -1, 1,
        -1, 1,
        0.0
    );

    map.initTestMap(
        Veci<3>(20, 20, 20),
        0.1,
        Vecf<3>(-1, -1, -1)
    );

    int boundary_x_i = 10;

    for (int x = 0; x < boundary_x_i; ++x)
        for (int y = 0; y < 20; ++y)
            for (int z = 0; z < 20; ++z)
                map.setFree(Veci<3>(x,y,z));

    float boundary_x_f =
        map.getx_min() + (boundary_x_i + 0.5f) * map.getRes();

    Vecf<3> expected(1, 0, 0);

    for (float y = -0.8f; y <= 0.8f; y += 0.1f)
    {
        Vecf<3> p(boundary_x_f, y, 0.0f);
        if (map.isFree(p)){
            continue;
        }
        else{
            auto occ = map.detectOcclusionAt(p, map.getRes(), 1);
            if (!occ.is_occlusion)
            {
                std::cerr << "Occlusion missed along boundary\n";
                std::abort();
            }

            float cos_angle =
                occ.normal.normalized().dot(expected);

            if (cos_angle < 0.95f)
            {
                printOccInfo(occ);
                std::cerr << "Normal instability along planar boundary\n";
                std::abort();
            }
        }
    }

    std::cout << "[PASS] Boundary normal stability test\n";
}

/**
 * Test occlusion identification and normal vector computation for a staircase pattern on map.
 */
void testOcclusionNormalStaircase()
{
    TestableMapUtil map(
        0.2,
        -2, 2,
        -2, 2,
        -1, 1,
        0.0
    );

    map.initTestMap(
        Veci<3>(20, 20, 5),
        0.2,
        Vecf<3>(-2, -2, -1)
    );

    // Staircase boundary: x < y is free
    for (int y = 0; y < 20; ++y)
    {
        int x_free_max = y / 2;  // slow slope
        for (int x = 0; x < x_free_max; ++x)
            for (int z = 0; z < 5; ++z)
                map.setFree(Veci<3>(x, y, z));
    }

    // Query UNKNOWN voxel just past boundary
    Veci<3> q(5, 10, 2);  // unknown but near free
    Vecf<3> p = map.intToFloat(q);

    auto occ = map.detectOcclusionAt(p, map.getRes(), 1);

    if (!occ.is_occlusion)
    {
        printOccInfo(occ);
        std::cout << "  is_free: " << map.isFree(p) << "\n";
        std::cout << "  is_outside: " << map.isOutside(p) << "\n";
        std::cout << "  x: " << p.x() << "\n";
        std::cout << "  y: " << p.y() << "\n";
        std::cout << "  z: " << p.z() << "\n";
        std::cerr << "Failed staircase occlusion test\n";
        std::abort();
    }

    // Expected normal roughly (1, -0.5, 0)
    Vecf<3> expected(1.0, -0.5, 0.0);
    expected.normalize();

    if (occ.normal.dot(expected) < 0.8)
    {
        printOccInfo(occ);
        std::cerr << "Incorrect staircase normal\n";
        std::abort();
    }

    std::cout << "[PASS] Staircase boundary occlusion test\n";
}

/**
 * Test occlusion detection with larger neighbor radius to make sure normal vector is skewed
 * towards the direction with most free voxels.
 */
void testUnevenNeighborDistribution()
{
    TestableMapUtil map(
        0.5,
        -5, 5,
        -5, 5,
        -5, 5,
        0.0
    );

    map.initTestMap(
        Veci<3>(10, 10, 10),
        0.5,
        Vecf<3>(-5, -5, -5)
    );

    Veci<3> q(5, 5, 5);

    // Dense free region on +x side
    for (int dx = 1; dx <= 3; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz)
                map.setFree(q + Veci<3>(dx, dy, dz));

    // Sparse free voxels elsewhere
    map.setFree(q + Veci<3>(-1, 0, 0));
    map.setFree(q + Veci<3>(0, -1, 0));

    Vecf<3> p = map.intToFloat(q);
    auto occ = map.detectOcclusionAt(p, 2*map.getRes(), 4);

    if (!occ.is_occlusion)
    {
        printOccInfo(occ);
        std::cerr << "Occlusion missed in anisotropic test\n";
        std::abort();
    }

    Vecf<3> expected(-1, 0, 0);
    float cos_angle =
        occ.normal.normalized().dot(expected);

    if (cos_angle < 0.9f)
    {
        std::cout << "  cos similarity: " << cos_angle << "\n";
        printOccInfo(occ);
        std::cerr << "Incorrect anisotropic normal direction\n";
        std::abort();
    }

    std::cout << "[PASS] Anisotropic neighbor normal test\n";
}

/**
 * A complicated map?
 */

int main(){
    testAllFreeMap();
    testCornerOcclusions();
    testOccludedMap();
    testOcclusionNormalPlanar();
    testOcclusionNormalCorner();
    testExactKFreeNeighborsOcclusion();
    testNormalSignConsistency();
    testNormalStabilityAlongBoundary();
    testOcclusionNormalStaircase();
    testUnevenNeighborDistribution();
}