
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
int main(int argc, char** argv)
{
  std::cout << "Running occlusion detection test..." << std::endl;

  testFlatBoundaryOcclusion();

  std::cout << "Test passed." << std::endl;
  return 0;
}
