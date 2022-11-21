#include <ikd_Tree.h>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <algorithm>
#include <eigen3/Eigen/Dense>


using PointType = ikdTree_PointType;//Eigen::Vector3f;
using PointVector = KD_TREE<PointType>::PointVector;
PointVector RandomCloud(size_t size, float range) {
   PointVector result;
  result.reserve(size);
  for (size_t i = 0; i < size; i++) {
	  auto r = range * Eigen::Vector3f::Random();
    result.emplace_back(r[0], r[1], r[2]);
  }
  return result;
}


int main(int argc, char** argv)
{ 
	if(argc == 1)
	{
		const char* name = std::string("hello").c_str();
		puts(name);
	}
	PointVector points = RandomCloud(200, 20);
  auto *tree = new KD_TREE<PointType>(0.2,0.2,0.2);
  //tree->set_downsample_param(0.001);
  tree->Build(points);

  // search nn test
  for (size_t i = 0; i < points.size(); i++) {
    std::vector<PointType,Eigen::aligned_allocator<PointType>> points_near;
    std::vector<float> sqr_distances;
    tree->Nearest_Search(points[i], 1, points_near, sqr_distances);
    puts(points_near.size()==1?"OK":"FAIL");
    //EXPECT_EQ(points_near.size(), 1);
      puts(points_near[0].x==points[i].x?"OK":"FAIL");
      puts(points_near[0].y==points[i].y?"OK":"FAIL");
      puts(points_near[0].z==points[i].z?"OK":"FAIL");
    //EXPECT_FLOAT_EQ(0.0, sqr_distances[0]);
  }
  puts("done");
}
