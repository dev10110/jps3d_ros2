#include "jps3d_ros/interpolate.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

template <int N> using Vecf = Eigen::Matrix<double, N, 1>;
typedef Vecf<2> Vec2f;
typedef Vecf<3> Vec3f;

int main() {

  std::vector<double> ts = {0, 1, 2, 3, 4};
  std::vector<Vec2f> xs = {
      {0.0, 0.0}, {1.5, 1.5}, {2.0, 2.0}, {3.0, 3.0}, {4.0, 4.0}};

  Vec2f out = {0, 0};
  double t = 1.25;
  bool res = interpolate(out, t, ts, xs);

  std::cout << (res ? "Success" : "FAIL")  << ": " << out << std::endl;

  // now try interpolating at a fine mesh:
  std::vector<double> sample_ts { 0, 0.01, 0.02, 0.03, 0.04, 1.0, 1.01, 1.02, 1.99, 2.0, 2.1, 2.1, 3.99, 4.0};
  std::vector<Vec2f> outs;
  res = interpolate(outs, sample_ts, ts, xs);

  std::cout << (res ? "suc" : "fail")  << ": \n";
  
  std::cout << "size: " << outs.size() << std::endl;

  for (auto & o : outs)
  {
	  std::cout << o.transpose() << std::endl;
  }
  
  res = interpolate(outs, sample_ts, ts, xs, InterpolationMode::PRE);

  std::cout << (res ? "suc" : "fail")  << ": \n";
  
  std::cout << "size: " << outs.size() << std::endl;

  for (auto & o : outs)
  {
	  std::cout << o.transpose() << std::endl;
  }

}
