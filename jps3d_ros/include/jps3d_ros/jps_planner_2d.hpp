#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"

// ros2 msgs
#include "dasc_msgs/msg/di_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "builtin_interfaces/msg/duration.hpp"

// tf2
#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#ifdef TF2_CPP_HEADERS
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

// JPS3D includes
#include <jps_basis/data_utils.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>
#include <jps_planner/jps_planner/jps_planner.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace JPS; // from the jps3d library

// define the hash function for the std::pair
struct pair_hash
{
	
	template<class T1, class T2>
	std::size_t operator()(std::pair<T1, T2> const &pair) const 
	{
		std::size_t h1 = std::hash<T1>()(pair.first);
		std::size_t h2 = std::hash<T2>()(pair.second);
		return h1 ^ h2;
	}

};


class JpsPlanner2D : public rclcpp::Node {

public:
  JpsPlanner2D();

private:
  // PARAMS
  std::string planning_frame_ = "vicon/world";
  std::string map_filepath_ =
      "/workspaces/isaac_ros-dev/src/jps3dROS2/jps3d/build/simple.yaml";
  double replan_rate_hz_ = 20.0;
  double goal_radius_ = 0.4; // meters
  double quad_radius_ = 0.2; // meters
  double desired_speed_ = 1.0; // m/s
  double resample_rate_hz_= 2.0; //hz

  // VARS
  std::shared_ptr<OccMapUtil> map_util_;
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  double map_z_ = 0.0;
  std::unordered_set<std::pair<double, double>, pair_hash> virtual_obstacles_;

  bool goal_updated_ = false;
  bool map_updated_ = false;

  double chebyshev_x_, chebyshev_y_, chebyshev_z_;

  // TF2 VARS
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_goal_;
  std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>>
      tf2_filter_;

  // SUBS
  rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_virtual_obs_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_chebyshev_center_;

  // PUBS
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<dasc_msgs::msg::DITrajectory>::SharedPtr pub_traj_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_traj_viz_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occ_grid_;
  rclcpp::Publisher<builtin_interfaces::msg::Duration>::SharedPtr pub_timer_dmp_;

  // Functions
  void callback_timer();
  void
  callback_goal(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg_ptr);
  void callback_map(const nvblox_msgs::msg::DistanceMapSlice &msg);
  void callback_virtual_obstacles(const geometry_msgs::msg::PointStamped &msg);
  void callback_chebyshev_center(const geometry_msgs::msg::PointStamped &msg);

  bool update_start_state();

  std::vector<double> allocate_times(const vec_Vec2f & path);
  std::vector<double> get_yaws(const vec_Vec2f & path);

  bool resample_path(vec_Vec2f & resampled_path, std::vector<double> & resampled_yaws, const vec_Vec2f & path, double dt);

  Vec2f project_to_map(Vec2f &goal);
  bool plan_path(vec_Vec2f &path, const Vec2f &start, const Vec2f &goal);

  void test();
  void set_test_map();
  void set_map_from_file();
  void write_map();
  void write_map(Vec2f start, Vec2f goal);
  void write_map(Vec2f start, Vec2f goal, vec_Vec2f path);

  void publish_path(const vec_Vec2f & path);
  void publish_traj(const vec_Vec2f & path, const std::vector<double> & yaws, double dt);

}; // class JpsPlanner2D
