#include "jps3d_ros/jps_planner_2d.hpp"
#include "jps3d_ros/interpolate.hpp"
#include "jps3d_ros/timer.hpp"
#include "read_map.hpp"

using namespace JPS;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Accel;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseArray;
using nav_msgs::msg::Path;
using nvblox_msgs::msg::DistanceMapSlice;
using dasc_msgs::msg::DITrajectory;
using std::placeholders::_1;
using namespace std::chrono_literals;

JpsPlanner2D::JpsPlanner2D() : Node("jps_planner_2d") {

  RCLCPP_INFO(get_logger(), "starting...");

  // PARAMS
  planning_frame_ =
      this->declare_parameter<std::string>("planning_frame", planning_frame_);
  replan_rate_hz_ =
      this->declare_parameter<double>("replan_rate_hz", replan_rate_hz_);
  quad_radius_ = this->declare_parameter<double>("quad_radius_m", quad_radius_);
  desired_speed_ = this->declare_parameter<double>("desired_speed_m_per_s", desired_speed_);
  resample_rate_hz_ = this->declare_parameter<double>("resample_rate_hz", resample_rate_hz_);
  smoothing_param_ =
      this->declare_parameter<double>("smoothing_param", smoothing_param_);
  dmp_potential_radius_ = this->declare_parameter<double>(
      "dmp_potential_radius_m", dmp_potential_radius_);

  // VARS
  map_util_ = std::make_shared<OccMapUtil>(); // specifically for 2D

  // PUBS
  pub_path_ = this->create_publisher<Path>("/path", 10);
  pub_path_jps_ = this->create_publisher<Path>("/path_jps", 10);
  pub_traj_ =
      this->create_publisher<DITrajectory>("/nominal_traj", 10);
  pub_traj_viz_ = 
    this-> create_publisher<PoseArray>("/nominal_traj/viz", 10);
  pub_occ_grid_ = 
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_map", 10);
  pub_timer_dmp_ = this-> create_publisher<builtin_interfaces::msg::Duration>("/diagnostics/timing/planning_dmp", 1);

  // TF SUBS
  std::chrono::duration<int> buffer_timeout(1);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  sub_goal_.subscribe(this, "/goal_pose");

  tf2_filter_ =
      std::make_shared<tf2_ros::MessageFilter<PoseStamped>>(
          sub_goal_, *tf2_buffer_, planning_frame_, 100,
          this->get_node_logging_interface(), this->get_node_clock_interface(),
          buffer_timeout);
  tf2_filter_->registerCallback(&JpsPlanner2D::callback_goal, this);

  // SUBS
  sub_map_ = this->create_subscription<DistanceMapSlice>(
      "/nvblox_node/map_slice", 10,
      std::bind(&JpsPlanner2D::callback_map, this, _1));
  sub_virtual_obs_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "virtual_obstacles", 100,
          std::bind(&JpsPlanner2D::callback_virtual_obstacles, this, _1));
  sub_chebyshev_center_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "/nvblox_node/sfc/chebyshev_center", 10,
          std::bind(&JpsPlanner2D::callback_chebyshev_center, this, _1));

  // Timer
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / replan_rate_hz_),
      std::bind(&JpsPlanner2D::callback_timer, this));

  test();
}

void JpsPlanner2D::callback_chebyshev_center(const geometry_msgs::msg::PointStamped &msg)
{
	if (msg.header.frame_id != "vicon/world") {
		RCLCPP_WARN(get_logger(), "chebyshev center is not in frame vicon/world");
		return;
	}

	chebyshev_x_ = msg.point.x;
	chebyshev_y_ = msg.point.y;
	chebyshev_z_ = msg.point.z;

}


void JpsPlanner2D::callback_virtual_obstacles(const geometry_msgs::msg::PointStamped &msg)
{

  if (msg.header.frame_id != "vicon/world")
  {
    RCLCPP_WARN(get_logger(), "virtual obstacle message expected to be in frame vicon/world");
    return;
  }

  // get the 2d index, wrt to the current map
  Vec2f obs = { msg.point.x, msg.point.y };
  Vec2i obs_i = map_util_->floatToInt(obs);
  
  // convert back to a obs point
  constexpr int D = 1;
  for (int i=-D; i <= D; i++) {
	  for (int j=-D; j<=D; j++) {
		  Vec2f obs_f = map_util_->intToFloat(obs_i + Vec2i(i,j));
                  // add it to the set
                  // virtual_obstacles_.insert(
                  //		  std::make_pair(obs_f(0), obs_f(1)));
                  virtual_obstacles_.insert(
                      {obs_f(0), obs_f(1), get_clock()->now()});
          }
  }


  RCLCPP_INFO(get_logger(), "Total %zu virtual obstacles" , virtual_obstacles_.size());

}

bool JpsPlanner2D::update_start_state() {
  geometry_msgs::msg::TransformStamped t;

  std::string toFrame = "vicon/px4_1/px4_1";

  try {
    t = tf2_buffer_->lookupTransform(planning_frame_, toFrame,
                                     tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                toFrame.c_str(), planning_frame_.c_str(), ex.what());
    return false;
  }

  start_pose_.header = t.header;
  start_pose_.pose.position.x = t.transform.translation.x;
  start_pose_.pose.position.y = t.transform.translation.y;
  start_pose_.pose.position.z = t.transform.translation.z;
  start_pose_.pose.orientation = t.transform.rotation;
  start_yaw_ = tf2::getYaw(start_pose_.pose.orientation);

  return true;
}

void JpsPlanner2D::callback_timer() {

  timer::Timer mytimer("replan_timer", false);


  // check that a goal has been specified
  if (!goal_updated_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *(get_clock()), 1000.0, "no goal location set");
    return;
  }

  if (!map_updated_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *(get_clock()), 1000.0, "no map");
    return;
  }

  // get the current location of the robot
  bool suc = update_start_state();
  if (!suc) {
    RCLCPP_WARN(get_logger(), "could not get start state");
    return;
  }

  // passed all the checks
  Vec2f start = {start_pose_.pose.position.x, start_pose_.pose.position.y};
  Vec2f goal = {goal_pose_.pose.position.x, goal_pose_.pose.position.y};
  
  // check if we have reached the goal
  if ((goal - start).norm() <= goal_radius_)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *(get_clock()), 500.0, "goal reached");
    return;
  }
  
  vec_Vec2f path;
  // check if the start is in an obstacle
  Vec2i start_idx = map_util_->floatToInt(start);
  if (! map_util_ -> isFree(start_idx) ) 
  {

    // RCLCPP_WARN(get_logger(), "start is not free!!");
    // return;

    // publish straight line path to the chebyshev center

    Vec2f chebyshev_goal = {chebyshev_x_, chebyshev_y_};
    goal(0) = chebyshev_goal(0);
    goal(1) = chebyshev_goal(1); // modify the goal
    path.push_back(start);
    path.push_back(chebyshev_goal);

  }
  else {
  
  // run jps

  Vec2f projected_goal = project_to_map(goal);

  suc = plan_path(path, start, projected_goal);
  

  if (!suc) {
    RCLCPP_WARN(get_logger(), "plan_path failed");
    write_map(start, projected_goal);
    return;
  }
  // add the true goal location
  path.push_back(goal);
  }

  RCLCPP_INFO(get_logger(), mytimer.log("path planned").c_str()); 


  // resample path
  double dt = 1.0 / resample_rate_hz_;
  vec_Vec2f resampled_path;
  std::vector<double> resampled_yaws;
  suc = resample_path(resampled_path, resampled_yaws, path, dt);
  if (!suc) {
    RCLCPP_WARN(get_logger(), "resampling failed");
    return;
  }
  
  // push back the goal
  resampled_path.push_back(goal);
  resampled_yaws.push_back(resampled_yaws.back());

  // smoothen the path
  // smoothen_path(resampled_path, resampled_yaws, resampled_yaws[0]);
  smoothen_path(resampled_path, resampled_yaws, start_yaw_);

  //// skip the first few
  //for (int i =0; i < 4; i++){
  //resampled_path.erase(resampled_path.begin());
  //resampled_yaws.erase(resampled_yaws.begin());
  //}

  // RCLCPP_INFO(get_logger(), mytimer.log("resampled path").c_str());

  publish_path(path);
  publish_traj(resampled_path, resampled_yaws, dt);

  write_map(start, goal, path);

  RCLCPP_INFO(get_logger(), mytimer.log("total").c_str());
  
  // publish timing info
  builtin_interfaces::msg::Duration timer_msg;
  timer_msg.sec = 0;
  timer_msg.nanosec = static_cast<uint32_t>(mytimer.elapsed_ns());
  pub_timer_dmp_->publish(timer_msg);
  
}

void JpsPlanner2D::smoothen_path(vec_Vec2f &path, std::vector<double> &yaws,
                                 double initial_yaw) {

  double f = smoothing_param_;
  for (std::size_t i = 0; i < yaws.size(); i++) {

    if (i == 0) {
      yaws[0] = interpolate_angles(yaws[0], initial_yaw, f);

    } else {
      yaws[i] = interpolate_angles(yaws[i], yaws[i - 1], f);
    }
  }
}

void JpsPlanner2D::publish_path(const vec_Vec2f & path) {

  nav_msgs::msg::Path path_msg;
  path_msg.header = start_pose_.header; // this is in the planning_frame_
                                        // already, and has the correct stamp.

  // orientation logic:
  // for all states within some radius of the goal state, set the target yaw to
  // the goal yaw for the rest, follow the path

  std::size_t N = path.size();
  path_msg.poses.reserve(N);
  
  std::vector<double> goal_yaws;
  goal_yaws.resize(N);

  for (std::size_t i = 0; i < N; i++) {

    PoseStamped p;
    p.header = path_msg.header;
    p.pose.position.x = path[i](0);
    p.pose.position.y = path[i](1);
    p.pose.position.z = map_z_;

    double distToGoal = (path[i] - path.back()).norm();
    if (distToGoal > 0.5 && i < N - 1) {
      double delta_x = path[i + 1](0) - path[i](0);
      double delta_y = path[i + 1](1) - path[i](1);
      double yaw = std::atan2(delta_y, delta_x);
      tf2::Quaternion tf2_quat;
      tf2_quat.setRPY(0, 0, yaw);
      p.pose.orientation = tf2::toMsg(tf2_quat);
      goal_yaws[i] = yaw;
    } else {
      p.pose.orientation = goal_pose_.pose.orientation;
      goal_yaws[i] = tf2::getYaw(p.pose.orientation);
    }

    path_msg.poses.push_back(p);
  }

  // publish the path msg
  pub_path_jps_->publish(path_msg);
}

void JpsPlanner2D::publish_traj(const vec_Vec2f & path, const std::vector<double> & yaws, double dt)
{

  // construct the message
  dasc_msgs::msg::DITrajectory traj_msg;
  traj_msg.header = start_pose_.header;
  traj_msg.dt = dt;

  const Vec2f goal = path.back();
  
  std::size_t N = path.size();
  if (N == 0){
    RCLCPP_WARN(get_logger(), "path has 0 length");
    return;
  }

  for (std::size_t i =0; i < N ; ++i)
  {
    // pose
    Pose p;
    p.position.x = path[i](0);
    p.position.y = path[i](1);
    p.position.z = map_z_;

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0,0,yaws[i]);
    p.orientation = tf2::toMsg(tf2_quat);

    traj_msg.poses.push_back(p);

    // vel
    Twist twist;
    if ((path[i] - goal).norm() > goal_radius_) 
    {
    Vec2f v =  (1.0/dt) * (path[std::min(i+1, N-1)] - path[i]);
    twist.linear.x = v(0);
    twist.linear.y = v(1);
    twist.linear.z = 0.0;
    }

    traj_msg.twists.push_back(twist);

    traj_msg.accelerations.emplace_back(Accel());

  }

  // publish the message
  pub_traj_-> publish(traj_msg);


  // construct the visualization message
  geometry_msgs::msg::PoseArray traj_viz_msg;
  traj_viz_msg.header = traj_msg.header;
  traj_viz_msg.poses = traj_msg.poses;

  pub_traj_viz_ -> publish(traj_viz_msg);

}

bool JpsPlanner2D::resample_path(vec_Vec2f & resampled_path, std::vector<double> & resampled_yaws, const vec_Vec2f & path, double dt){

        // first assign the timepoints to each element of path
	std::vector<double> ts = allocate_times(path);

        // fill the sample times vector
	std::size_t N = (ts.back() / dt); // total number of samples
	std::vector<double> sample_ts(N, 0);
	for (std::size_t i=0; i < N; ++i)
	{
		sample_ts[i] = i * dt;
	}
	
	// resample the path vector
	bool res = interpolate(resampled_path, sample_ts, ts, path);

	if (!res){
    RCLCPP_WARN(get_logger(), "interpolation failed");
		return false;
	}
	
	
	// determine yaws
        resampled_yaws = get_yaws(resampled_path);

        return true;

}

std::vector<double> JpsPlanner2D::allocate_times(const vec_Vec2f & path)
{

	const double v = desired_speed_;

	std::vector<double> ts(path.size(), 0.0);
	
	ts[0] = 0;
	for (std::size_t i = 1; i < path.size(); ++i)
	{

		double d = (path[i] - path[i-1]).norm();
		ts[i] = ts[i-1] + d / v;

	}

	return ts;

}

std::vector<double> JpsPlanner2D::get_yaws(const vec_Vec2f & path)
{

        double goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);

	double sq_goal_radius = goal_radius_ * goal_radius_;

        // fill the yaws with the goal yaw
        std::vector<double> yaws(path.size(), goal_yaw);

	const Vec2f goal = path.back();

        // if the point is far from the goal, specify a new yaw
        for (std::size_t i=0; i < path.size() - 1; ++i)
	{

		if ( (path[i] - goal).squaredNorm() > sq_goal_radius) {

			double delta_x = path[i + 1](0) - path[i](0);
			double delta_y = path[i + 1](1) - path[i](1);
			yaws[i] = std::atan2(delta_y, delta_x);
		}

	}

	return yaws;

}



Vec2f JpsPlanner2D::project_to_map(Vec2f &goal) {

  Vec2i pt = map_util_->floatToInt(goal);

  if (map_util_->isOutside(pt)) {
    Vec2i dim = map_util_->getDim();

    // modify each dim
    for (int i = 0; i < 2; i++) {
      pt(i) = std::max(0, std::min(dim(i) - 1, pt(i)));
    }

    return map_util_->intToFloat(pt);
  }

  return goal;
}

bool JpsPlanner2D::plan_path(vec_Vec2f &path, const Vec2f &start,
                             const Vec2f &goal) {
  // Set up JPS planner
  JPSPlanner2D jps(false);   // Declare a planner
  jps.setMapUtil(map_util_); // Set collision checking function
  jps.updateMap();

  // Run JPS planner
  timer::Timer time_jps("timer_jps", false);
  bool suc = jps.plan(start, goal, 1, true);

  if (!suc) {
    RCLCPP_WARN(get_logger(), "jps failed to find a path");
    return false;
  }

  auto path_jps = jps.getRawPath();         // Get the planned raw path from JPS
  path_jps = jps.removeCornerPts(path_jps); // simplify path
  // path_jps = jps.removeLinePts(path_jps);  // simplify path
  // RCLCPP_INFO(get_logger(), time_jps.log("JPS").c_str());

  // Set up DMP planner
  DMPlanner2D dmp(false);
  dmp.setPotentialRadius(
      Vec2f(dmp_potential_radius_,
            dmp_potential_radius_)); // Set 2D potential field radius
  dmp.setSearchRadius(
      Vec2f(0.5, 0.5));         // Set the valid search region around given path
  dmp.setMap(map_util_, start); // Set map util for collision checking, must be
                                // called before planning

  // Run DMP planner
  timer::Timer time_dist("time_dist", false);
  bool suc_dmp = dmp.computePath(
      start, goal, path_jps); // Compute the path given the jps path
  if (!suc_dmp) {
    RCLCPP_WARN(get_logger(), "dmp failed");
    return false;
  }

  auto path_dist = dmp.getRawPath();

  path_dist = jps.removeLinePts(path_dist); // simplify path
  // RCLCPP_INFO(get_logger(), time_dist.log("DIST").c_str());


  // copy over the answer
  path = path_dist;

  return true;
}

void JpsPlanner2D::callback_goal(const PoseStamped::SharedPtr goal_msg_ptr) {

  try {
    tf2_buffer_->transform(*goal_msg_ptr, goal_pose_, planning_frame_);
    RCLCPP_INFO(get_logger(), "Goal: %f, %f, %f", goal_pose_.pose.position.x,
                goal_pose_.pose.position.y, goal_pose_.pose.position.z);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Failure %s\n", ex.what());
  }

  goal_updated_ = true;
}

void JpsPlanner2D::callback_map(const DistanceMapSlice &msg) {

  timer::Timer map_timer("map_timer", false);

  // check that it is in the correct frame
  if (msg.header.frame_id != planning_frame_) {
    RCLCPP_WARN(get_logger(), "map is in frame %s, not the planning frame: %s",
                msg.header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }

  // load the data in
  Vec2f origin = {msg.origin.x, msg.origin.y};
  Vec2i dim = {msg.width, msg.height};
  double res = msg.resolution;

  const bool mark_unknown_as_free = true;

  std::vector<int8_t> data;
  // occupancy data, the subscript follows: id = x + dim.x * y + dim.x * dim.y *
  // z; occupied: 100, free: 0, unknown: -1
  data.resize(msg.data.size());
  for (std::size_t i = 0; i < msg.data.size(); i++) {
    if (msg.data[i] == msg.unknown_value) {
      if (mark_unknown_as_free) {
        data[i] = 0; // free
      } else {
        data[i] = -1; // unknown
      }
    } else if (msg.data[i] < quad_radius_) {
      data[i] = 100; // occupied
    } else {
      data[i] = 0; // free
    }
  }
  
  map_util_->setMap(origin, dim, data, res);

  // add in the virtual obstacles
  for (auto &vobs : virtual_obstacles_) {

    // // check if i should delete the element
    // if ((get_clock()->now() - vobs.t).seconds() > 10.0)
    // {
    //         virtual_obstacles_.erase(vobs);
    //         continue;
    // }

    // Vec2f p = {pair.first, pair.second};
    Vec2f p = {vobs.x, vobs.y};

    // convert to index based on map
    Vec2i pi = map_util_-> floatToInt(p);

    // if in bounds
    if (!map_util_-> isOutside(pi))
    {
      data[map_util_->getIndex(pi)] = 100; // Occupied
      // RCLCPP_DEBUG(get_logger(), "marked cell (%f, %f) as virtual obstacle", p(0), p(1));
    }
    else {
      RCLCPP_DEBUG(get_logger(), "virtual obs (%f, %f) is out of  map bounds", p(0), p(1));
    }
  }

  map_util_->setMap(origin, dim, data, res);

  RCLCPP_INFO(get_logger(), map_timer.log("set_map").c_str());

  // auto it = std::minmax_element(msg.data.begin(), msg.data.end());
  // RCLCPP_INFO(
  //     get_logger(),
  //     "loaded map of size %d x %d = (%d), with %zu cells, and min: %f, max: %f",
  //     msg.width, msg.height, msg.width * msg.height, msg.data.size(), *it.first,
  //     *it.second);

  // write map
  // write_map();

  map_updated_ = true;
  map_z_ = msg.origin.z;


  // publish map as occupancy grid
  //
  if (pub_occ_grid_->get_subscription_count() > 0 ) { 
  nav_msgs::msg::OccupancyGrid occ_grid_msg;
  occ_grid_msg.header = msg.header;
  occ_grid_msg.data = data;
  occ_grid_msg.info.map_load_time = msg.header.stamp;
  occ_grid_msg.info.resolution = res;
  occ_grid_msg.info.width = msg.width;
  occ_grid_msg.info.height = msg.height;
  occ_grid_msg.info.origin.position.x = origin(0);
  occ_grid_msg.info.origin.position.y = origin(1);
  occ_grid_msg.info.origin.position.z = map_z_;


  pub_occ_grid_->publish(occ_grid_msg);

  RCLCPP_INFO(get_logger(), map_timer.log("publish_occ_grid_map").c_str());
  } 

}

void JpsPlanner2D::test() {

  set_test_map();
  // set_map_from_file(map_util_);

  const Vec2f start(0.5, 9.5);
  const Vec2f goal(19.5, 0.5);

  // Set up JPS planner
  JPSPlanner2D jps(false);   // Declare a planner
  jps.setMapUtil(map_util_); // Set collision checking function
  jps.updateMap();

  // Run JPS planner
  timer::Timer time_jps("timer_jps", false);
  jps.plan(start, goal, 1, true);
  double dt_jps = time_jps.elapsed_us();
  const auto path_jps = jps.getRawPath(); // Get the planned raw path from JPS
  printf("JPS Planner takes: %f us\n", dt_jps);
  printf("JPS Path Distance: %f\n", total_distance2f(path_jps));

  // Set up DMP planner
  DMPlanner2D dmp(false);
  dmp.setPotentialRadius(Vec2f(1.0, 1.0)); // Set 2D potential field radius
  dmp.setSearchRadius(
      Vec2f(0.5, 0.5));         // Set the valid search region around given path
  dmp.setMap(map_util_, start); // Set map util for collision checking, must be
                                // called before planning

  // Run DMP planner
  timer::Timer time_dist("dmp timer", false);
  dmp.computePath(start, goal, path_jps); // Compute the path given the jps path
  double dt_dist = time_dist.elapsed_us();
  const auto path_dist = dmp.getRawPath();
  printf("DMP Planner takes: %f us\n", dt_dist);
  printf("DMP Path Distance: %f\n", total_distance2f(path_dist));

  write_map(start, goal, path_dist);

  RCLCPP_INFO(get_logger(), "DONE!");
}

void JpsPlanner2D::set_test_map() {

  Vec2f origin = {0, 0.};
  Vec2i dim = {199, 99};
  double res = 0.1;
  std::vector<int8_t> data; // occupancy data, the subscript follows: id = x +
                            // dim.x * y + dim.x * dim.y * z;
  data.resize(dim[0] * dim[1], 0); // fill with 0 for free cells
                                   // occupied: 100, free: 0, unknown: -1
                                   //
                                   // Add the first block
  for (int x = dim[0] / 2 + 1; x < dim[0]; x++) {
    for (int y = dim[1] / 2 + 1; y < dim[1]; y++) {
      int id = x + dim[0] * y;
      data[id] = 100;
    }
  }

  // Add the second block
  for (int x = 2 / res; x < 14 / res; x++) {
    for (int y = 3 / res; y < 5 / res; y++) {
      int id = x + dim[0] * y;
      data[id] = 100;
    }
  }

  map_util_->setMap(origin, dim, data, res);
}

void JpsPlanner2D::set_map_from_file() {
  // try loading the map
  MapReader<Vec2i, Vec2f> reader(map_filepath_,
                                 true); // Map read from a given file
  if (!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET,
           map_filepath_.c_str());
    rclcpp::shutdown();
    return;
  }
  map_util_->setMap(reader.origin(), reader.dim(), reader.data(),
                    reader.resolution());
}

void JpsPlanner2D::write_map() {
#ifndef WRITE_MAP
  return;
#else
  // Plot the result in svg image
  typedef boost::geometry::model::d2::point_xy<double> point_2d;
  // Declare a stream and an SVG mapper
  std::ofstream svg("output.svg");
  boost::geometry::svg_mapper<point_2d> mapper(svg, 1000, 1000);

  // Draw the canvas
  boost::geometry::model::polygon<point_2d> bound;
  const Vec2i dim = map_util_->getDim();
  const Vec2f ori = map_util_->getOrigin();
  const double res = map_util_->getRes();

  const double origin_x = ori(0);
  const double origin_y = ori(1);
  const double range_x = dim(0) * res;
  const double range_y = dim(1) * res;
  std::vector<point_2d> points;
  points.push_back(point_2d(origin_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y + range_y));
  points.push_back(point_2d(origin_x + range_x, origin_y + range_y));
  points.push_back(point_2d(origin_x + range_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y));
  boost::geometry::assign_points(bound, points);
  boost::geometry::correct(bound);

  mapper.add(bound);
  mapper.map(bound, "fill-opacity:1.0;fill:rgb(255,255,255);stroke:rgb(0,0,0);"
                    "stroke-width:2"); // White

  // Draw the obstacles
  // const auto data = dmp.getMapUtil()->getMap();
  const auto data = map_util_->getMap();
  for (int x = 0; x < dim(0); x++) {
    for (int y = 0; y < dim(1); y++) {
      auto value = data[map_util_->getIndex(Vec2i(x, y))];
      if (value > 0) {
        Vec2f pt = map_util_->intToFloat(Vec2i(x, y));
        decimal_t occ = (decimal_t)value / 100;
        point_2d a;
        boost::geometry::assign_values(a, pt(0), pt(1));
        mapper.add(a);
        if (occ < 1)
          mapper.map(a,
                     "fill-opacity:" + std::to_string(occ / 2.0) +
                         ";fill:rgb(118,215,234);",
                     1);
        else
          mapper.map(a, "fill-opacity:1.0;fill:rgb(0,0,0);", 1);
      }
    }
  }

  // Write title at the lower right corner on canvas
  mapper.text(point_2d(origin_x + range_x - 11, origin_y + 2.4),
              "test_distance_map_planner_2d",
              "fill-opacity:1.0;fill:rgb(10,10,250);");

  mapper.text(point_2d(origin_x + range_x - 13, origin_y + 1.8),
              "Green: ", "fill-opacity:1.0;fill:rgb(100,200,100);");
  mapper.text(point_2d(origin_x + range_x - 10.5, origin_y + 1.8),
              "search region", "fill-opacity:1.0;fill:rgb(0,0,0);");

  mapper.text(point_2d(origin_x + range_x - 13, origin_y + 1.2),
              "Red: ", "fill-opacity:1.0;fill:rgb(212,0,0);");
  mapper.text(point_2d(origin_x + range_x - 10.5, origin_y + 1.2),
              "original path", "fill-opacity:1.0;fill:rgb(0,0,0);");

  mapper.text(point_2d(origin_x + range_x - 13, origin_y + 0.6),
              "Blue:", "fill-opacity:1.0;fill:rgb(10,10,250);");
  mapper.text(point_2d(origin_x + range_x - 10.5, origin_y + 0.6),
              "perturbed path", "fill-opacity:1.0;fill:rgb(0,0,0);");

#endif // WRITE_MAP
}

void JpsPlanner2D::write_map(Vec2f start, Vec2f goal) {
#ifndef WRITE_MAP
  (void) start;
  (void) goal;
  return;
#else

  // Plot the result in svg image
  typedef boost::geometry::model::d2::point_xy<double> point_2d;
  // Declare a stream and an SVG mapper
  std::ofstream svg("output.svg");
  boost::geometry::svg_mapper<point_2d> mapper(svg, 1000, 1000);

  // Draw the canvas
  boost::geometry::model::polygon<point_2d> bound;
  const Vec2i dim = map_util_->getDim();
  const Vec2f ori = map_util_->getOrigin();
  const double res = map_util_->getRes();

  const double origin_x = ori(0);
  const double origin_y = ori(1);
  const double range_x = dim(0) * res;
  const double range_y = dim(1) * res;
  std::vector<point_2d> points;
  points.push_back(point_2d(origin_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y + range_y));
  points.push_back(point_2d(origin_x + range_x, origin_y + range_y));
  points.push_back(point_2d(origin_x + range_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y));
  boost::geometry::assign_points(bound, points);
  boost::geometry::correct(bound);

  mapper.add(bound);
  mapper.map(bound, "fill-opacity:1.0;fill:rgb(255,255,255);stroke:rgb(0,0,0);"
                    "stroke-width:2"); // White

  // Draw start and goal
  point_2d start_pt, goal_pt;
  boost::geometry::assign_values(start_pt, start(0), start(1));
  mapper.add(start_pt);
  mapper.map(start_pt, "fill-opacity:1.0;fill:rgb(255,0,0);", 10); // Red
  boost::geometry::assign_values(goal_pt, goal(0), goal(1));
  mapper.add(goal_pt);
  mapper.map(goal_pt, "fill-opacity:1.0;fill:rgb(0,255,0);", 10); // Blue

  // Draw the obstacles
  // const auto data = dmp.getMapUtil()->getMap();
  const auto data = map_util_->getMap();
  for (int x = 0; x < dim(0); x++) {
    for (int y = 0; y < dim(1); y++) {
      auto value = data[map_util_->getIndex(Vec2i(x, y))];
      if (value > 0) {
        Vec2f pt = map_util_->intToFloat(Vec2i(x, y));
        decimal_t occ = (decimal_t)value / 100;
        point_2d a;
        boost::geometry::assign_values(a, pt(0), pt(1));
        mapper.add(a);
        if (occ < 1)
          mapper.map(a,
                     "fill-opacity:" + std::to_string(occ / 2.0) +
                         ";fill:rgb(118,215,234);",
                     1);
        else
          mapper.map(a, "fill-opacity:1.0;fill:rgb(0,0,0);", 1);
      }
    }
  }

  // Write title at the lower right corner on canvas
  mapper.text(point_2d(origin_x + range_x - 11, origin_y + 2.4),
              "test_distance_map_planner_2d",
              "fill-opacity:1.0;fill:rgb(10,10,250);");

  mapper.text(point_2d(origin_x + range_x - 13, origin_y + 1.8),
              "Green: ", "fill-opacity:1.0;fill:rgb(100,200,100);");
  mapper.text(point_2d(origin_x + range_x - 10.5, origin_y + 1.8),
              "search region", "fill-opacity:1.0;fill:rgb(0,0,0);");

  mapper.text(point_2d(origin_x + range_x - 13, origin_y + 1.2),
              "Red: ", "fill-opacity:1.0;fill:rgb(212,0,0);");
  mapper.text(point_2d(origin_x + range_x - 10.5, origin_y + 1.2),
              "original path", "fill-opacity:1.0;fill:rgb(0,0,0);");

  mapper.text(point_2d(origin_x + range_x - 13, origin_y + 0.6),
              "Blue:", "fill-opacity:1.0;fill:rgb(10,10,250);");
  mapper.text(point_2d(origin_x + range_x - 10.5, origin_y + 0.6),
              "perturbed path", "fill-opacity:1.0;fill:rgb(0,0,0);");
#endif // WRITE_MAP
}

void JpsPlanner2D::write_map(Vec2f start, Vec2f goal, vec_Vec2f path) {
#ifndef WRITE_MAP
  (void) start;
  (void) goal;
  (void) path;
  return;
#else

  // Plot the result in svg image
  typedef boost::geometry::model::d2::point_xy<double> point_2d;
  // Declare a stream and an SVG mapper
  std::ofstream svg("output.svg");
  boost::geometry::svg_mapper<point_2d> mapper(svg, 1000, 1000);

  // Draw the canvas
  boost::geometry::model::polygon<point_2d> bound;
  const Vec2i dim = map_util_->getDim();
  const Vec2f ori = map_util_->getOrigin();
  const double res = map_util_->getRes();

  const double origin_x = ori(0);
  const double origin_y = ori(1);
  const double range_x = dim(0) * res;
  const double range_y = dim(1) * res;
  std::vector<point_2d> points;
  points.push_back(point_2d(origin_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y + range_y));
  points.push_back(point_2d(origin_x + range_x, origin_y + range_y));
  points.push_back(point_2d(origin_x + range_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y));
  boost::geometry::assign_points(bound, points);
  boost::geometry::correct(bound);

  mapper.add(bound);
  mapper.map(bound, "fill-opacity:1.0;fill:rgb(255,255,255);stroke:rgb(0,0,0);"
                    "stroke-width:2"); // White

  // Draw start and goal
  point_2d start_pt, goal_pt;
  boost::geometry::assign_values(start_pt, start(0), start(1));
  mapper.add(start_pt);
  mapper.map(start_pt, "fill-opacity:1.0;fill:rgb(255,0,0);", 10); // Red
  boost::geometry::assign_values(goal_pt, goal(0), goal(1));
  mapper.add(goal_pt);
  mapper.map(goal_pt, "fill-opacity:1.0;fill:rgb(0,255,0);", 10); // Green

  // Draw the obstacles
  // const auto data = dmp.getMapUtil()->getMap();
  const auto data = map_util_->getMap();
  for (int x = 0; x < dim(0); x++) {
    for (int y = 0; y < dim(1); y++) {
      auto value = data[map_util_->getIndex(Vec2i(x, y))];
      if (value > 0) {
        Vec2f pt = map_util_->intToFloat(Vec2i(x, y));
        decimal_t occ = (decimal_t)value / 100;
        point_2d a;
        boost::geometry::assign_values(a, pt(0), pt(1));
        mapper.add(a);
        if (occ < 1)
          mapper.map(a,
                     "fill-opacity:" + std::to_string(occ / 2.0) +
                         ";fill:rgb(118,215,234);",
                     1);
        else
          mapper.map(a, "fill-opacity:1.0;fill:rgb(0,0,0);", 1);
      }
    }
  }

  // Draw the path from DM
  boost::geometry::model::linestring<point_2d> line;
  for (auto pt : path)
    line.push_back(point_2d(pt(0), pt(1)));
  mapper.add(line);
  mapper.map(
      line,
      "opacity:0.8;fill:none;stroke:rgb(10,10,250);stroke-width:5"); // Blue

  // Write title at the lower right corner on canvas
  mapper.text(point_2d(origin_x + range_x - 11, origin_y + 2.4),
              "test_distance_map_planner_2d",
              "fill-opacity:1.0;fill:rgb(10,10,250);");

  mapper.text(point_2d(origin_x + range_x - 13, origin_y + 1.8),
              "Green: ", "fill-opacity:1.0;fill:rgb(100,200,100);");
  mapper.text(point_2d(origin_x + range_x - 10.5, origin_y + 1.8),
              "search region", "fill-opacity:1.0;fill:rgb(0,0,0);");

  mapper.text(point_2d(origin_x + range_x - 13, origin_y + 1.2),
              "Red: ", "fill-opacity:1.0;fill:rgb(212,0,0);");
  mapper.text(point_2d(origin_x + range_x - 10.5, origin_y + 1.2),
              "original path", "fill-opacity:1.0;fill:rgb(0,0,0);");

  mapper.text(point_2d(origin_x + range_x - 13, origin_y + 0.6),
              "Blue:", "fill-opacity:1.0;fill:rgb(10,10,250);");
  mapper.text(point_2d(origin_x + range_x - 10.5, origin_y + 0.6),
              "perturbed path", "fill-opacity:1.0;fill:rgb(0,0,0);");
#endif // WRITE_MAP
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JpsPlanner2D>());
  rclcpp::shutdown();
  return 0;
}
