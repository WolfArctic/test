#include "astar_search.h"

AstarSearch::AstarSearch()
  : node_initialized_(false)
{
  ros::NodeHandle private_nh_("~");
  ros::NodeHandle nh_;
  nh_.param<int>("angle_size", angle_size_, 40);
  nh_.param<double>("map_resolution", map_resolution, 0.2);
  nh_.param<double>("map_resolution", resolution, 0.2);
  nh_.param<double>("minimum_turning_radius", minimum_turning_radius_, 2.0);
  nh_.param<double>("goal_radius", goal_radius_, 0.15);
  nh_.param<double>("goal_angle", goal_angle_, 6.0);
  nh_.param<bool>("use_back", use_back_, true);
  nh_.param<double>("robot_length", robot_length_, 1.8);
  nh_.param<double>("robot_width", robot_width_, 1.8);
  nh_.param<double>("base2back", base2back_, 0.5);
  nh_.param<double>("curve_weight", curve_weight_, 1.05);
  nh_.param<double>("reverse_weight", reverse_weight_, 2.00);
  nh_.param<bool>("use_wavefront_heuristic", use_wavefront_heuristic_, true);

  createStateUpdateTable(angle_size_);
}

AstarSearch::~AstarSearch()
{
}

void AstarSearch::resizeNode(int width, int height, int angle_size)
{
  nodes_.resize(height);

  for (int i = 0; i < height; i++)
    nodes_[i].resize(width);

  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++)
      nodes_[i][j].resize(angle_size);
}

void AstarSearch::poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y, int *index_theta)
{
  *index_x = (pose.position.x)/ resolution;
  *index_y = (pose.position.y) / resolution;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose.orientation, quat);
  double yaw = tf::getYaw(quat);
  if (yaw < 0)
    yaw += 2 * M_PI;

  // Descretize angle
  static double one_angle_range = 2 * M_PI / angle_size_;
  *index_theta = yaw / one_angle_range;
  *index_theta %= angle_size_;
}


void AstarSearch::createStateUpdateTable(int angle_size)
{
  // Vehicle moving for each angle
  state_update_table_.resize(angle_size);

  // 6 is the number of steering actions
  // max-left, no-turn, and max-right of forward and backward
  if (use_back_) {
    for (int i = 0; i < angle_size; i++)
      state_update_table_[i].resize(6);
  } else {
    for (int i = 0; i < angle_size; i++)
      state_update_table_[i].resize(3);
  }

  // Minimum moving distance with one state update
  //     arc  = r                       * theta
  double step = minimum_turning_radius_ * (2.0 * M_PI / angle_size_);

  for (int i = 0; i < angle_size; i++) {
    double descretized_angle = 2.0 * M_PI / angle_size;
    double robot_angle = descretized_angle * i;

    // Calculate right and left circle
    // Robot moves along these circles
    double right_circle_center_x = minimum_turning_radius_ * std::sin(robot_angle);
    double right_circle_center_y = minimum_turning_radius_ * std::cos(robot_angle) * -1.0;
    double left_circle_center_x  = right_circle_center_x * -1.0;
    double left_circle_center_y  = right_circle_center_y * -1.0;

    // Calculate x and y shift to next state
    NodeUpdate nu;
    // forward
    nu.shift_x     = step * std::cos(robot_angle);
    nu.shift_y     = step * std::sin(robot_angle);
    nu.rotation    = 0;
    nu.index_theta = 0;
    nu.step        = step;
    nu.curve       = false;
    nu.back        = false;
    state_update_table_[i][0] = nu;

    // forward right
    nu.shift_x     = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + robot_angle - descretized_angle);
    nu.shift_y     = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + robot_angle - descretized_angle);
    nu.rotation    = descretized_angle * -1.0;
    nu.index_theta = -1;
    nu.step        = step;
    nu.curve       = true;
    nu.back        = false;
    state_update_table_[i][1] = nu;

    // forward left
    nu.shift_x     = left_circle_center_x + minimum_turning_radius_ * std::cos(-1.0 * M_PI_2 + robot_angle + descretized_angle);
    nu.shift_y     = left_circle_center_y + minimum_turning_radius_ * std::sin(-1.0 * M_PI_2 + robot_angle + descretized_angle);
    nu.rotation    = descretized_angle;
    nu.index_theta = 1;
    nu.step        = step;
    nu.curve       = true;
    nu.back        = false;
    state_update_table_[i][2] = nu;

    // We don't use back move
    if (!use_back_)
      continue;

    // backward
    nu.shift_x     = step * std::cos(robot_angle) * -1.0;
    nu.shift_y     = step * std::sin(robot_angle) * -1.0;
    nu.rotation    = 0;
    nu.index_theta = 0;
    nu.step        = step;
    nu.curve       = false;
    nu.back        = true;
    state_update_table_[i][3] = nu;

    // backward right
    nu.shift_x     = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + robot_angle + descretized_angle);
    nu.shift_y     = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + robot_angle + descretized_angle);
    nu.rotation    = descretized_angle;
    nu.index_theta = 1;
    nu.step        = step;
    nu.curve       = true;
    nu.back        = true;
    state_update_table_[i][4] = nu;

    // backward left
    nu.shift_x     = left_circle_center_x + minimum_turning_radius_ * std::cos(-1.0 * M_PI_2 + robot_angle - descretized_angle);
    nu.shift_y     = left_circle_center_y + minimum_turning_radius_ * std::sin(-1.0 * M_PI_2 + robot_angle - descretized_angle);
    nu.rotation    = descretized_angle * -1.0;
    nu.index_theta = -1;
    nu.step        = step;
    nu.curve       = true;
    nu.back        = true;
    state_update_table_[i][5] = nu;
  }

}

bool AstarSearch::isOutOfRange(int index_x, int index_y)
{
  if (index_x < 0 || index_x >= static_cast<int>(map_width) || index_y < 0 || index_y >= static_cast<int>(map_height))
    return true;

  return false;
}

void AstarSearch::setPath(const SimpleNode &goal)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "map";
  path_.header = header;

  // From the goal node to the start node
  AstarNode *node = &nodes_[goal.index_y][goal.index_x][goal.index_theta];

  while (node != NULL) {
    // Set tf pose
    tf::Vector3 origin(node->x - 5.0, node->y - 5.0, 0);
    tf::Pose tf_pose;
    tf_pose.setOrigin(origin);
    tf_pose.setRotation(tf::createQuaternionFromYaw(node->theta));

    // Set path as ros message
    geometry_msgs::PoseStamped ros_pose;
    tf::poseTFToMsg(tf_pose, ros_pose.pose);
    ros_pose.header = header;
    path_.poses.push_back(ros_pose);

    // To the next node
    node = node->parent;
  }

  // Reverse the vector to be start to goal order
  std::reverse(path_.poses.begin(), path_.poses.end());
}

// theta is 0 ~ 2pi
bool AstarSearch::isGoal(double x, double y, double theta)
{
  // To reduce computation time, we use square value
  static const double goal_radius = goal_radius_ * goal_radius_; // meter
  static const double goal_angle  = M_PI * goal_angle_ / 180.0; // degrees -> radian

  // Check the pose of goal
  if (std::pow(goal_pose_local_.pose.position.x - x, 2) + std::pow(goal_pose_local_.pose.position.y - y, 2) < goal_radius) {
    // Get goal yaw in 0 ~ 2pi
    double goal_yaw = astar::modifyTheta(tf::getYaw(goal_pose_local_.pose.orientation));

    // Check the orientation of goal
    if (astar::calcDiffOfRadian(goal_yaw, theta) < goal_angle)
      return true;
  }

  return false;
}

bool AstarSearch::detectCollision(const SimpleNode &sn)
{
  // Define the robot as rectangle
  static double left   = -1.0 * base2back_;
  static double right  = robot_length_ - base2back_;
  static double top    = robot_width_ / 2.0;
  static double bottom = -1.0 * robot_width_ / 2.0;

  // Coordinate of base_link in OccupancyGrid frame
  static double one_angle_range = 2.0 * M_PI / angle_size_;
  double base_x     = sn.index_x * resolution;
  double base_y     = sn.index_y * resolution;
  double base_theta = sn.index_theta * one_angle_range;

  // Calculate cos and sin in advance
  double cos_theta = std::cos(base_theta);
  double sin_theta = std::sin(base_theta);

  // Convert each point to index and check if the node is Obstacle
  for (double x = left; x < right; x += resolution) {
    for (double y = top; y > bottom; y -= resolution) {
      // 2D point rotation
      int index_x = (x * cos_theta - y * sin_theta + base_x) / resolution;
      int index_y = (x * sin_theta + y * cos_theta + base_y) / resolution;

      if (isOutOfRange(index_x, index_y))
        return true;
      if (nodes_[index_y][index_x][0].status == STATUS::OBS)
        return true;
    }
  }

  return false;
}

bool AstarSearch::calcWaveFrontHeuristic(const SimpleNode &sn)
{
  // Set start point for wavefront search
  // This is goal for Astar search
  nodes_[sn.index_y][sn.index_x][0].hc = 0;
  WaveFrontNode wf_node(sn.index_x, sn.index_y, 1e-10);
  std::queue<WaveFrontNode> qu;
  qu.push(wf_node);

  // State update table for wavefront search
  // Nodes are expanded for each neighborhood cells (moore neighborhood)
  static std::vector<WaveFrontNode> updates = {
    astar::getWaveFrontNode( 0,  1, resolution),
    astar::getWaveFrontNode(-1,  0, resolution),
    astar::getWaveFrontNode( 1,  0, resolution),
    astar::getWaveFrontNode( 0, -1, resolution),
    astar::getWaveFrontNode(-1,  1, std::hypot(resolution, resolution)),
    astar::getWaveFrontNode( 1,  1, std::hypot(resolution, resolution)),
    astar::getWaveFrontNode(-1, -1, std::hypot(resolution, resolution)),
    astar::getWaveFrontNode( 1, -1, std::hypot(resolution, resolution)),
  };

  // Get start index
  int start_index_x;
  int start_index_y;
  int start_index_theta;
  poseToIndex(start_pose_local_.pose, &start_index_x, &start_index_y, &start_index_theta);

  // Whether the robot can reach goal
  bool reachable = false;

  // Start wavefront search
  while (!qu.empty()) {
    WaveFrontNode ref = qu.front();
    qu.pop();

    WaveFrontNode next;
    for (const auto &u : updates) {
      next.index_x = ref.index_x + u.index_x;
      next.index_y = ref.index_y + u.index_y;

      // out of range OR already visited OR obstacle node
      if (isOutOfRange(next.index_x, next.index_y) ||
          nodes_[next.index_y][next.index_x][0].hc > 0 ||
          nodes_[next.index_y][next.index_x][0].status == STATUS::OBS)
        continue;

      // Take the size of robot into account
      if (detectCollisionWaveFront(next))
        continue;

      // Check if we can reach from start to goal
      if (next.index_x == start_index_x && next.index_y == start_index_y)
        reachable = true;

      // Set wavefront heuristic cost
      next.hc = ref.hc + u.hc;
      nodes_[next.index_y][next.index_x][0].hc = next.hc;

      qu.push(next);
    }
  }

  // End of search
  return reachable;
}

// Simple collidion detection for wavefront search
bool AstarSearch::detectCollisionWaveFront(const WaveFrontNode &ref)
{
  // Define the robot as square
  static double half = robot_width_ / 2;
  double robot_x = ref.index_x * resolution;
  double robot_y = ref.index_y * resolution;

  for (double y = half; y > -1.0 * half; y -= resolution) {
    for (double x = -1.0 * half; x < half; x += resolution) {
      int index_x = (robot_x + x) / resolution;
      int index_y = (robot_y + y) / resolution;

      if (isOutOfRange(index_x, index_y))
        return true;

      if (nodes_[index_y][index_x][0].status == STATUS::OBS)
        return true;
    }
  }

  return false;
}

void AstarSearch::reset()
{
  // Clear path
  path_.poses.clear();

  // Clear queue
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> empty;
  std::swap(openlist_, empty);
}

void AstarSearch::setMap(const ivpredict::ivmsgpredict &map)
{
  // Initialize node according to map size
  if (!node_initialized_) {
    map_width  = 40.0 / map_resolution;
    map_height = 10.0 / map_resolution;
    resizeNode(map_width, map_height, angle_size_);
    node_initialized_ = true;
  }

  for (auto a : map.objects)
  {

    if (fabs(a.vrel) > 0.20)
    {
        continue;
    }

    for (auto b : a.cell)
    {
        float gicsCellSize = 0.05;
        float x = b.xc * gicsCellSize;
        float y = b.yc * gicsCellSize;
        float dis = std::hypot(x, y);
        if ( dis > 15.0)
        {
            continue;
        }
        int index_x = ((5.0 + x) / map_resolution);
        int index_y = ((5.0 + y) / map_resolution);
        if ((index_x < 0 || index_x >= map_width) ||
            (index_y < 0 || index_y >= map_height))
        {
          continue;
        } 
        nodes_[index_y][index_x][0].status = STATUS::OBS;
    }
  }

  for (size_t i = 0; i < (size_t)map_height; i++) {
    for (size_t j = 0; j < (size_t)map_width; j++) {

      if (STATUS::OBS == nodes_[i][j][0].status)
      {
        continue;
      }
      else
      {
        for (int k = 0; k < angle_size_; k++) {
          //nodes_[i][j][k].gc     = 0;
          nodes_[i][j][k].hc     = 0;
          nodes_[i][j][k].status = STATUS::NONE;
          nodes_[i][j][k].parent = NULL;
        }        
      }
    }
  }
}

bool AstarSearch::setStartNode()
{
  // Get index of start pose
  int index_x;
  int index_y;
  int index_theta;
  poseToIndex(start_pose_local_.pose, &index_x, &index_y, &index_theta);
  SimpleNode start_sn(index_x, index_y, index_theta, 0, 0);

  // Check if start is valid
  if (isOutOfRange(index_x, index_y) || detectCollision(start_sn))
    return false;

  // Set start node
  AstarNode &start_node = nodes_[index_y][index_x][index_theta];
  start_node.x      = start_pose_local_.pose.position.x;
  start_node.y      = start_pose_local_.pose.position.y;
  start_node.theta  = 2.0 * M_PI / angle_size_ * index_theta;
  start_node.gc     = 0;
  start_node.back   = false;
  start_node.status = STATUS::OPEN;
  if (!use_wavefront_heuristic_)
    start_node.hc = astar::calcDistance(start_pose_local_.pose.position.x, start_pose_local_.pose.position.y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y);

  // Push start node to openlist
  start_sn.cost = start_node.gc + start_node.hc;
  openlist_.push(start_sn);
  return true;
}

bool AstarSearch::setGoalNode()
{
  // Get index of goal pose
  int index_x;
  int index_y;
  int index_theta;
  poseToIndex(goal_pose_local_.pose, &index_x, &index_y, &index_theta);

  SimpleNode goal_sn(index_x, index_y, index_theta, 0, 0);

  // Check if goal is valid
  if (isOutOfRange(index_x, index_y) || detectCollision(goal_sn))
    return false;

  // Calculate wavefront heuristic cost
  if (use_wavefront_heuristic_) {
    bool wavefront_result = calcWaveFrontHeuristic(goal_sn);
    if (!wavefront_result) {
      ROS_WARN("Goal is not reachable...");
      return false;
    }
  }

  return true;
}

bool AstarSearch::search()
{
  // -- Start Astar search ----------
  // If the openlist is empty, search failed
  while (!openlist_.empty()) {

    // Terminate the search if the count reaches a certain value
    static int search_count = 0;
    search_count++;
    if (search_count > 300000) {
      ROS_WARN("Exceed time limit");
      search_count = 0;
      return false;
    }

    // Pop minimum cost node from openlist
    SimpleNode sn;
    sn = openlist_.top();
    openlist_.pop();
    nodes_[sn.index_y][sn.index_x][sn.index_theta].status = STATUS::CLOSED;

    // Expand nodes from this node
    AstarNode *current_node = &nodes_[sn.index_y][sn.index_x][sn.index_theta];

    // for each update
    for (const auto &state : state_update_table_[sn.index_theta]) {
      // Next state
      double next_x     = current_node->x + state.shift_x;
      double next_y     = current_node->y + state.shift_y;
      double next_theta = astar::modifyTheta(current_node->theta + state.rotation);
      double move_cost  = state.step;

#if DEBUG
      // Display search process
      geometry_msgs::Pose p;
      p.position.x = next_x;
      p.position.y = next_y;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(next_theta), p.orientation);
      debug_pose_array_.poses.push_back(p);
#endif

      // Increase curve cost
      if (state.curve)
        move_cost *= curve_weight_;

      // Increase reverse cost
      if (current_node->back != state.back)
        move_cost *= reverse_weight_;

      // Calculate index of the next state
      SimpleNode next;
      next.index_x     = next_x / resolution;
      next.index_y     = next_y / resolution;
      next.index_theta = sn.index_theta + state.index_theta;
      // Avoid invalid index
      next.index_theta = (next.index_theta + angle_size_) % angle_size_;

      // Check if the index is valid
      if (isOutOfRange(next.index_x, next.index_y) || detectCollision(next))
        continue;

      AstarNode *next_node = &nodes_[next.index_y][next.index_x][next.index_theta];
      double next_hc       =  nodes_[next.index_y][next.index_x][0].hc;

      // Calculate euclid distance heuristic cost
      if (!use_wavefront_heuristic_)
        next_hc = astar::calcDistance(next_x, next_y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y);

      // GOAL CHECK
      if (isGoal(next_x, next_y, next_theta)) {
        search_count = 0;
        next_node->status = STATUS::OPEN;
        next_node->x      = next_x;
        next_node->y      = next_y;
        next_node->theta  = next_theta;
        next_node->gc     = current_node->gc + move_cost;
        next_node->hc     = next_hc;
        next_node->back   = state.back;
        next_node->parent = current_node;

        setPath(next);
        return true;
      }

      // NONE
      if (next_node->status == STATUS::NONE) {
        next_node->status = STATUS::OPEN;
        next_node->x      = next_x;
        next_node->y      = next_y;
        next_node->theta  = next_theta;
        next_node->gc     = current_node->gc + move_cost;
        next_node->hc     = next_hc;
        next_node->back   = state.back;
        next_node->parent = current_node;

        next.cost = next_node->gc + next_node->hc;
        openlist_.push(next);
        continue;
      }

      // OPEN or CLOSED
      if (next_node->status == STATUS::OPEN || next_node->status == STATUS::CLOSED) {
        if (current_node->gc + move_cost + next_hc < next_node->gc + next_hc) {
          next_node->status = STATUS::OPEN;
          next_node->x      = next_x;
          next_node->y      = next_y;
          next_node->theta  = next_theta;
          next_node->gc     = current_node->gc + move_cost;
          next_node->hc     = next_hc;
          next_node->back   = state.back;
          next_node->parent = current_node;

          next.cost = next_node->gc + next_node->hc;
          openlist_.push(next);
          continue;
        }
      }

      if (search_count == 0)
        break;

    } // state update

  }

  // Failed to find path
  ROS_WARN("Openlist is Empty!");
  return false;
}

bool AstarSearch::makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose, const ivpredict::ivmsgpredict &map)
{
  start_pose_local_.pose = start_pose;
  goal_pose_local_.pose  = goal_pose;

  // std::cout<<start_pose.position<<std::endl;
  // std::cout<<goal_pose.position<<std::endl;

  setMap(map);

  if (!setStartNode()) {
    ROS_WARN("Invalid start pose!");
    return false;
  }

  if (!setGoalNode()) {
    ROS_WARN("Invalid goal pose!");
    return false;
  }

  bool result = search();

  return result;
}

// for debug
void AstarSearch::publishPoseArray(const ros::Publisher &pub, const std::string &frame)
{
  debug_pose_array_.header.frame_id = frame;
  //debug_pose_array_.poses.push_back(origin);
  debug_pose_array_.poses.push_back(start_pose_.pose);
  debug_pose_array_.poses.push_back(goal_pose_.pose);

  pub.publish(debug_pose_array_);

  debug_pose_array_.poses.clear();
}

ivpathplanner::path AstarSearch::pathinterface(ivmap::ivmapmsglocpos ivlocpos, ivpathplanner::path current, 
                                               ivpathplanner::path raw, ivpredict::ivmsgpredict objs)
{
    ivpathplanner::path ivpath;
    if (objs.objects.empty())
    {
      return current;
    }

    geometry_msgs::Pose start_pose_;
    geometry_msgs::Pose goal_pose_;

    std::vector<int> roadSegId;
    float dis = 0.0;
    if (raw.points.size() > 10)
    {
        for (int i = 0; i < raw.points.size() - 2; ++i)
        {
            dis += std::hypot(raw.points.at(i + 1).x - raw.points.at(i).x,
                              raw.points.at(i + 1).y - raw.points.at(i).y);
            if (dis > 4.0)
            {
                roadSegId.push_back(i);
                dis = 0.0;
            }
        }
        roadSegId.push_back(raw.points.size() - 2);

        start_pose_.position.x = current.points.at(0).x + 5.0;
        start_pose_.position.y = current.points.at(0).y + 5.0;
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), start_pose_.orientation);
    }

    for (int j = 2; j < roadSegId.size(); ++j)
    {
        int cc = roadSegId.at(j);
        goal_pose_.position.x = raw.points.at(cc).x + 5.0;
        goal_pose_.position.y = raw.points.at(cc).y + 5.0;
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(raw.points.at(cc).angle * M_PI / 180.0), goal_pose_.orientation);

        auto start = std::chrono::system_clock::now();

        bool result = makePlan(start_pose_, goal_pose_, objs);

        auto end = std::chrono::system_clock::now();

        auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        ROS_INFO("astar msec: %lf", usec / 1000.0);

        if(true == result) {
          ROS_INFO("Find goal!!!");
          ivpath.points.clear();
          for (auto a : path_.poses)
          {
            ivpathplanner::pathpoint temp;
            temp.x = a.pose.position.x;
            temp.y = a.pose.position.y;

            tf::Quaternion quat;
            tf::quaternionMsgToTF(a.pose.orientation, quat);
            temp.angle= tf::getYaw(quat) * 180.0 / M_PI;

            temp.velocity = -88.0;
            ivpath.points.push_back(temp);
          }
          break;
        } else {
          ROS_INFO("Can't find goal...");
        }
    }
    reset();
    return ivpath;
}
