#include "map_loader.h"

map_loader::map_loader(ros::NodeHandle n) {
  maps_ = "INVALID";
  map_length = 30;
  map_bit = 4;
  map_number = 7;
  map_divide_flag = false;

  n.param("ROTATE_IMG", rotate_img, rotate_img);
  n.param("MAX_X", max_x, max_x);
  n.param("MAX_Y", max_y, max_y);
  n.param("MIN_X", min_x, min_x);
  n.param("MIN_Y", min_y, min_y);
  n.param("pointmap", pointmap_, pointmap_);
  n.param("maps", maps_, maps_);
  n.param("map_length", map_length, map_length);
  n.param("map_bit", map_bit, map_bit);
  n.param("map_number", map_number, map_number);
  n.param("map_divide_flag", map_divide_flag, map_divide_flag);

  flagX = flagY = 0;
  gps_flag = pos_flag = initial_flag = false;
  cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  last_gps_time = gpsTime = ros::Time::now().toSec();
  pcd_size = findSubFile(maps_);

  maps_pub = n.advertise<sensor_msgs::PointCloud2>("points_map", 10, true);
  sub_pose = n.subscribe("ndt_pose_", 10, &map_loader::callback_pose, this);
  sub_status =
      n.subscribe("ndt_status", 10, &map_loader::callback_status, this);
  sub_gps = n.subscribe("ivlocposfusion", 10, &map_loader::callback_gps, this);

  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pointmap_.c_str(), *cloud) == -1) {
    PCL_ERROR("Could not read the file\n ");
  }
  if (!cloud->points.size()) PCL_ERROR("The cloud is empty!\n");
  if (map_length <= 0) map_length = 30;
}

int map_loader::findSubFile(std::string path) {
  if (path == "INVALID") return 0;
  DIR* dir;
  struct dirent* pdir;
  struct stat st;
  dir = opendir(path.c_str());
  if (NULL == dir) return 0;

  while (pdir = readdir(dir)) {
    if (atoi(pdir->d_name) > 0 && atoi(pdir->d_name) < 10000) {
      // remove the backup file, for example: 4~
      bool flag = false;
      for (int i = 0; i < files.size(); i++) {
        if (atoi(pdir->d_name) == atoi(files[i].c_str())) {
          flag = true;
        }
      }
      if (false == flag) {
        files.push_back(pdir->d_name);
      }
    }
  }
  closedir(dir);
  return files.size();
}

void map_loader::mergeCloud(int x, int y, int row, int col) {
  int k = col * y + x;
  if (k < 0) return;
  if (!checkPCD(k)) {
    std::string test = maps_;
    std::stringstream ss;
    ss << setw(map_bit) << setfill('0') << std::right << k;
    test += ss.str() + ".pcd";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudTemp(
        new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile(test.c_str(), *cloudTemp) == -1) {
      PCL_ERROR("Could not read the file\n ");
      return;
    }
    UnorderedPCD_.emplace(k, *cloudTemp);
  }
}

bool map_loader::checkPCD(int input) {
  UnorderedPCD::iterator it = UnorderedPCD_.find(input);
  if (it == UnorderedPCD_.end())
    return false;
  else
    return true;
}

bool map_loader::checkIndex(int input, int center_x, int center_y, int row,
                            int col) {
  int temp_size = map_number / 2;
  for (int i = -temp_size; i < (temp_size + 1); ++i) {
    for (int j = -temp_size; j < (temp_size + 1); ++j) {
      int temp_id = col * (center_y - 1 + i) + center_x + j;
      if (temp_id == input) return true;
    }
  }
  return false;
}

void map_loader::publish_adjacent_pcd(int numX, int numY) {
  int col = (int)(max_x - min_x) / map_length + 1;
  int row = (int)(max_y - min_y) / map_length + 1;
  pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  if (!(flagX == numX && flagY == numY)) {
    flagX = numX;
    flagY = numY;
    double load_start = ros::Time::now().toSec();
    int temp_size = map_number / 2;
    std::vector<int> index_to_remove;

    // remove old maps
    for (auto& temp_index : UnorderedPCD_) {
      if (!checkIndex(temp_index.first, flagX, flagY, row, col)) {
        index_to_remove.push_back(temp_index.first);
      }
    }
    for (auto& temp_index : index_to_remove) UnorderedPCD_.erase(temp_index);

    // insert new maps
    for (int i = -temp_size; i < (temp_size + 1); ++i) {
      for (int j = -temp_size; j < (temp_size + 1); ++j) {
        int tempX = numX + j;
        int tempY = numY + i;
        if (tempX > 0 && tempX <= col && tempY > 0 && tempY <= row)
          mergeCloud(tempX, tempY, row, col);
      }
    }
    for (auto& temp_map : UnorderedPCD_) {
      outCloud->insert(outCloud->end(), temp_map.second.begin(),
                       temp_map.second.end());
    }

    // publish maps
    pcl::toROSMsg(*outCloud, outPointCloud);
    outPointCloud.header.frame_id = std::string("map");
    clock_t end = clock();
    ROS_INFO_STREAM("The part map Has been published!!!");
    maps_pub.publish(outPointCloud);
    double load_end = ros::Time::now().toSec();
    // ROS_INFO("The MAP LOADED WITHIN %f (s)", (load_end-load_start));
    last_gps_time = ros::Time::now().toSec();
  }
}

void map_loader::publish_pcd(ivlocmsg::ivmsglocpos loc_input) {
  if (map_divide_flag && pcd_size > 0 && !pos_flag && loc_input.lon > 100 &&
      loc_input.lat > 30 &&
      fabs(ros::Time::now().toSec() - last_gps_time) > 1) {
    int numX = (int)(loc_input.xg) / map_length;
    int numY = (int)(loc_input.yg) / map_length;
    publish_adjacent_pcd(numX, numY);
  } else if (!map_divide_flag && !pos_flag &&
             fabs(ros::Time::now().toSec() - last_gps_time) > 3) {
    pcl::toROSMsg(*cloud, outPointCloud);
    outPointCloud.header.frame_id = std::string("map");
    ROS_INFO_STREAM("The overall map Has been published!!!");
    maps_pub.publish(outPointCloud);
    last_gps_time = ros::Time::now().toSec();
    pos_flag = true;
  } else{
    return;
  }
}

void map_loader::callback_gps(const ivlocmsg::ivmsglocpos::ConstPtr& msg) {
  get_gps_pos = *msg;
  gps_flag = true;
  gpsTime = ros::Time::now().toSec();
  publish_pcd(get_gps_pos);
}

void map_loader::callback_status(const ivlocmsg::ndt_status::ConstPtr& msg) {
  status_ = *msg;
  if (status_.score > 0 && status_.score < 1) initial_flag = true;
}

void map_loader::callback_pose(const geometry_msgs::PoseStamped msg) {
  if (initial_flag) {
    pos_flag = true;
    int numX = (int)(msg.pose.position.x - min_x) / map_length + 1;
    int numY = (int)(msg.pose.position.y - min_y) / map_length + 1;
    if (map_divide_flag) publish_adjacent_pcd(numX, numY);
  } else {
    ROS_ERROR("NDT POSE IS ERROR!!!!!!!");
    return;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_loader");
  ros::NodeHandle n;
  ros::Rate loop(1);
  map_loader loading_map(n);

  while (ros::ok()) {
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
