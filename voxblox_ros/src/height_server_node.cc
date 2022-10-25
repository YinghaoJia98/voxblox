#include "voxblox_ros/height_server.h"
#include <gflags/gflags.h>
int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_height");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::HeightServer node(nh, nh_private);

  ros::spin();
  return 0;
}
