#include "voxblox_ros/traversability_tsdf_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox");
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::TraversabilityTsdfServer node(nh, nh_private);

  ros::spin();
  return 0;
}
