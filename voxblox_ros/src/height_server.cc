#include "voxblox_ros/height_server.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox {

HeightServer::HeightServer(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      world_frame_("world")
{

  TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(nh_private);
  height_layer_.reset(
    new Layer<HeightVoxel>(tsdf_config.tsdf_voxel_size,
                           tsdf_config.tsdf_voxels_per_side));
  
  height_integrator_.reset(new HeightIntegrator(height_layer_.get()));

  // Publishers for output.
  
  height_layer_pub_ = nh_private_.advertise<voxblox_msgs::Layer> ("height_layer",
                                                                          1,
                                                                          true);

  height_pointcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI>> ("height_pointcloud",
                                                                                            1,
                                                                                            true);

  height_pointcloud_plane_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI>> ("height_pointcloud_plane",
                                                                                            1,
                                                                                            true);
  
  height_sub_ = nh_private_.subscribe("traversability",
                                              1,
                                              &HeightServer::heightCallback,
                                              this);
}


void HeightServer::integrateHeight(const Pointcloud& pointcloud) {
  height_integrator_->integrateHeight(pointcloud);
}

void HeightServer::publishHeightLayer() {
  voxblox_msgs::Layer layer_msg;

  serializeLayerAsMsg<HeightVoxel>(*height_layer_,
                                   false,
                                   &layer_msg);

  height_layer_pub_.publish(layer_msg);
}


void HeightServer::publishHeightPointcloud() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createHeightPointcloudFromHeightLayer(*height_layer_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;

  height_pointcloud_pub_.publish(pointcloud);
}

void HeightServer::publishHeightPointcloudPlane() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createHeightPointcloudPlaneFromHeightLayer(*height_layer_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;

  height_pointcloud_plane_pub_.publish(pointcloud);
}

void HeightServer::heightCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {

  // convert the msg to pcl and then to a voxblox pointcloud and traversability vector
  Pointcloud traversability_pointcloud;

  pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;

  pcl::fromROSMsg(*pointcloud_msg,
                  pointcloud_pcl);


//   convertTraversabilityPointcloud(pointcloud_pcl,
//                                   &traversability_pointcloud,
//                                   &traversabilities);

  // integrate the height here
  integrateHeight(traversability_pointcloud);

  publishHeightLayer();
  publishHeightPointcloud();
  publishHeightPointcloudPlane();
}

}  // namespace voxblox
