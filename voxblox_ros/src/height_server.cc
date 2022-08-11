#include "voxblox_ros/height_server.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox {

HeightServer::HeightServer(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      world_frame_("map"),
      publish_pointclouds_on_update_(false)
{

  // TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(nh_private);
  // height_layer_.reset(
  //   new Layer<HeightVoxel>(tsdf_config.tsdf_voxel_size,
  //                          tsdf_config.tsdf_voxels_per_side));
  double voxel_size = 0.1;
  int voxels_per_side = 16;
  std::string input_pointcloud_topic_name = "/voxblox_node/local_height_pointcloud";
  nh_private_.param("height_voxel_size", voxel_size, voxel_size);
  nh_private_.param("height_voxels_per_side", voxels_per_side, voxels_per_side);
  nh_private_.param("input_pointcloud_topic_name", input_pointcloud_topic_name, input_pointcloud_topic_name);

  nh_private_.param("publish_pointclouds_on_update", publish_pointclouds_on_update_, publish_pointclouds_on_update_);



  height_layer_.reset(
  new Layer<HeightVoxel>(voxel_size,
                         voxels_per_side));
  
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
  
  height_sub_ = nh_private_.subscribe(input_pointcloud_topic_name,
                                      1,
                                      &HeightServer::heightCallback,
                                      this);

  double publish_poinclouds_every_n_sec = 3;
  nh_private_.param("publish_poinclouds_every_n_sec", publish_poinclouds_every_n_sec,
                    publish_poinclouds_every_n_sec);

  if (publish_poinclouds_every_n_sec > 0.0) {
    publish_pointclouds_timer_ =
        nh_private_.createTimer(ros::Duration(publish_poinclouds_every_n_sec),
                                &HeightServer::publishPointcloudsEvent, this);
  }

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

void HeightServer::publishPointclouds() {
  publishHeightPointcloud();
  publishHeightPointcloudPlane();
}

void HeightServer::publishPointcloudsEvent(const ros::TimerEvent& /*event*/) {
  publishPointclouds();
}

void HeightServer::heightCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {

  // convert the msg to pcl and then to a voxblox pointcloud and traversability vector
  Pointcloud height_pointcloud;

  pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;

  pcl::fromROSMsg(*pointcloud_msg,
                  pointcloud_pcl);

  convertPointcloud(pointcloud_pcl, &height_pointcloud);

  // integrate the height here
  integrateHeight(height_pointcloud);

  publishHeightLayer();
  if(publish_pointclouds_on_update_)
  {
    publishPointclouds();
  }

}



}  // namespace voxblox
