#include "voxblox_ros/traversability_tsdf_server.h"

namespace voxblox {

TraversabilityTsdfServer::TraversabilityTsdfServer(const ros::NodeHandle& nh,
                                                   const ros::NodeHandle& nh_private)
    : TsdfServer(nh, nh_private) {

  traversability_layer_.reset(
      new Layer<TraversabilityVoxel>(tsdf_map_->getTsdfLayer().voxel_size(),
                                        tsdf_map_->getTsdfLayer().voxels_per_side()));

  traversability_tsdf_integrator_.reset(
      new TraversabilityTsdfIntegrator(tsdf_map_->getTsdfLayer(),
                                          traversability_layer_.get()));

  traversability_pub_ = nh_private_.advertise<voxblox_msgs::Layer> ("traversability_layer",
                                                                    1,
                                                                    true);

  //TODO(Zoe): do remapping for the topic instead of hard coding the path
  // name this traversability_pointcloud instead of traversability
  traversability_pointcloud_sub_ = nh_private_.subscribe("/traversability_node/traversability",
                                                         1,
                                                         &TraversabilityTsdfServer::traversabilityPointcloudCallback,
                                                         this);
}

void TraversabilityTsdfServer::integrateTraversabilityPointcloud(const Pointcloud& traversability_pointcloud,
                                                                 const Traversabilities& traversabilities) {
  traversability_tsdf_integrator_->integrateTraversability(traversability_pointcloud,
                                                            traversabilities);
}

void TraversabilityTsdfServer::publishTraversabilityLayer() {
  voxblox_msgs::Layer layer_msg;

  serializeLayerAsMsg<TraversabilityVoxel>(*traversability_layer_,
                                           true,
                                           &layer_msg);

  traversability_pub_.publish(layer_msg);
}

void TraversabilityTsdfServer::traversabilityPointcloudCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {

  // convert the msg to pcl and then to a voxblox pointcloud and traversability vector
  Pointcloud traversability_pointcloud;
  Traversabilities traversabilities;

  pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;

  pcl::fromROSMsg(*pointcloud_msg,
                  pointcloud_pcl);

  convertTraversabilityPointcloud(pointcloud_pcl,
                                  &traversability_pointcloud,
                                  &traversabilities);

  // integrate the traversability pointcloud to a traversability layer
  integrateTraversabilityPointcloud(traversability_pointcloud,
                                    traversabilities);

  // publish the traversability layer
  publishTraversabilityLayer();
}

}  // namespace voxblox
