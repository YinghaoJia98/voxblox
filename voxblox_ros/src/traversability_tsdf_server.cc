#include "voxblox_ros/traversability_tsdf_server.h"

namespace voxblox {

TraversabilityTsdfServer::TraversabilityTsdfServer(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : TsdfServer(nh, nh_private) {
  cache_mesh_ = true;

  traversability_layer_.reset(
      new Layer<TraversabilityVoxel>(tsdf_map_->getTsdfLayer().voxel_size(),
                                        tsdf_map_->getTsdfLayer().voxels_per_side()));

  traversability_tsdf_integrator_.reset(new TraversabilityTsdfIntegrator(tsdf_map_->getTsdfLayer(),
                                                                            traversability_layer_.get()));

  pointcloud_sub_ = nh_private_.subscribe("/traversability_node/traversability", 1, &TraversabilityTsdfServer::pointcloudCallback, this);

  traversability_tsdf_integrator_->integrateTraversability(traversability_pointcloud_);

  traversability_pub_ = nh_private_.advertise<voxblox::Layer<TraversabilityVoxel>> ("/traversability_node/traversability_layer", 1, true);
  traversability_pub_.publish(traversability_layer_);
}


void TraversabilityTsdfServer::pointcloudCallback(const voxblox::Pointcloud &pointcloud) {
  CHECK(traversability_layer_);
  CHECK(traversability_tsdf_integrator_);

  traversability_pointcloud_ = pointcloud;
  traversability_tsdf_integrator_->integrateTraversability(pointcloud);
}

}  // namespace voxblox
