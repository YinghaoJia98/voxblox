#ifndef VOXBLOX_TRAVERSABILITY_TSDF_SERVER_H_
#define VOXBLOX_TRAVERSABILITY_TSDF_SERVER_H_

#include <voxblox/core/voxel.h>
#include <voxblox/integrator/traversability_tsdf_integrator.h>

#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

  class TraversabilityTsdfServer : public TsdfServer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TraversabilityTsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~TraversabilityTsdfServer() {}

    void integrateTraversabilityPointcloud();

    void publishTraversabilityLayer();

    void traversabilityPointcloudCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud);

  protected:
    ros::Subscriber traversability_pointcloud_sub_;

    ros::Publisher traversability_pub_;

    std::shared_ptr<Layer<TraversabilityVoxel>> traversability_layer_;
    std::unique_ptr<TraversabilityTsdfIntegrator> traversability_tsdf_integrator_;

    voxblox::Pointcloud traversability_pointcloud_;
  };

}  // namespace voxblox

#endif  // VOXBLOX_TRAVERSABILITY_TSDF_SERVER_H_
