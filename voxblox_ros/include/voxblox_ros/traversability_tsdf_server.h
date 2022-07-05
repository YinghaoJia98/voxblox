#ifndef VOXBLOX_TRAVERSABILITY_TSDF_SERVER_H_
#define VOXBLOX_TRAVERSABILITY_TSDF_SERVER_H_

#include <voxblox/integrator/traversability_tsdf_integrator.h>

#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

  class TraversabilityTsdfServer : public TsdfServer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TraversabilityTsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~TraversabilityTsdfServer() {}

    void integrateTraversability(const Pointcloud& traversability_pointcloud, const Traversabilities& traversabilities);

    void integrateHeight(const Pointcloud& pointcloud);

    void publishTraversabilityLayer();
    void publishPointclouds() override;

    void publishHeightLayer();
    void publishHeightPointcloud();
    void publishHeightPointcloudPlane();

    void traversabilityCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);

  protected:
    ros::Subscriber traversability_sub_;

    ros::Publisher traversability_layer_pub_;
    ros::Publisher traversability_pointcloud_pub_;

    std::shared_ptr<Layer<TraversabilityVoxel>> traversability_layer_;
    std::unique_ptr<TraversabilityTsdfIntegrator> traversability_tsdf_integrator_;

    std::shared_ptr<Layer<HeightVoxel>> height_layer_;
    ros::Publisher height_layer_pub_;
    ros::Publisher height_pointcloud_pub_;
    ros::Publisher height_pointcloud_plane_pub_;


  };

}  // namespace voxblox

#endif  // VOXBLOX_TRAVERSABILITY_TSDF_SERVER_H_
