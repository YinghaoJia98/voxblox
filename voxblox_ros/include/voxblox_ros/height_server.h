#ifndef VOXBLOX_HEIGHT_SERVER_H_
#define VOXBLOX_HEIGHT_SERVER_H_

#include <voxblox/integrator/height_integrator.h>
#include "voxblox_ros/tsdf_server.h" //to include all headers needed

namespace voxblox {

  class HeightServer{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HeightServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~HeightServer() {}

    void integrateHeight(const Pointcloud& pointcloud);

    
    void publishHeightLayer();

    void publishPointclouds();
    void publishHeightPointcloud();
    void publishHeightPointcloudPlane();

    void publishPointcloudsEvent(const ros::TimerEvent& event);

    void heightCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);

  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    std::string world_frame_;
    
    ros::Subscriber height_sub_;

    std::unique_ptr<HeightIntegrator> height_integrator_;

    std::shared_ptr<Layer<HeightVoxel>> height_layer_;
    ros::Publisher height_layer_pub_;
    ros::Publisher height_pointcloud_pub_;
    ros::Publisher height_pointcloud_plane_pub_;

    ros::Timer publish_pointclouds_timer_;
    bool publish_pointclouds_on_update_;

  };

}  // namespace voxblox

#endif  // VOXBLOX_HEIGHT_SERVER_H_
