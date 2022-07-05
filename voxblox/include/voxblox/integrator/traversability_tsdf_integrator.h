#ifndef VOXBLOX_TRAVERSABILITY_TSDF_INTEGRATOR_H_
#define VOXBLOX_TRAVERSABILITY_TSDF_INTEGRATOR_H_

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

class TraversabilityTsdfIntegrator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TraversabilityTsdfIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                 Layer<TraversabilityVoxel>* traversability_layer,
                                 Layer<HeightVoxel>* height_layer);

    void integrateTraversability(const voxblox::Pointcloud &pointcloud, const Traversabilities& traversabilities);

    void integrateHeight(const voxblox::Pointcloud &pointcloud);

  private:
    const Layer<TsdfVoxel>& tsdf_layer_;
    Layer<TraversabilityVoxel>* traversability_layer_;
    Layer<HeightVoxel>* height_layer_;
    
  };

}  // namespace voxblox

#endif //VOXBLOX_TRAVERSABILITY_TSDF_INTEGRATOR_H_


