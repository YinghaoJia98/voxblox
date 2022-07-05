#ifndef VOXBLOX_HEIGHT_INTEGRATOR_H_
#define VOXBLOX_HEIGHT_INTEGRATOR_H_

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

class HeightIntegrator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HeightIntegrator(Layer<HeightVoxel>* height_layer);

    void integrateHeight(const voxblox::Pointcloud &pointcloud);

  private:
    Layer<HeightVoxel>* height_layer_;
    
  };

}  // namespace voxblox

#endif //VOXBLOX_HEIGHT_INTEGRATOR_H_

