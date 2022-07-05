#include "voxblox/integrator/height_integrator.h"

namespace voxblox {

HeightIntegrator::HeightIntegrator(Layer<HeightVoxel>* height_layer)
  : height_layer_(height_layer){}


void HeightIntegrator::integrateHeight(const Pointcloud &pointcloud) {
  std::cout << "HeightIntegrator::start integrateHeight with " << pointcloud.size() << " points" << std::endl;
  auto start__ = std::chrono::system_clock::now();
  for (unsigned int i = 0; i < pointcloud.size(); ++i) {
    auto cur_point = pointcloud[i];
    cur_point[2] = 0; // store in the layer where z=0;
    BlockIndex  block_index = height_layer_->computeBlockIndexFromCoordinates(cur_point);

    // If there's no block at the new point yet, storage is allocated
    if(!height_layer_->hasBlock(block_index)) {
      height_layer_->allocateBlockPtrByCoordinates(cur_point);
    }

    HeightVoxel* height_voxel_ptr = height_layer_->getVoxelPtrByCoordinates(cur_point);

    const float height = height_voxel_ptr->height;
    const unsigned int wgt = height_voxel_ptr->n_values;

    height_voxel_ptr->height = (height * wgt + pointcloud[i][2]) / (1.0 + wgt);
    height_voxel_ptr->n_values += 1;
  }
  auto end__ = std::chrono::system_clock::now();
  auto duration__ = std::chrono::duration<double>(end__ - start__).count();

  std::cout << "HeightIntegrator::end integrate height poincloud in " << duration__ << " s" << std::endl;
   
}

}  // namespace voxblox
