#include "voxblox/integrator/traversability_tsdf_integrator.h"

namespace voxblox {

TraversabilityTsdfIntegrator::TraversabilityTsdfIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                                           Layer<TraversabilityVoxel>* traversability_layer,
                                                           Layer<HeightVoxel>* height_layer)
  : tsdf_layer_(tsdf_layer),
    traversability_layer_(traversability_layer),
    height_layer_(height_layer) {}

void TraversabilityTsdfIntegrator::integrateTraversability(const Pointcloud &pointcloud,
                                                           const Traversabilities& traversabilities) {
  for (unsigned int i = 0; i < pointcloud.size(); ++i) {
    BlockIndex  block_index = traversability_layer_->computeBlockIndexFromCoordinates(pointcloud[i]);

    // If there's no block at the new point yet, storage is allocated
    if(!traversability_layer_->hasBlock(block_index)) {
      traversability_layer_->allocateBlockPtrByCoordinates(pointcloud[i]);
    }

    TraversabilityVoxel* traversability_voxel_ptr = traversability_layer_->getVoxelPtrByCoordinates(pointcloud[i]);

    const float trav = traversability_voxel_ptr->traversability;
    const unsigned int wgt = traversability_voxel_ptr->n_values;

    traversability_voxel_ptr->traversability = (trav * wgt + traversabilities[i]) / (1.0 + wgt);
    traversability_voxel_ptr->n_values += 1;
  }
}

void TraversabilityTsdfIntegrator::integrateHeight(const Pointcloud &pointcloud) {
  std::cout << "TraversabilityTsdfIntegrator::start integrateHeight with " << pointcloud.size() << " points" << std::endl;
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

  std::cout << "TraversabilityTsdfIntegrator::end integrate height poincloud in " << duration__ << " s" << std::endl;
   
}

}  // namespace voxblox
