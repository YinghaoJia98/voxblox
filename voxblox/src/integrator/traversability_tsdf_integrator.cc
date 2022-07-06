#include "voxblox/integrator/traversability_tsdf_integrator.h"

namespace voxblox {

TraversabilityTsdfIntegrator::TraversabilityTsdfIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                                           Layer<TraversabilityVoxel>* traversability_layer)
  : tsdf_layer_(tsdf_layer),
    traversability_layer_(traversability_layer) {}

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

}  // namespace voxblox
