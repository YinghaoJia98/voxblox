#include "voxblox/core/traversability_map.h"

namespace voxblox {

std::string TraversabilityMap::Config::print() const {
  std::stringstream ss;
  // clang-format off
  ss << "====================== Traversability Map Config ========================\n";
  ss << " - tsdf_voxel_size:               " << traversability_voxel_size << "\n";
  ss << " - tsdf_voxels_per_side:          " << traversability_voxels_per_side << "\n";
  ss << "==============================================================\n";
  // clang-format on
  return ss.str();
}

bool TraversabilityMap::getMinimumTraversabilityBetweenPoints(const Point &p1,
                                                              const Point &p2,
                                                              float *min_traversability) const {

  float voxel_size_inv = 1.0 / traversability_layer_->voxel_size();

  const Point start_scaled = p1 * voxel_size_inv;
  const Point end_scaled = p2 * voxel_size_inv;

  AlignedVector<GlobalIndex> global_voxel_index;
  castRay(start_scaled, end_scaled, &global_voxel_index);

  float min_trav = 1.0;
  unsigned int count = 0;

  for (const GlobalIndex& global_voxel_idx : global_voxel_index) {
    auto voxel_ptr = traversability_layer_->getVoxelPtrByGlobalIndex(global_voxel_idx);
    if(!voxel_ptr || voxel_ptr->n_values == 0) continue;

    min_trav = std::min(min_trav, voxel_ptr->traversability);
    ++count;
  }

  if(count == 0) return false;

  *min_traversability = min_trav;
  return true;
}

bool TraversabilityMap::getAverageTraversabilityBetweenPoints(const Point &p1,
                                                              const Point &p2,
                                                              float *avg_traversability) const {
  float voxel_size_inv = 1.0 / traversability_layer_->voxel_size();
  
  const Point start_scaled = p1 * voxel_size_inv;
  const Point end_scaled = p2 * voxel_size_inv;

  AlignedVector<GlobalIndex> global_voxel_index;
  castRay(start_scaled, end_scaled, &global_voxel_index);

  float avg_trav = 0.0;
  unsigned int count = 0;

  for (const GlobalIndex& global_voxel_idx : global_voxel_index) {
    auto voxel_ptr = traversability_layer_->getVoxelPtrByGlobalIndex(global_voxel_idx);
    if(!voxel_ptr || voxel_ptr->n_values == 0) continue;

    avg_trav += voxel_ptr->traversability;
    ++count;
  }

  if(count == 0) return false;

  *avg_traversability = avg_trav / float(count);
  return true;
}

}  // namespace voxblox
