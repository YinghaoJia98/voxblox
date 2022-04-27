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
  Eigen::Matrix<float, 3, 1> direction = (p2 - p1);
  float min_trav = 100.0;

  // Divide the line in 1000 segments
  float n_segments = 1000.0;
  for (int i = 0; i < n_segments + 1; ++i) {
    Point p_i = p1 + direction * i / n_segments;

    auto voxel_ptr_i = traversability_layer_->getVoxelPtrByCoordinates(p_i);
    if(!voxel_ptr_i) {
      continue;
    }

    min_trav = std::min(min_trav, voxel_ptr_i->traversability);
  }

  if(min_trav == 100.0) return false;

  *min_traversability = min_trav;
  return true;
}

bool TraversabilityMap::getAverageTraversabilityBetweenPoints(const Point &p1,
                                                              const Point &p2,
                                                              float *avg_traversability) const {
  Eigen::Matrix<float, 3, 1> direction = (p2 - p1);
  float avg_trav = 0;
  int voxel_count = 0;

  Point p_prev = p1;

  // Divide the line in 1000 segments
  float n_segments = 1000.0;
  for (int i = 1; i < n_segments + 1; ++i) {
    Point p_i = p1 + direction * i / n_segments;

    auto voxel_ptr_i = traversability_layer_->getVoxelPtrByCoordinates(p_i);
    auto voxel_ptr_prev = traversability_layer_->getVoxelPtrByCoordinates(p_prev);

    p_prev = p_i;

    if (utils::isSameVoxel(voxel_ptr_i, voxel_ptr_prev)) continue;

    if(!voxel_ptr_i) {
      continue;
    }

    avg_trav += voxel_ptr_i->traversability;
    ++voxel_count;
  }

  if(avg_trav == 0.0) return false;

  *avg_traversability = avg_trav / float(voxel_count);
  return true;
}

}  // namespace voxblox
