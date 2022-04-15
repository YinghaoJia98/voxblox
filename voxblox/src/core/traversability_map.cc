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

}  // namespace voxblox
