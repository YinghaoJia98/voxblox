//
// Created by johannes on 18.12.19.
//
#include <voxblox/interpolator/interpolator.h>

using namespace voxblox;

template <>
bool Interpolator<EsdfCachingVoxel>::getInterpolatedDistanceGradientHessian(
    const Point& pos, FloatingPoint* distance, Point* gradient,
    Eigen::Matrix<FloatingPoint, 3, 3>* hessian) const {
  typename Layer<EsdfCachingVoxel>::BlockType::ConstPtr block_ptr =
      layer_->getBlockPtrByCoordinates(pos);
  if (block_ptr == nullptr) {
    return false;
  }
  VoxelIndex voxel_index =
      block_ptr->computeTruncatedVoxelIndexFromCoordinates(pos);
  Point voxel_pos = block_ptr->computeCoordinatesFromVoxelIndex(voxel_index);

  const EsdfCachingVoxel* voxel_ptr = block_ptr->getVoxelPtrByCoordinates(pos);

  if (voxel_ptr) {
    float_t voxelSizeInv = block_ptr->voxel_size_inv();
    Point offset = (pos - voxel_pos);
//    *distance = voxel_ptr->distance + voxel_ptr->gradient.dot(offset) +
//                0.5 * offset.dot(voxel_ptr->hessian * offset);
    *distance = voxel_ptr->distance + voxel_ptr->gradient.dot(offset);
    //*gradient = voxel_ptr->gradient + voxel_ptr->hessian * offset;
    *gradient = voxel_ptr->gradient;
    //*hessian = voxel_ptr->hessian;

//    std::cout << "voxel_ptr->gradient: " << std::endl << voxel_ptr->gradient << std::endl;
//    std::cout << "voxel_ptr->hessian: " << std::endl << voxel_ptr->hessian << std::endl;
//    std::cout << "offset: " << std::endl << offset << std::endl;
//    std::cout << "voxel_pos: " << std::endl << voxel_pos << std::endl;

    return true;
  }
  return false;
}

template <>
bool Interpolator<EsdfCachingVoxel>::getInterpolatedGradient(
    const Point& pos, Point* grad) const {
  typename Layer<EsdfCachingVoxel>::BlockType::ConstPtr block_ptr =
      layer_->getBlockPtrByCoordinates(pos);
  if (block_ptr == nullptr) {
    return false;
  }
  VoxelIndex voxel_index =
      block_ptr->computeTruncatedVoxelIndexFromCoordinates(pos);
  Point voxel_pos = block_ptr->computeCoordinatesFromVoxelIndex(voxel_index);

  const EsdfCachingVoxel* voxel_ptr = block_ptr->getVoxelPtrByCoordinates(pos);

  if (voxel_ptr) {
    float_t voxelSizeInv = block_ptr->voxel_size_inv();
    Point offset = (pos - voxel_pos);
    *grad = voxel_ptr->gradient + voxel_ptr->hessian * offset;
    return true;
  }
  return false;
}

template <>
bool Interpolator<EsdfCachingVoxel>::getInterpolatedDistance(
    const Point& pos, FloatingPoint* distance) const {
  typename Layer<EsdfCachingVoxel>::BlockType::ConstPtr block_ptr =
      layer_->getBlockPtrByCoordinates(pos);
  if (block_ptr == nullptr) {
    return false;
  }
  VoxelIndex voxel_index =
      block_ptr->computeTruncatedVoxelIndexFromCoordinates(pos);
  Point voxel_pos = block_ptr->computeCoordinatesFromVoxelIndex(voxel_index);

  const EsdfCachingVoxel* voxel_ptr = block_ptr->getVoxelPtrByCoordinates(pos);

  if (voxel_ptr) {
    float_t voxelSizeInv = block_ptr->voxel_size_inv();
    Point offset = (pos - voxel_pos);
    *distance = voxel_ptr->distance + voxel_ptr->gradient.dot(offset) +
                0.5 * offset.dot(voxel_ptr->hessian * offset);
    return true;
  }
  return false;
}