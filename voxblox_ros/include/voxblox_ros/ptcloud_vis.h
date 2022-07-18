#ifndef VOXBLOX_ROS_PTCLOUD_VIS_H_
#define VOXBLOX_ROS_PTCLOUD_VIS_H_

#include <eigen_conversions/eigen_msg.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

#include <algorithm>
#include <string>

#include "voxblox_ros/conversions.h"

#include <chrono> // time how long iterate through all the voxels

/**
 * This file contains a set of functions to visualize layers as pointclouds
 * (or marker arrays) based on a passed-in function. It also offers some
 * specializations of functions as samples.
 */

namespace voxblox {

/** Shortcut the placeholders namespace, since otherwise it chooses the boost
 * placeholders which are in the global namespace (thanks boost!).
 */
namespace ph = std::placeholders;

/// This is a fancy alias to be able to template functions.
template <typename VoxelType>
using ShouldVisualizeVoxelColorFunctionType = std::function<bool(
    const VoxelType& voxel, const Point& coord, Color* color)>;

/**
 * For intensities values, such as distances, which are mapped to a color only
 * by the subscriber.
 */
template <typename VoxelType>
using ShouldVisualizeVoxelIntensityFunctionType = std::function<bool(
    const VoxelType& voxel, const Point& coord, double* intensity)>;

/**
 * For boolean checks -- either a voxel is visualized or it is not.
 * This is used for occupancy bricks, for instance.
 */
template <typename VoxelType>
using ShouldVisualizeVoxelFunctionType =
    std::function<bool(const VoxelType& voxel, const Point& coord)>;  // NOLINT;

/// Template function to visualize a colored pointcloud.
template <typename VoxelType>
void createColorPointcloudFromLayer(
    const Layer<VoxelType>& layer,
    const ShouldVisualizeVoxelColorFunctionType<VoxelType>& vis_function,
    pcl::PointCloud<pcl::PointXYZRGB>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  pointcloud->clear();
  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Temp variables.
  Color color;
  // Iterate over all blocks.
  for (const BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const Block<VoxelType>& block = layer.getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      if (vis_function(block.getVoxelByLinearIndex(linear_index), coord,
                       &color)) {
        pcl::PointXYZRGB point;
        point.x = coord.x();
        point.y = coord.y();
        point.z = coord.z();
        point.r = color.r;
        point.g = color.g;
        point.b = color.b;
        pointcloud->push_back(point);
      }
    }
  }
}

template<typename T>
std::vector<T> arange(T start, T stop, T step = 1) {
    std::vector<T> values;
    for (T value = start; value < stop; value += step)
        values.push_back(value);
    return values;
}

/// Template function to visualize a colored pointcloud.
template <typename VoxelType>
void createColorPointcloudFromLayerBounds(
    const Layer<VoxelType>& layer,
    const ShouldVisualizeVoxelColorFunctionType<VoxelType>& vis_function,
    pcl::PointCloud<pcl::PointXYZRGB>* pointcloud, Transformation* trafo) {
  CHECK_NOTNULL(pointcloud);
  pointcloud->clear();
  
  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;
  
  Color color;
  // Iterate over blocks close to center position.
  // tsdf_voxel_size: 0.1; tsdf_voxels_per_side: 16 -> block = 1.6m (bound -3;+3) -> 5 (6 for rounding)
  
  auto position = trafo->getPosition();
  for (auto x_ : arange<float>( position(0,0)-4, position(0,0)+4, 1.6 )){
    for (auto y_ : arange<float>( position(1,0)-4, position(1,0)+4, 1.6 )){
      for (auto z_ : arange<float>( position(2,0)-2, position(2,0)+2.8, 1.6 )){
        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        Point point_block(x_,y_,z_);
        
        auto block_index = layer.computeBlockIndexFromCoordinates(point_block);
        if (layer.hasBlock(block_index)){
          auto block = layer.getBlockPtrByIndex(block_index);

          for (size_t linear_index = 0; linear_index < num_voxels_per_block;
              ++linear_index) {
            Point coord = block->computeCoordinatesFromLinearIndex(linear_index);
            if (vis_function(block->getVoxelByLinearIndex(linear_index), coord,
                            &color)) {
              pcl::PointXYZRGB point;
              point.x = coord.x();
              point.y = coord.y();
              point.z = coord.z();
              pointcloud->push_back(point);
            }
          }
        }
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        //std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
      }
    }
  }
}


/// Template function to visualize an intensity pointcloud.
template <typename VoxelType>
void createColorPointcloudFromLayer(
    const Layer<VoxelType>& layer,
    const ShouldVisualizeVoxelIntensityFunctionType<VoxelType>& vis_function,
    pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  pointcloud->clear();
  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Temp variables.
  double intensity = 0.0;
  // Iterate over all blocks.
  for (const BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const Block<VoxelType>& block = layer.getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      if (vis_function(block.getVoxelByLinearIndex(linear_index), coord,
                       &intensity)) {
        pcl::PointXYZI point;
        point.x = coord.x();
        point.y = coord.y();
        point.z = coord.z();
        point.intensity = intensity;
        pointcloud->push_back(point);
      }
    }
  }
}

template <typename VoxelType>
void createOccupancyBlocksFromLayer(
    const Layer<VoxelType>& layer,
    const ShouldVisualizeVoxelFunctionType<VoxelType>& vis_function,
    const std::string& frame_id,
    visualization_msgs::MarkerArray* marker_array) {
  CHECK_NOTNULL(marker_array);
  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;
  FloatingPoint voxel_size = layer.voxel_size();

  visualization_msgs::Marker block_marker;
  block_marker.header.frame_id = frame_id;
  block_marker.ns = "occupied_voxels";
  block_marker.id = 0;
  block_marker.type = visualization_msgs::Marker::CUBE_LIST;
  block_marker.scale.x = block_marker.scale.y = block_marker.scale.z =
      voxel_size;
  block_marker.action = visualization_msgs::Marker::ADD;

  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);
  int ii = 0;
  for (const BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const Block<VoxelType>& block = layer.getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      ii++;
      Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      if (vis_function(block.getVoxelByLinearIndex(linear_index), coord)) {
        geometry_msgs::Point cube_center;
        cube_center.x = coord.x();
        cube_center.y = coord.y();
        cube_center.z = coord.z();
        block_marker.points.push_back(cube_center);
        std_msgs::ColorRGBA color_msg;
        colorVoxbloxToMsg(rainbowColorMap((coord.z() + 2.5) / 5.0), &color_msg);
        block_marker.colors.push_back(color_msg);
      }
    }
  }
  std::cout << " iterated through " << ii << " voxels" << std::endl;
  marker_array->markers.push_back(block_marker);
}

// /Short-hand functions for visualizing different types of voxels.
inline bool visualizeNearSurfaceTsdfVoxels(const TsdfVoxel& voxel,
                                           const Point& /*coord*/,
                                           double surface_distance,
                                           Color* color) {
  CHECK_NOTNULL(color);
  constexpr float kMinWeight = 0;
  if (voxel.weight > kMinWeight &&
      std::abs(voxel.distance) < surface_distance) {
    *color = voxel.color;
    return true;
  }
  return false;
}

// /Short-hand functions for visualizing different types of voxels.
inline bool visualizeNearLocalSurfaceTsdfVoxels(const TsdfVoxel& voxel,
                                                const Point& coord,
                                                double surface_distance,
                                                Color* color,
                                                Transformation* trafo) {
  constexpr float kMinWeight = 0;
  return (voxel.weight > kMinWeight &&
      std::abs(voxel.distance) < surface_distance && 
      std::abs(trafo->getPosition()[0] - coord.x()) < 4.0 &&
      std::abs(trafo->getPosition()[1] - coord.y()) < 4.0);
}

inline bool visualizeTsdfVoxels(const TsdfVoxel& voxel, const Point& /*coord*/,
                                Color* color) {
  CHECK_NOTNULL(color);
  constexpr float kMinWeight = 0;
  if (voxel.weight > kMinWeight) {
    *color = voxel.color;
    return true;
  }
  return false;
}

inline bool visualizeDistanceIntensityTsdfVoxels(const TsdfVoxel& voxel,
                                                 const Point& /*coord*/,
                                                 double* intensity) {
  CHECK_NOTNULL(intensity);
  constexpr float kMinWeight = 1e-3;
  if (voxel.weight > kMinWeight) {
    *intensity = voxel.distance;
    return true;
  }
  return false;
}

inline bool visualizeDistanceIntensityTsdfVoxelsNearSurface(
    const TsdfVoxel& voxel, const Point& /*coord*/, double surface_distance,
    double* intensity) {
  CHECK_NOTNULL(intensity);
  constexpr float kMinWeight = 1e-3;
  if (voxel.weight > kMinWeight &&
      std::abs(voxel.distance) < surface_distance) {
    *intensity = voxel.distance;
    return true;
  }
  return false;
}

inline bool visualizeDistanceIntensityTsdfVoxelsSlice(
    const TsdfVoxel& voxel, const Point& coord, unsigned int free_plane_index,
    FloatingPoint free_plane_val, FloatingPoint voxel_size, double* intensity) {
  CHECK_NOTNULL(intensity);
  constexpr float kMinWeight = 1e-3;

  if (std::abs(coord(free_plane_index) - free_plane_val) <=
      (voxel_size / 2.0 + kFloatEpsilon)) {
    if (voxel.weight > kMinWeight) {
      *intensity = voxel.distance;
      return true;
    }
  }
  return false;
}

inline bool visualizeDistanceIntensityEsdfVoxels(const EsdfVoxel& voxel,
                                                 const Point& /*coord*/,
                                                 double* intensity) {
  CHECK_NOTNULL(intensity);
  if (voxel.observed) {
    *intensity = voxel.distance;
    return true;
  }
  return false;
}

inline bool visualizeIntensityVoxels(const IntensityVoxel& voxel,
                                     const Point& /*coord*/,
                                     double* intensity) {
  constexpr float kMinWeight = 1e-3;

  CHECK_NOTNULL(intensity);
  if (voxel.weight > kMinWeight) {
    *intensity = voxel.intensity;
    return true;
  }
  return false;
}

inline bool visualizeTraversabilityVoxels(const TraversabilityVoxel& voxel,
                                     const Point& /*coord*/,
                                     double* traversability) {
  CHECK_NOTNULL(traversability);
  if (voxel.n_values > 0) {
    *traversability = voxel.traversability;
    return true;
  }
  return false;
}

inline bool visualizeHeightVoxels(const HeightVoxel& voxel,
                                     const Point& /*coord*/,
                                     double* height) {
  CHECK_NOTNULL(height);
  if (voxel.n_values > 0) {
    *height = voxel.height;
    return true;
  }
  return false;
}

inline bool visualizeDistanceIntensityEsdfVoxelsSlice(
    const EsdfVoxel& voxel, const Point& coord, unsigned int free_plane_index,
    FloatingPoint free_plane_val, FloatingPoint voxel_size, double* intensity) {
  CHECK_NOTNULL(intensity);

  if (std::abs(coord(free_plane_index) - free_plane_val) <=
      (voxel_size / 2.0 + kFloatEpsilon)) {
    if (voxel.observed) {
      *intensity = voxel.distance;
      return true;
    }
  }
  return false;
}

inline bool visualizeOccupiedTsdfVoxels(const TsdfVoxel& voxel,
                                        const Point& /*coord*/,
                                        const float min_distance = 0.0) {
  constexpr float kMinWeight = 1e-3;
  if (voxel.weight > kMinWeight && voxel.distance <= min_distance) {
    return true;
  }
  return false;
}

inline bool visualizeFreeEsdfVoxels(const EsdfVoxel& voxel,
                                    const Point& /*coord*/, float min_distance,
                                    double* intensity) {
  if (voxel.observed && voxel.distance >= min_distance) {
    *intensity = voxel.distance;
    return true;
  }
  return false;
}

inline bool visualizeOccupiedOccupancyVoxels(const OccupancyVoxel& voxel,
                                             const Point& /*coord*/) {
  const float kThresholdLogOccupancy = logOddsFromProbability(0.7);
  if (voxel.probability_log > kThresholdLogOccupancy) {
    return true;
  }
  return false;
}

// All functions that can be used directly for TSDF voxels.

/**
 * Create a pointcloud based on the TSDF voxels near the surface.
 * The RGB color is determined by the color of the TSDF voxel.
 */
inline void createSurfacePointcloudFromTsdfLayer(
    const Layer<TsdfVoxel>& layer, double surface_distance,
    pcl::PointCloud<pcl::PointXYZRGB>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  createColorPointcloudFromLayer<TsdfVoxel>(
      layer,
      std::bind(&visualizeNearSurfaceTsdfVoxels, ph::_1, ph::_2,
                surface_distance, ph::_3),
      pointcloud);
}

inline void createLocalSurfacePointcloudFromTsdfLayer(
    const Layer<TsdfVoxel>& layer, double surface_distance,
    pcl::PointCloud<pcl::PointXYZRGB>* pointcloud, Transformation& trafo) {
  CHECK_NOTNULL(pointcloud);

  createColorPointcloudFromLayerBounds<TsdfVoxel>(
      layer,
      std::bind(&visualizeNearLocalSurfaceTsdfVoxels, ph::_1, ph::_2,
                surface_distance, ph::_3, &trafo),
      pointcloud, &trafo);
}
inline void createUncoloredSurfacePointcloudFromTsdfLayer(
        const Layer<TsdfVoxel>& layer, double surface_distance,
        pcl::PointCloud<pcl::PointXYZ>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  pointcloud->clear();
  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Iterate over all blocks.
  for (const BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const Block<TsdfVoxel>& block = layer.getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      if (block.getVoxelByLinearIndex(linear_index).weight > 0 &&
          std::abs(block.getVoxelByLinearIndex(linear_index).distance) < surface_distance) {
        pcl::PointXYZ point;
        point.x = coord.x();
        point.y = coord.y();
        point.z = coord.z();
        pointcloud->push_back(point);
      }
    }
  }
}

/**
 * Create a pointcloud based on all the TSDF voxels.
 * The RGB color is determined by the color of the TSDF voxel.
 */
inline void createPointcloudFromTsdfLayer(
    const Layer<TsdfVoxel>& layer,
    pcl::PointCloud<pcl::PointXYZRGB>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  createColorPointcloudFromLayer<TsdfVoxel>(layer, &visualizeTsdfVoxels,
                                            pointcloud);
}

/**
 * Create a pointcloud based on all the TSDF voxels.
 * The intensity is determined based on the distance to the surface.
 */
inline void createDistancePointcloudFromTsdfLayer(
    const Layer<TsdfVoxel>& layer,
    pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  createColorPointcloudFromLayer<TsdfVoxel>(
      layer, &visualizeDistanceIntensityTsdfVoxels, pointcloud);
}

/**
 * Create a pointcloud based on the TSDF voxels near the surface.
 * The intensity is determined based on the distance to the surface.
 */
inline void createSurfaceDistancePointcloudFromTsdfLayer(
    const Layer<TsdfVoxel>& layer, double surface_distance,
    pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  createColorPointcloudFromLayer<TsdfVoxel>(
      layer,
      std::bind(&visualizeDistanceIntensityTsdfVoxelsNearSurface, ph::_1,
                ph::_2, surface_distance, ph::_3),
      pointcloud);
}

inline void createDistancePointcloudFromEsdfLayer(
    const Layer<EsdfVoxel>& layer,
    pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  createColorPointcloudFromLayer<EsdfVoxel>(
      layer, &visualizeDistanceIntensityEsdfVoxels, pointcloud);
}

inline void createFreePointcloudFromEsdfLayer(
    const Layer<EsdfVoxel>& layer, float min_distance,
    pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  createColorPointcloudFromLayer<EsdfVoxel>(
      layer,
      std::bind(&visualizeFreeEsdfVoxels, ph::_1, ph::_2, min_distance, ph::_3),
      pointcloud);
}

inline void createIntensityPointcloudFromIntensityLayer(
    const Layer<IntensityVoxel>& layer,
    pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  createColorPointcloudFromLayer<IntensityVoxel>(
      layer, &visualizeIntensityVoxels, pointcloud);
}

inline void createTraversabilityPointcloudFromTraversabilityLayer(
        const Layer<TraversabilityVoxel>& layer,
        pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  createColorPointcloudFromLayer<TraversabilityVoxel>(
          layer, &visualizeTraversabilityVoxels, pointcloud);
}

// only used for visualization, so speed does not matter too much
inline void createHeightPointcloudFromHeightLayer(
        const Layer<HeightVoxel>& layer,
        pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);

  pointcloud->clear();
  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Temp variables.
  double height = 0.0;
  // Iterate over all blocks.
  for (const BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const Block<HeightVoxel>& block = layer.getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      if (visualizeHeightVoxels(block.getVoxelByLinearIndex(linear_index), coord,
                       &height)) {
        pcl::PointXYZI point;
        point.x = coord.x();
        point.y = coord.y();
        point.z = height;
        point.intensity = height;
        pointcloud->push_back(point);
      }
    }
  }
}

inline void createHeightPointcloudPlaneFromHeightLayer(
        const Layer<HeightVoxel>& layer,
        pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);

  pointcloud->clear();
  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Temp variables.
  double height = 0.0;
  // Iterate over all blocks.
  for (const BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const Block<HeightVoxel>& block = layer.getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      if (visualizeHeightVoxels(block.getVoxelByLinearIndex(linear_index), coord,
                       &height)) {
        pcl::PointXYZI point;
        point.x = coord.x();
        point.y = coord.y();
        point.z = coord.z();
        point.intensity = height;
        pointcloud->push_back(point);
      }
    }
  }
}

inline void createDistancePointcloudFromTsdfLayerSlice(
    const Layer<TsdfVoxel>& layer, unsigned int free_plane_index,
    FloatingPoint free_plane_val, pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  // Make sure that the free_plane_val doesn't fall evenly between 2 slices.
  // Prefer to push it up.
  // Using std::remainder rather than std::fmod as the latter has huge crippling
  // issues with floating-point division.
  if (std::remainder(free_plane_val, layer.voxel_size()) < kFloatEpsilon) {
    free_plane_val += layer.voxel_size() / 2.0;
  }

  createColorPointcloudFromLayer<TsdfVoxel>(
      layer,
      std::bind(&visualizeDistanceIntensityTsdfVoxelsSlice, ph::_1, ph::_2,
                free_plane_index, free_plane_val, layer.voxel_size(), ph::_3),
      pointcloud);
}

inline void createDistancePointcloudFromEsdfLayerSlice(
    const Layer<EsdfVoxel>& layer, unsigned int free_plane_index,
    FloatingPoint free_plane_val, pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  // Make sure that the free_plane_val doesn't fall evenly between 2 slices.
  // Prefer to push it up.
  // Using std::remainder rather than std::fmod as the latter has huge crippling
  // issues with floating-point division.
  if (std::remainder(free_plane_val, layer.voxel_size()) < kFloatEpsilon) {
    free_plane_val += layer.voxel_size() / 2.0;
  }

  createColorPointcloudFromLayer<EsdfVoxel>(
      layer,
      std::bind(&visualizeDistanceIntensityEsdfVoxelsSlice, ph::_1, ph::_2,
                free_plane_index, free_plane_val, layer.voxel_size(), ph::_3),
      pointcloud);
}

inline void createOccupancyBlocksFromTsdfLayer(
    const Layer<TsdfVoxel>& layer, const std::string& frame_id,
    const FloatingPoint occupied_voxel_min_distance,
    visualization_msgs::MarkerArray* marker_array) {
  std::cout << " begin createOccupancyBlocksFromTsdfLayer" << std::endl;
  auto start__ = std::chrono::system_clock::now();
  CHECK_NOTNULL(marker_array);
  createOccupancyBlocksFromLayer<TsdfVoxel>(
      layer,
      std::bind(visualizeOccupiedTsdfVoxels, std::placeholders::_1,
                std::placeholders::_2, occupied_voxel_min_distance),
      frame_id, marker_array);
  auto end__ = std::chrono::system_clock::now();
  auto duration__ = std::chrono::duration<double>(end__ - start__).count();
  std::cout << " end createOccupancyBlocksFromTsdfLayer, uses : " << duration__ << " s" << std::endl; 
}

inline void createOccupancyBlocksFromOccupancyLayer(
    const Layer<OccupancyVoxel>& layer, const std::string& frame_id,
    visualization_msgs::MarkerArray* marker_array) {
  CHECK_NOTNULL(marker_array);
  createOccupancyBlocksFromLayer<OccupancyVoxel>(
      layer, &visualizeOccupiedOccupancyVoxels, frame_id, marker_array);
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_PTCLOUD_VIS_H_
