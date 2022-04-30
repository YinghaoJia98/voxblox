#ifndef VOXBLOX_TRAVERSABILITY_MAP_H
#define VOXBLOX_TRAVERSABILITY_MAP_H

#include <glog/logging.h>
#include <memory>
#include <utility>

#include "voxblox/core/layer.h"
#include "voxblox/utils/layer_utils.h"
#include "voxblox/integrator/integrator_utils.h"

namespace voxblox {
/// Map holding a Traversability Layer.
class TraversabilityMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<TraversabilityMap> Ptr;

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FloatingPoint traversability_voxel_size = 0.2;
    size_t traversability_voxels_per_side = 16u;

    std::string print() const;
  };

  explicit TraversabilityMap(const Config& config)
      : traversability_layer_(new Layer<TraversabilityVoxel>(
            config.traversability_voxel_size, config.traversability_voxels_per_side)) {
    block_size_ =
        config.traversability_voxel_size * config.traversability_voxels_per_side;
  }

  // Creates a new TraversabilityMap based on a COPY of this layer.
  explicit TraversabilityMap(const Layer<TraversabilityVoxel>& layer)
      : TraversabilityMap(aligned_shared<Layer<TraversabilityVoxel>>(layer)) {}

  // Creates a new TraversabilityMap that contains this layer.
  explicit TraversabilityMap(Layer<TraversabilityVoxel>::Ptr layer)
      : traversability_layer_(layer) {
    CHECK(layer);
    block_size_ = layer->block_size();
  }

  virtual ~TraversabilityMap() {}

  Layer<TraversabilityVoxel>* getTraversabilityLayerPtr() {
    return traversability_layer_.get();
  }
  const Layer<TraversabilityVoxel>& getTraversabilityLayer() const {
    return *traversability_layer_;
  }

  FloatingPoint block_size() const { return block_size_; }

  bool getMinimumTraversabilityBetweenPoints(const Point &p1, const Point &p2, float *min_traversability) const;

  bool getAverageTraversabilityBetweenPoints(const Point &p1, const Point &p2, float *avg_traversability) const;

 protected:
  FloatingPoint block_size_;

  // The layers.
  Layer<TraversabilityVoxel>::Ptr traversability_layer_;
};

}  // namespace voxblox

#endif  // VOXBLOX_TRAVERSABILITY_MAP_H
