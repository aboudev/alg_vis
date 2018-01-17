#ifndef PARAMETERS_H
#define PARAMETERS_H

namespace Params {

struct Shape_detection {
  /// Sets probability to miss the largest primitive at each iteration.
  double probability;
  /// Detect shapes with at minimum points.
  std::size_t min_points;
  /// Sets maximum Euclidean distance between a point and a shape.
  double epsilon;
  /// Sets maximum Euclidean distance between points to be clustered.
  double cluster_epsilon;
  /// Sets maximum normal deviation.
  double normal_threshold;
};

} // Params

#endif // PARAMETERS_H
