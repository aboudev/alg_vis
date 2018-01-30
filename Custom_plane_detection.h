////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-30
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#ifndef Custom_plane_detection_H
#define Custom_plane_detection_H

#include "types.h"
#include "parameters.h"

namespace Algs {

/************************************************************************/
/* RANSAC Custom Plane Detection                                        */
/* Plane with constraint parameter, e.g. horizontal                     */
/************************************************************************/
class Custom_plane_detection {
  // Type declarations
  typedef std::pair<Kernel2::Point_3, Kernel2::Vector_3>       Point_with_normal;
  typedef std::vector<Point_with_normal>                       Pwn_vector;

public:
  Custom_plane_detection() {}

  void detect(const std::string &fname, const Params::Shape_detection &params);

  const Bbox_3 &bbox() { return m_bbox; }

  void draw();

private:
  Bbox_3 m_bbox;

  // points with normals.
  Pwn_vector m_points;
  // shape index of each point
  std::vector<int> m_point_shapes;
  // shape color
  std::vector<std::size_t> m_shape_colors;

  // convex hulls of shape points
  std::vector<std::vector<Kernel2::Point_3>> m_convex_hulls;
};

} // namespace Algs

#endif // Custom_plane_detection_H
