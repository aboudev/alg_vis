////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-16
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#ifndef Scene_shape_detection_H
#define Scene_shape_detection_H

#include "types.h"
#include "parameters.h"

namespace Algs {

/************************************************************************/
/* RANSAC Shape Detection Algorithm                                     */
/* http://doc.cgal.org/latest/Point_set_shape_detection_3/index.html    */
/************************************************************************/
class Shape_detection {
  // Type declarations
  typedef std::pair<Kernel2::Point_3, Kernel2::Vector_3>       Point_with_normal;
  typedef std::vector<Point_with_normal>                       Pwn_vector;

public:
  Shape_detection() {}

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

#endif // Scene_shape_detection_H
