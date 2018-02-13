////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-02-08
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#ifndef SYMMETRIC_NORMAL_DETECTION_H
#define SYMMETRIC_NORMAL_DETECTION_H

#include "types.h"
#include "parameters.h"

#include <boost/tuple/tuple.hpp>

namespace Algs {

/************************************************************************/
/* RANSAC custom symmetric normal detection                             */
/************************************************************************/
class Symmetric_normal_detection {
  // Type declarations
  typedef boost::tuple<Kernel2::Point_3, Kernel2::Vector_3, Kernel2::Point_3> Point_with_normal_point;
  typedef std::vector<Point_with_normal_point> Pwnp_vector;

public:
  Symmetric_normal_detection() {}

  void detect(const std::string &fname,
    const Params::Shape_detection &params,
    const bool is_constrained);

  const Bbox_3 &bbox() { return m_bbox; }

  void draw();

private:
  Bbox_3 m_bbox;

  // points with normals and normal points.
  Pwnp_vector m_points;
  // shape index of each point
  std::vector<int> m_point_shapes;
  // shape color
  std::vector<std::size_t> m_shape_colors;
};

} // namespace Algs

#endif // SYMMETRIC_NORMAL_DETECTION_H
