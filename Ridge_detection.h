////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-24
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#ifndef RIDGE_DETECTION_H
#define RIDGE_DETECTION_H

#include "types.h"
#include "parameters.h"

namespace Algs {

/************************************************************************/
/* Approximation of Ridges on Triangulated Surface Meshes               */
/* https://doc.cgal.org/latest/Ridges_3/index.html                      */
/************************************************************************/
class Ridge_detection {
public:
  Ridge_detection() {}

  void detect(const std::string &fname);

  const Bbox_3 &bbox() { return m_bbox; }

  void draw();

private:
  Bbox_3 m_bbox;

  Surface_mesh m_mesh;

  // rendering data
  std::vector<std::vector<Point_3>> m_ridges;
  std::vector<Point_3> m_umbilics;
};

} // namespace Algs

#endif // RIDGE_DETECTION_H
