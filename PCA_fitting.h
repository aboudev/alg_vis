////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-16
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#ifndef Scene_shape_detection_H
#define Scene_shape_detection_H

#include "types.h"

namespace Algs {

class PCA_fitting {
public:
  PCA_fitting();

  const Bbox &bbox() { return m_bbox; }

  void draw();

private:
  Vector normalize(const Vector& v);

  void refine_loop();
  void fit_triangles();
  void fit_edges();
  void fit_vertices();

  void render_line();
  void render_plane();
  void render_centroid();

private:
  Bbox m_bbox;

  Line m_line;
  Plane m_plane;
  Point m_centroid;

  Polyhedron *m_pPolyhedron;
};

}

#endif // Scene_shape_detection_H
