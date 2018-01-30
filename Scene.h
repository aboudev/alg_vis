#ifndef SCENE_H
#define SCENE_H

#include "types.h"
#include "parameters.h"

namespace Algs {
  class Surface_simplification;
  class Shape_detection;
  class Custom_plane_detection;
  class Ridge_detection;
}

class Scene
{
public:
  Scene();

  ~Scene();

public:
  // file menu
  int open(const std::string &fname);

  const Bbox_3 &bbox() { return m_bbox; }

  // toggle view options
  void toggle_view_poyhedron() {
    m_view_polyhedron = !m_view_polyhedron;
  }

  // algorithms
  // triangulated surface mesh simplification algorithm
  int surface_simplification(const std::string &fname);

  // RANSAC shape detection on point cloud algorithm
  int shape_detection(const std::string &fname, const Params::Shape_detection &params);

  // RANSAC custom plane detection on point cloud algorithm
  int custom_plane_detection(const std::string &fname, const Params::Shape_detection &params);

  // Ridge approximation
  int ridge_detection(const std::string &fname);

  // rendering
  void draw(); 
  void render_polyhedron();

private:
  // member data
  Polyhedron *m_pPolyhedron;
  Bbox_3 m_bbox;

  // view options
  bool m_view_polyhedron;

  // algorithms
  Algs::Surface_simplification *m_surface_simplification;
  Algs::Shape_detection *m_shape_detection;
  Algs::Custom_plane_detection *m_custom_plane_detection;
  Algs::Ridge_detection *m_ridge_detection;
}; // end class Scene


#endif // SCENE_H
