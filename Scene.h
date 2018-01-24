#ifndef SCENE_H
#define SCENE_H

#include "types.h"
#include "parameters.h"

namespace Algs {
  class Shape_detection;
  class Surface_simplification;
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
  // RANSAC shape detection on point cloud algorithm
  int shape_detection(const std::string &fname, const Params::Shape_detection &params);

  // triangulated surface mesh simplification algorithm
  //int surface_simplification();
  int surface_simplification(const std::string &fname);

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
  Algs::Shape_detection *m_shape_detection;
  Algs::Surface_simplification *m_surface_simplification;
}; // end class Scene


#endif // SCENE_H
