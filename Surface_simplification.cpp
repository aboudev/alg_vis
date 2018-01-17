////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-16
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#include "Surface_simplification.h"

#include <iostream>
#include <fstream>

namespace Algs {

int Surface_simplification::simplify()
{
  if (m_pPolyhedron == nullptr) {
    std::cout << "Load polyhedron first." << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "\nStarting surface simplification...\n" 
    << m_pPolyhedron->size_of_halfedges() / 2
    << " original undirected edges." << std::endl;

  // This is a stop predicate (defines when the algorithm terminates).
  // In this example, the simplification stops when the number of undirected edges
  // left in the surface mesh drops below the specified number (1000)
  SMS::Count_stop_predicate<Polyhedron> stop(14000);

  // This the actual call to the simplification algorithm.
  // The surface mesh and stop conditions are mandatory arguments.
  // The index maps are needed because the vertices and edges
  // of this surface mesh lack an "id()" field.
  int r = SMS::edge_collapse
  (*m_pPolyhedron
    , stop
    , CGAL::parameters::vertex_index_map(get(CGAL::vertex_external_index, *m_pPolyhedron))
    .halfedge_index_map(get(CGAL::halfedge_external_index, *m_pPolyhedron))
    .get_cost(SMS::Edge_length_cost<Polyhedron>())
    .get_placement(SMS::Midpoint_placement<Polyhedron>())
  );

  std::cout << "\nFinished...\n" << r << " edges removed.\n"
    << ((*m_pPolyhedron).size_of_halfedges() / 2) << " final edges.\n";

  std::ofstream os("out.off");
  os << (*m_pPolyhedron);

  return EXIT_SUCCESS;
}

// The following is a Visitor that keeps track of the simplification process.
// In this example the progress is printed real-time and a few statistics are
// recorded (and printed in the end).

int Surface_simplification::simplify(const std::string &filename)
{
  std::cout << "Opening file \"" << filename << "\"" << std::endl;
  std::ifstream ifs(filename);
  if (!ifs.is_open()) {
    std::cerr << "unable to open file" << std::endl;
    return EXIT_FAILURE;
  }

  Surface_mesh surface_mesh;
  ifs >> surface_mesh;
  if (!ifs) {
    std::cerr << "invalid OFF file" << std::endl;
    return EXIT_FAILURE;
  }

  // The items in this polyhedron have an "id()" field 
  // which the default index maps used in the algorithm
  // need to get the index of a vertex/edge.
  // However, the Polyhedron_3 class doesn't assign any value to
  // this id(), so we must do it here:
  int index = 0;

  for (Surface_mesh::Halfedge_iterator eb = surface_mesh.halfedges_begin()
    , ee = surface_mesh.halfedges_end()
    ; eb != ee
    ; ++eb
    )
    eb->id() = index++;
  index = 0;
  for (Surface_mesh::Vertex_iterator vb = surface_mesh.vertices_begin()
    , ve = surface_mesh.vertices_end()
    ; vb != ve
    ; ++vb
    )
    vb->id() = index++;

  // In this example, the simplification stops when the number of undirected edges
  // drops below 10% of the initial count
  SMS::Count_ratio_stop_predicate<Surface_mesh> stop(0.1);

  Stats stats;

  My_visitor vis(&stats);

  // The index maps are not explicitly passed as in the previous
  // example because the surface mesh items have a proper id() field.
  // On the other hand, we pass here explicit cost and placement
  // function which differ from the default policies, omitted in
  // the previous example.
  int r = SMS::edge_collapse
  (surface_mesh
    , stop
    , CGAL::parameters::get_cost(SMS::Edge_length_cost<Surface_mesh>())
    .get_placement(SMS::Midpoint_placement<Surface_mesh>())
    .visitor(vis)
  );

  std::cout << "\nEdges collected: " << stats.collected
    << "\nEdges processed: " << stats.processed
    << "\nEdges collapsed: " << stats.collapsed
    << std::endl
    << "\nEdges not collapsed due to topological constraints: " << stats.non_collapsable
    << "\nEdge not collapsed due to cost computation constraints: " << stats.cost_uncomputable
    << "\nEdge not collapsed due to placement computation constraints: " << stats.placement_uncomputable
    << std::endl;

  std::cout << "\nFinished...\n" << r << " edges removed.\n"
    << (surface_mesh.size_of_halfedges() / 2) << " final edges.\n";

  std::ofstream os("out.off");
  os << surface_mesh;
  os.flush();
  os.close();

  if (m_pPolyhedron != nullptr)
    delete m_pPolyhedron;
  m_pPolyhedron = new Polyhedron;

  // load out.off to m_pPolyhedron for visualization
  std::ifstream is("out.off");
  is >> *m_pPolyhedron;
  is.close();

  return EXIT_SUCCESS;
}

} // Algs
