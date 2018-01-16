////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-16
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#ifdef SURFACE_SIMPLIFICATION_H
#define SURFACE_SIMPLIFICATION_H

#include "types.h"

namespace Algs {

/************************************************************************/
/* Lindstrom - Turk Surface Simplification                              */
/* http://doc.cgal.org/latest/Surface_mesh_simplification/              */
/************************************************************************/
class Surface_simplification {
  struct Stats {
    Stats()
      : collected(0)
      , processed(0)
      , collapsed(0)
      , non_collapsable(0)
      , cost_uncomputable(0)
      , placement_uncomputable(0)
    {}

    std::size_t collected;
    std::size_t processed;
    std::size_t collapsed;
    std::size_t non_collapsable;
    std::size_t cost_uncomputable;
    std::size_t placement_uncomputable;
  };

  struct My_visitor : SMS::Edge_collapse_visitor_base<Surface_mesh> {
    My_visitor(Stats* s) : stats(s) {}
    // Called during the collecting phase for each edge collected.
    void OnCollected(SmProfile const&, boost::optional<double> const&)
    {
      ++stats->collected;
      std::cerr << "\rEdges collected: " << stats->collected << std::flush;
    }

    // Called during the processing phase for each edge selected.
    // If cost is absent the edge won't be collapsed.
    void OnSelected(SmProfile const&
      , boost::optional<double> cost
      , std::size_t             initial
      , std::size_t             current
    )
    {
      ++stats->processed;
      if (!cost)
        ++stats->cost_uncomputable;

      if (current == initial)
        std::cerr << "\n" << std::flush;
      std::cerr << "\r" << current << std::flush;
    }

    // Called during the processing phase for each edge being collapsed.
    // If placement is absent the edge is left uncollapsed.
    void OnCollapsing(SmProfile const&
      , boost::optional<SmPoint>  placement
    )
    {
      if (!placement)
        ++stats->placement_uncomputable;
    }

    // Called for each edge which failed the so called link-condition,
    // that is, which cannot be collapsed because doing so would
    // turn the surface mesh into a non-manifold.
    void OnNonCollapsable(SmProfile const&)
    {
      ++stats->non_collapsable;
    }

    // Called AFTER each edge has been collapsed
    void OnCollapsed(SmProfile const&, SmVertex_handle)
    {
      ++stats->collapsed;
    }

    Stats* stats;
  };

public:
  Surface_simplification() {}

  const Bbox &bbox() { return m_bbox; }

  void simplify();

  void simplify(const std::string &filename);

private:
  Bbox m_bbox;
  Polyhedron *m_pPolyhedron;
};

}

#endif // SURFACE_SIMPLIFICATION_H
