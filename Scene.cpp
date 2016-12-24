#include "Scene.h"

#include <iostream>
#include <fstream>

#include <QString>
#include <QTextStream>
#include <QFileInfo>
#include <QInputDialog>

#include <CGAL/Timer.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Subdivision_method_3.h>

#include <CGAL/centroid.h>
#include <CGAL/linear_least_squares_fitting_3.h>

#include "render_edges.h"

Scene::Scene()
{
    m_pPolyhedron = NULL;

    // view options
    m_view_polyhedron = true;

    int i = 0;
    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.515600;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.531300;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.546900;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.562500;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.578100;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.593800;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.609400;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.625000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.640600;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.656300;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.671900;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.687500;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.703100;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.718800;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.734400;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.750000;
    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.765600;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.781300;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.796900;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.812500;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.828100;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.843800;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.859400;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.875000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.890600;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.906300;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.921900;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.937500;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.953100;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.968800;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.984400;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 1.000000;
    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.015600;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.031300;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.046900;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.062500;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.078100;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.093800;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.109400;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.125000;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.140600;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.156300;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.171900;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.187500;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.203100;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.218800;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.234400;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.250000;    LUT_Seg[i++] = 1.000000;
    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.265600;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.281300;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.296900;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.312500;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.328100;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.343800;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.359400;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.375000;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.390600;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.406300;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.421900;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.437500;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.453100;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.468800;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.484400;    LUT_Seg[i++] = 1.000000;
    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.500000;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.515600;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.531300;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.546900;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.562500;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.578100;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.593800;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.609400;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.625000;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.640600;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.656300;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.671900;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.687500;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.703100;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.718800;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.734400;    LUT_Seg[i++] = 1.000000;
    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.750000;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.765600;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.781300;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.796900;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.812500;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.828100;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.843800;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.859400;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.875000;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.890600;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.906300;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.921900;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.937500;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.953100;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.968800;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.984400;    LUT_Seg[i++] = 1.000000;
    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.015600;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 1.000000;        LUT_Seg[i++] = 0.031300;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.984400;        LUT_Seg[i++] = 0.046900;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.968800;        LUT_Seg[i++] = 0.062500;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.953100;        LUT_Seg[i++] = 0.078100;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.937500;        LUT_Seg[i++] = 0.093800;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.921900;        LUT_Seg[i++] = 0.109400;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.906300;        LUT_Seg[i++] = 0.125000;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.890600;        LUT_Seg[i++] = 0.140600;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.875000;        LUT_Seg[i++] = 0.156300;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.859400;        LUT_Seg[i++] = 0.171900;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.843800;        LUT_Seg[i++] = 0.187500;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.828100;        LUT_Seg[i++] = 0.203100;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.812500;        LUT_Seg[i++] = 0.218800;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.796900;        LUT_Seg[i++] = 0.234400;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.781300;
    LUT_Seg[i++] = 0.250000;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.765600;        LUT_Seg[i++] = 0.265600;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.750000;        LUT_Seg[i++] = 0.281300;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.734400;        LUT_Seg[i++] = 0.296900;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.718800;        LUT_Seg[i++] = 0.312500;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.703100;        LUT_Seg[i++] = 0.328100;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.687500;        LUT_Seg[i++] = 0.343800;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.671900;        LUT_Seg[i++] = 0.359400;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.656300;        LUT_Seg[i++] = 0.375000;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.640600;        LUT_Seg[i++] = 0.390600;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.625000;        LUT_Seg[i++] = 0.406300;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.609400;        LUT_Seg[i++] = 0.421900;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.593800;        LUT_Seg[i++] = 0.437500;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.578100;        LUT_Seg[i++] = 0.453100;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.562500;        LUT_Seg[i++] = 0.468800;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.546900;        LUT_Seg[i++] = 0.484400;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.531300;
    LUT_Seg[i++] = 0.500000;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.515600;        LUT_Seg[i++] = 0.515600;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.500000;        LUT_Seg[i++] = 0.531300;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.484400;        LUT_Seg[i++] = 0.546900;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.468800;        LUT_Seg[i++] = 0.562500;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.453100;        LUT_Seg[i++] = 0.578100;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.437500;        LUT_Seg[i++] = 0.593800;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.421900;        LUT_Seg[i++] = 0.609400;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.406300;        LUT_Seg[i++] = 0.625000;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.390600;        LUT_Seg[i++] = 0.640600;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.375000;        LUT_Seg[i++] = 0.656300;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.359400;        LUT_Seg[i++] = 0.671900;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.343800;        LUT_Seg[i++] = 0.687500;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.328100;        LUT_Seg[i++] = 0.703100;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.312500;        LUT_Seg[i++] = 0.718800;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.296900;        LUT_Seg[i++] = 0.734400;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.281300;
    LUT_Seg[i++] = 0.750000;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.265600;        LUT_Seg[i++] = 0.765600;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.250000;        LUT_Seg[i++] = 0.781300;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.234400;        LUT_Seg[i++] = 0.796900;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.218800;        LUT_Seg[i++] = 0.812500;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.203100;        LUT_Seg[i++] = 0.828100;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.187500;        LUT_Seg[i++] = 0.843800;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.171900;        LUT_Seg[i++] = 0.859400;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.156300;        LUT_Seg[i++] = 0.875000;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.140600;        LUT_Seg[i++] = 0.890600;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.125000;        LUT_Seg[i++] = 0.906300;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.109400;        LUT_Seg[i++] = 0.921900;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.093800;        LUT_Seg[i++] = 0.937500;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.078100;        LUT_Seg[i++] = 0.953100;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.062500;        LUT_Seg[i++] = 0.968800;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.046900;        LUT_Seg[i++] = 0.984400;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.031300;
    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.015600;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.984400;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.968800;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.953100;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.937500;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.921900;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.906300;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.890600;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.875000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.859400;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.843800;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.828100;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.812500;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.796900;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.781300;    LUT_Seg[i++] = 0.000000;
    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.765600;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.750000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.734400;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.718800;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.703100;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.687500;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.671900;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.656300;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.640600;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.625000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.609400;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.593800;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.578100;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.562500;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.546900;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.531300;    LUT_Seg[i++] = 0.000000;
    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.515600;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.500000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.484400;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.468800;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.453100;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.437500;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.421900;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.406300;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.390600;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.375000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.359400;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.343800;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.328100;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.312500;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.296900;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.281300;    LUT_Seg[i++] = 0.000000;
    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.265600;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.250000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.234400;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.218800;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.203100;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.187500;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.171900;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.156300;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.140600;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.125000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.109400;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.093800;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.078100;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.062500;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.046900;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.031300;    LUT_Seg[i++] = 0.000000;
    LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.015600;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 1.000000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.984400;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.968800;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.953100;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.937500;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.921900;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.906300;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.890600;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.875000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.859400;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.843800;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.828100;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.812500;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.796900;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.781300;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;
    LUT_Seg[i++] = 0.765600;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.750000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.734400;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.718800;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.703100;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.687500;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.671900;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.656300;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.640600;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.625000;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.609400;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.593800;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.578100;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.562500;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.546900;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;        LUT_Seg[i++] = 0.531300;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;
    LUT_Seg[i++] = 0.515600;    LUT_Seg[i++] = 0.000000;    LUT_Seg[i++] = 0.000000;
}

Scene::~Scene()
{
    delete m_pPolyhedron;
}

int Scene::open(QString filename)
{
    QTextStream cerr(stderr);
    cerr << QString("Opening file \"%1\"\n").arg(filename);
    QApplication::setOverrideCursor(QCursor(::Qt::WaitCursor));

    QFileInfo fileinfo(filename);
    std::ifstream in(filename.toUtf8());

    if(!in || !fileinfo.isFile() || ! fileinfo.isReadable())
    {
        std::cerr << "unable to open file" << std::endl;
        QApplication::restoreOverrideCursor();
        return -1;
    }

    if(m_pPolyhedron != NULL)
        delete m_pPolyhedron;

    // allocate new polyhedron
    m_pPolyhedron = new Polyhedron;
    in >> *m_pPolyhedron;
    if(!in)
    {
        std::cerr << "invalid OFF file" << std::endl;
        QApplication::restoreOverrideCursor();

        delete m_pPolyhedron;
        m_pPolyhedron = NULL;

        return -1;
    }

    QApplication::restoreOverrideCursor();
    return 0;
}

void Scene::update_bbox()
{
    std::cout << "Compute bbox...";
    m_bbox = Bbox();

    if(m_pPolyhedron == NULL)
    {
        std::cout << "failed (no polyhedron)." << std::endl;
        return;
    }

    if(m_pPolyhedron->empty())
    {
        std::cout << "failed (empty polyhedron)." << std::endl;
        return;
    }

    Polyhedron::Point_iterator it = m_pPolyhedron->points_begin();
    m_bbox = (*it).bbox();
    for(; it != m_pPolyhedron->points_end();it++)
        m_bbox = m_bbox + (*it).bbox();
    std::cout << "done (" << m_pPolyhedron->size_of_facets()
        << " facets)" << std::endl;
}

void Scene::draw()
{
    if(m_view_polyhedron)
        render_polyhedron();

    render_line();
    render_plane();
    render_centroid();
    render_shape();
}

void Scene::render_plane()
{
    ::glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    ::glLineWidth(3.0f);
    ::glColor3ub(255,0,0);
    ::glBegin(GL_QUADS);
    Point o = m_plane.projection(m_centroid);
    Point a = o + normalize(m_plane.base1()) + normalize(m_plane.base2());
    Point b = o + normalize(m_plane.base1()) - normalize(m_plane.base2());
    Point c = o - normalize(m_plane.base1()) - normalize(m_plane.base2());
    Point d = o - normalize(m_plane.base1()) + normalize(m_plane.base2());
    ::glVertex3d(a.x(),a.y(),a.z());
    ::glVertex3d(b.x(),b.y(),b.z());
    ::glVertex3d(c.x(),c.y(),c.z());
    ::glVertex3d(d.x(),d.y(),d.z());
    ::glEnd();
}

void Scene::render_line()
{
    ::glLineWidth(3.0f);
    ::glColor3ub(0,0,255);
    ::glBegin(GL_LINES);
    Point o = m_line.projection(m_centroid);
    Point a = o + normalize(m_line.to_vector());
    Point b = o - normalize(m_line.to_vector());
    ::glVertex3d(a.x(),a.y(),a.z());
    ::glVertex3d(b.x(),b.y(),b.z());
    ::glEnd();
}

void Scene::render_centroid()
{
    ::glPointSize(10.0f);
    ::glColor3ub(0,128,0);
    ::glBegin(GL_POINTS);
    ::glVertex3d(m_centroid.x(),m_centroid.y(),m_centroid.z());
    ::glEnd();
}

void Scene::render_shape()
{
    if (m_points.empty())
        return;

    Efficient_ransac::Shape_range shapes = m_ransac.shapes();
    Efficient_ransac::Shape_range::iterator it = shapes.begin();

    // shape color table
    std::vector<int> color_tab;
    for (int i = 0; it != shapes.end(); ++it, ++i) {
        double R = ((double)i / double(shapes.size())) * (double)255.0;
        int indiceLut = std::floor(R);
        color_tab.push_back(indiceLut);
    }
    std::vector<int>::iterator color_itr = color_tab.begin();

    std::set<int> pt_idx_set;
    for (int i = 0; i < m_points.size(); ++i)
        pt_idx_set.insert(i);

    ::glDisable(GL_LIGHTING);

    // draw point cloud with respect color
    ::glPointSize(5.0);
    shapes = m_ransac.shapes();
    it = shapes.begin();
    while (it != shapes.end()) {
        ::glColor3f(LUT_Seg[(*color_itr) * 3], LUT_Seg[(*color_itr) * 3 + 1], LUT_Seg[(*color_itr) * 3 + 2]);
        ++color_itr;
        ::glBegin(GL_POINTS);
        // Iterates through point indices assigned to each detected shape.
        std::vector<std::size_t>::const_iterator
            index_it = (*it)->indices_of_assigned_points().begin();
        while (index_it != (*it)->indices_of_assigned_points().end()) {
            // Retrieves point
            const Point_with_normal &p = *(m_points.begin() + (*index_it));
            ::glVertex3d(p.first.x(), p.first.y(), p.first.z());
            // Proceeds with next point.
            index_it++;
            pt_idx_set.erase(int(*index_it));
        }
        // Proceeds with next detected shape.
        it++;
        ::glEnd();
    }
    // remaining points
    std::cout << m_points.size() << " --- " << pt_idx_set.size() << std::endl;
    ::glColor3f(0.0, 0.0, 0.0);
    ::glBegin(GL_POINTS);
    for (std::set<int>::iterator itr = pt_idx_set.begin();
        itr != pt_idx_set.end(); ++itr)
        ::glVertex3d(m_points[*itr].first.x(), m_points[*itr].first.y(), m_points[*itr].first.z());
    ::glEnd();

    ::glLineWidth(1.0);
    it = shapes.begin();
    color_itr = color_tab.begin();
    while (it != shapes.end()) {
        ::glColor3f(LUT_Seg[(*color_itr) * 3], LUT_Seg[(*color_itr) * 3 + 1], LUT_Seg[(*color_itr) * 3 + 2]);
        ++color_itr;
        ::glBegin(GL_LINES);
        // Iterates through point indices assigned to each detected shape.
        std::vector<std::size_t>::const_iterator
            index_it = (*it)->indices_of_assigned_points().begin();
        while (index_it != (*it)->indices_of_assigned_points().end()) {
            // Retrieves point
            const Point_with_normal &p = *(m_points.begin() + (*index_it));
            ::glVertex3d(p.first.x(), p.first.y(), p.first.z());
            ::glVertex3d(p.first.x() + p.second.x() * 0.05, p.first.y() + p.second.y() * 0.05, p.first.z() + p.second.z() * 0.05);
            // Proceeds with next point.
            index_it++;
        }
        // Proceeds with next detected shape.
        it++;
        ::glEnd();
    }
    ::glEnable(GL_LIGHTING);
}

void Scene::toggle_view_poyhedron()
{
    m_view_polyhedron = !m_view_polyhedron;
}

void Scene::render_polyhedron()
{
    // draw black edges
    if (m_pPolyhedron != NULL)
    {
        ::glDisable(GL_LIGHTING);
        ::glColor3ub(0, 0, 0);
        ::glLineWidth(1.0f);
        gl_render_edges(*m_pPolyhedron);
    }
}

Vector Scene::normalize(const Vector& v)
{
    return v / std::sqrt(v*v);
}

/************************************************************************/
/* PCA Algorithm                                                        */
/* http://doc.cgal.org/latest/Principal_component_analysis/index.html   */
/************************************************************************/
void Scene::refine_loop()
{
    if(m_pPolyhedron == NULL)
    {
        std::cout << "Load polyhedron first." << std::endl;
        return;
    }
    std::cout << "Loop subdivision...";
    CGAL::Subdivision_method_3::Loop_subdivision(*m_pPolyhedron, 1);
    std::cout << "done (" << m_pPolyhedron->size_of_facets() << " facets)" << std::endl;
}

void Scene::fit_triangles()
{
    if (m_pPolyhedron == NULL) {
        std::cout << "Load polyhedron first." << std::endl;
        return;
    }

    std::cout << "Fit triangles...";

    std::list<Triangle> triangles;
    Polyhedron::Facet_iterator it;
    for(it = m_pPolyhedron->facets_begin();
        it != m_pPolyhedron->facets_end();
        it++) {
        Polyhedron::Halfedge_handle he = it->halfedge();
        const Point& a = he->vertex()->point();
        const Point& b = he->next()->vertex()->point();
        const Point& c = he->next()->next()->vertex()->point();
        Triangle triangle(a,b,c);
        triangles.push_back(triangle);
    }

    m_centroid = CGAL::centroid(triangles.begin(),triangles.end());
    CGAL::linear_least_squares_fitting_3(triangles.begin(),
        triangles.end(), m_line, CGAL::Dimension_tag<2>()); 
    CGAL::linear_least_squares_fitting_3(triangles.begin(),
        triangles.end(), m_plane, CGAL::Dimension_tag<2>()); 

    std::cout << "done" << std::endl;
}

void Scene::fit_edges()
{
    if (m_pPolyhedron == NULL) {
        std::cout << "Load polyhedron first." << std::endl;
        return;
    }

    std::cout << "Fit edges...";

    std::list<Segment> segments;
    Polyhedron::Edge_iterator he;
    for(he = m_pPolyhedron->edges_begin();
        he != m_pPolyhedron->edges_end();
        he++) {
        const Point& a = he->vertex()->point();
        const Point& b = he->opposite()->vertex()->point();
        Segment segment(a,b);
        segments.push_back(segment);
    }
    
    m_centroid = CGAL::centroid(segments.begin(),segments.end());
    CGAL::linear_least_squares_fitting_3(segments.begin(),
        segments.end(), m_line, CGAL::Dimension_tag<1>()); 
    CGAL::linear_least_squares_fitting_3(segments.begin(),
        segments.end(), m_plane, CGAL::Dimension_tag<1>()); 

    std::cout << "done" << std::endl;
}

void Scene::fit_vertices()
{
    if (m_pPolyhedron == NULL) {
        std::cout << "Load polyhedron first." << std::endl;
        return;
    }

    std::cout << "Fit vertices...";

    std::list<Point> points;
    Polyhedron::Vertex_iterator v;
    for(v = m_pPolyhedron->vertices_begin();
        v != m_pPolyhedron->vertices_end();
        v++) {
        const Point& p = v->point();
        points.push_back(p);
    }
    
    m_centroid = CGAL::centroid(points.begin(),points.end());
    CGAL::linear_least_squares_fitting_3(points.begin(),
        points.end(), m_line, CGAL::Dimension_tag<0>()); 
    CGAL::linear_least_squares_fitting_3(points.begin(),
        points.end(), m_plane, CGAL::Dimension_tag<0>()); 

    std::cout << "done" << std::endl;
}

/************************************************************************/
/* RANSAC Shape Detection Algorithm                                     */
/* http://doc.cgal.org/latest/Point_set_shape_detection_3/index.html    */
/************************************************************************/
int Scene::shape_detection()
{
    /*if (m_pPolyhedron == NULL)
    {
    std::cout << "Load polyhedron first." << std::endl;
    return EXIT_FAILURE;
    }*/

    std::cout << "Shape detection...";

    // QFileDialog
    /*QString fileName = QFileDialog::getOpenFileName(this,
    tr("Open point with normal"), ".", tr("Point Cloud File (*.pwn)") );*/
    std::string fileName("data/cube.pwn");

    m_points.clear();
    // Loads point set from a file. 
    // read_xyz_points_and_normals takes an OutputIterator for storing the points
    // and a property map to store the normal vector with each point.
    std::ifstream stream(fileName);
    if (!stream ||
        !CGAL::read_xyz_points_and_normals(stream,
            std::back_inserter(m_points),
            Point_map(),
            Normal_map()))
    {
        std::cerr << "Error: cannot read file cube.pwn" << std::endl;
        return EXIT_FAILURE;
    }
    // Instantiates shape detection engine.
    // Provides the input data.
    m_ransac.set_input(m_points);
    // Registers planar shapes via template method.
    m_ransac.add_shape_factory<RansacPlane>();
    //// Detects registered shapes with default parameters.
    //m_ransac.detect();
    //// Prints number of detected shapes.
    //std::cout << m_ransac.shapes().end() - m_ransac.shapes().begin() << " shapes detected." << std::endl;


    // Measures time before setting up the shape detection.
    CGAL::Timer time;
    time.start();
    // Build internal data structures.
    m_ransac.preprocess();
    // Measures time after preprocessing.
    time.stop();
    std::cout << "preprocessing took: " << time.time() * 1000 << "ms" << std::endl;
    // Perform detection several times and choose result with highest coverage.
    Efficient_ransac::Shape_range shapes = m_ransac.shapes();
    FT best_coverage = 0;
    for (size_t i = 0; i<3; i++) {
        // Reset timer.
        time.reset();
        time.start();
        // Detects shapes.
        m_ransac.detect();
        // Measures time after detection.
        time.stop();
        // Compute coverage, i.e. ratio of the points assigned to a shape.
        FT coverage = FT(m_points.size() - m_ransac.number_of_unassigned_points())
            / FT(m_points.size());
        // Prints number of assigned shapes and unsassigned points.
        std::cout << "time: " << time.time() * 1000 << "ms" << std::endl;
        std::cout << m_ransac.shapes().end() - m_ransac.shapes().begin() << " primitives, "
            << coverage << " coverage" << std::endl;

        // Choose result with highest coverage.
        if (coverage > best_coverage) {
            best_coverage = coverage;
            // Efficient_ransac::shapes() provides
            // an iterator range to the detected shapes. 
            shapes = m_ransac.shapes();
        }
    }

    Efficient_ransac::Shape_range::iterator it = shapes.begin();
    while (it != shapes.end()) {
        boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
        // Using Shape_base::info() for printing 
        // the parameters of the detected shape.
        std::cout << (*it)->info();
        // Sums distances of points to detected shapes.
        FT sum_distances = 0;
        // Iterates through point indices assigned to each detected shape.
        std::vector<std::size_t>::const_iterator
            index_it = (*it)->indices_of_assigned_points().begin();
        while (index_it != (*it)->indices_of_assigned_points().end()) {

            // Retrieves point
            const Point_with_normal &p = *(m_points.begin() + (*index_it));
            // Adds Euclidean distance between point and shape.
            sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));
            // Proceeds with next point.
            index_it++;
        }
        // Computes and prints average distance.
        FT average_distance = sum_distances / shape->indices_of_assigned_points().size();
        std::cout << " average distance: " << average_distance << std::endl;
        // Proceeds with next detected shape.
        it++;
    }

    // plane regularization in CGAL 4.9
    // described in "LOD Generation for Urban Scenes", Verdie. et al
    //CGAL::regulariz_planes(ransac,
    //    true, //Regularize parallelism
    //    true, // Regularize orthogonality
    //    false, // Do not regularize coplanarity
    //    true, // Regularize Z-symmetry (default)
    //    10); // 10 degrees of tolerance for parallelism / orthogonality

    std::cout << "done" << std::endl;

    return EXIT_SUCCESS;
}

/************************************************************************/
/* Lindstrom - Turk Surface Simplification                              */
/* http://doc.cgal.org/latest/Surface_mesh_simplification/              */
/************************************************************************/
//int Scene::surface_simplification()
//{
//    if (m_pPolyhedron == NULL) {
//        std::cout << "Load polyhedron first." << std::endl;
//        return EXIT_FAILURE;
//    }
//
//    std::cout << "\nStarting surface simplification...\n" 
//        << m_pPolyhedron->size_of_halfedges() / 2
//        << " original undirected edges." << std::endl;
//
//    // This is a stop predicate (defines when the algorithm terminates).
//    // In this example, the simplification stops when the number of undirected edges
//    // left in the surface mesh drops below the specified number (1000)
//    SMS::Count_stop_predicate<Polyhedron> stop(14000);
//
//    // This the actual call to the simplification algorithm.
//    // The surface mesh and stop conditions are mandatory arguments.
//    // The index maps are needed because the vertices and edges
//    // of this surface mesh lack an "id()" field.
//    int r = SMS::edge_collapse
//    (*m_pPolyhedron
//        , stop
//        , CGAL::parameters::vertex_index_map(get(CGAL::vertex_external_index, *m_pPolyhedron))
//        .halfedge_index_map(get(CGAL::halfedge_external_index, *m_pPolyhedron))
//        .get_cost(SMS::Edge_length_cost<Polyhedron>())
//        .get_placement(SMS::Midpoint_placement<Polyhedron>())
//    );
//
//    std::cout << "\nFinished...\n" << r << " edges removed.\n"
//        << ((*m_pPolyhedron).size_of_halfedges() / 2) << " final edges.\n";
//
//    std::ofstream os("out.off");
//    os << (*m_pPolyhedron);
//
//    return EXIT_SUCCESS;
//}

// The following is a Visitor that keeps track of the simplification process.
// In this example the progress is printed real-time and a few statistics are
// recorded (and printed in the end).

struct Stats
{
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
struct My_visitor : SMS::Edge_collapse_visitor_base<Surface_mesh>
{
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

int Scene::surface_simplification(QString filename)
{
    std::cout << "Opening file \"" 
        << filename.toStdString() 
        << "\"" << std::endl;
    QFileInfo fileinfo(filename);
    std::ifstream in(filename.toUtf8());
    if (!in || !fileinfo.isFile() || !fileinfo.isReadable()) {
        std::cerr << "unable to open file" << std::endl;
        return EXIT_FAILURE;
    }
    Surface_mesh surface_mesh;
    in >> surface_mesh;
    if (!in) {
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

    std::ofstream os("out.off"); os << surface_mesh;
    // load out.off to m_pPolyhedron for visualization
    std::ifstream is("out.off");
    if (m_pPolyhedron != NULL)
        delete m_pPolyhedron;
    m_pPolyhedron = new Polyhedron;
    is >> *m_pPolyhedron;

    return EXIT_SUCCESS;
}

/************************************************************************/
/* Point set Bilateral Smoothing                                         */
/* http://doc.cgal.org/latest/Point_set_processing_3/index.html         */
/************************************************************************/
// Modified to run on surface mesh while keeping the topology
// Refer to https://github.com/bldeng/GuidedDenoising
