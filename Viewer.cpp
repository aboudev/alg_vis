#include "Viewer.h"
#include "Scene.h"

Viewer::Viewer(QWidget* parent) :
    QGLViewer(parent),
    m_pScene(NULL)
{
}

void Viewer::setScene(Scene* pScene)
{
    this->m_pScene = pScene;
}

QString Viewer::helpString() const
{
    QString text("<h1>CGAL Algorithm Demo</h1>");

    text += "This example illustrates algorithms in CGAL. ";
    text += "This demo could be used as a simple skeleton ";
    text += "for potential demos of other 3D packages or for teaching CGAL.<br><br>";

    text += "The key feature is to edit vertices/points with mouse.";
    text += "There are several modes:<br><br>";

    text += " - <u>Normal Mode</u>: ";
    text += "Rotate, zoom, or translate camera using mouse.<br>";
    text += " - <u>Insert Vertex</u>: ";
    text += "Insert a vertex on the surface of the trackball ";
    text += "and the triangulation will be updated correspondingly.<br>";
    text += " - <u>Insert Point</u>: ";
    text += "Insert a point on the surface of the trackball. ";
    text += "Its conflict region will be highlighted. ";
    text += "When the new point is moving, ";
    text += "its conflict region will be updated correspondingly.<br>";
    text += " - <u>Select</u>: ";
    text += "Click or drag mouse left button to select multiple points.<br>";
    text += " - <u>Move</u>: Hold mouse left button to move a vertex ";
    text += "and the triangulation will be updated correspondingly.<br>";
    text += " - <u>Find Nearest Neighbor</u>: ";
    text += "Place a query point and its nearest neighbor will be highlighted.<br>";
    text += " - <u>Show Empty Sphere</u>: ";
    text += "Place a query point, locate the point in a cell ";
    text += "and then show the empty sphere of that cell. ";
    text += "An empty sphere of a cell is a sphere ";
    text += "with all four vertices of the cell lying on it ";
    text += "and no other vertices inside it.<br><br>";
    text += "<b>Shift+Wheel</b> to resize the trackball when it exists. ";
    text += "See <b>Mouse</b> page for more details.<br><br>";

    text += "Other basic features include:<br>";
    text += " - Randomly generate points,<br>";
    text += " - Read/Write files,<br>";
    text += " - Show vertices, Voronoi edges, Delaunay edges, and/or facets,<br>";
    text += " - Incremental Construct: ";
    text += "Re-construct the current triangulation incrementally. ";
    text += "If no triangulation exists yet, randomly generate 100 points ";
    text += "and construct a Delaunay triangulation of those points.<br>";

    return text;
}

void Viewer::draw()
{
    QGLViewer::draw();
    if(m_pScene != NULL)
    {
        ::glClearColor(1.0f,1.0f,1.0f,0.0f);
        m_pScene->draw();
    }
}

void Viewer::initializeGL()
{
    QGLViewer::initializeGL();
    /*QColor bck_color("red");
    setBackgroundColor(bck_color);*/
    setBackgroundColor(::Qt::white);
    //QGLViewer::qglClearColor(bck_color);
}

