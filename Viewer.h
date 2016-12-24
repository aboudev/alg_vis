#ifndef VIEWER_H
#define VIEWER_H

#include <QGLViewer/qglviewer.h>

// forward declarations
class QWidget;
class Scene;

class Viewer : 
    public QGLViewer
{

    Q_OBJECT

public:
    Viewer(QWidget * parent);

    // overload several QGLViewer virtual functions
    void draw();
    void initializeGL();
    void setScene(Scene* pScene);

    // customize help message
    QString helpString() const;

private:
    Scene* m_pScene;
}; // end class Viewer

#endif // VIEWER_H
