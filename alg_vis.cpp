////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-01
// RVG, NLPR, CASIA
// File Description: algorithms with visualization
////////////////////////////////////////////////////

#include "Mainwindow.h"
#include <QApplication>
#include <CGAL/Qt/resources.h>

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  app.setOrganizationDomain("ia.cas.cn");
  app.setOrganizationName("CASIA");
  app.setApplicationName("AlgVis");

  // Import resources from libCGALQt.
  // See http://doc.qt.io/qt-5/qdir.html#Q_INIT_RESOURCE
  CGAL_QT_INIT_RESOURCES;

  Mainwindow mw;
  mw.show();

  return app.exec();
}
