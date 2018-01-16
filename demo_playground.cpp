////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-01
// RVG, NLPR, CASIA
// File Description: various algorithms with visualization
////////////////////////////////////////////////////

#include "MainWindow.h"
#include <QApplication>
#include <CGAL/Qt/resources.h>

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  app.setOrganizationDomain("ia.cas.cn");
  app.setOrganizationName("CASIA");
  app.setApplicationName("Demo Playground");

  // Import resources from libCGALQt.
  // See http://doc.qt.io/qt-5/qdir.html#Q_INIT_RESOURCE

  CGAL_QT_INIT_RESOURCES;

  MainWindow mainWindow;
  mainWindow.show();
  QStringList args = app.arguments();
  args.removeAt(0);

  if(!args.empty() && args[0] == "--use-meta") {
    mainWindow.setAddKeyFrameKeyboardModifiers(::Qt::MetaModifier);
    args.removeAt(0);
  }

  Q_FOREACH(QString filename, args)
    mainWindow.open(filename);

  return app.exec();
}
