#include "Settings_dialog.h"

Settings_dialog::Settings_dialog(QWidget *parent) :
  QDialog(parent)
{
  setupUi(this);
  loadFromSettings();
}

void Settings_dialog::loadFromSettings()
{
  QSettings settings("settings.ini", QSettings::IniFormat);
  settings.beginGroup("AlgVis");

  if (settings.contains("shape_detection_probability"))
    shape_detection_probability->setValue(settings.value("shape_detection_probability").toDouble());
  if (settings.contains("shape_detection_min_points"))
    shape_detection_min_points->setValue(settings.value("shape_detection_min_points").toInt());
  if (settings.contains("shape_detection_epsilon"))
    shape_detection_epsilon->setValue(settings.value("shape_detection_epsilon").toDouble());
  if (settings.contains("shape_detection_cluster_epsilon"))
    shape_detection_cluster_epsilon->setValue(settings.value("shape_detection_cluster_epsilon").toDouble());
  if (settings.contains("shape_detection_normal_threshold"))
    shape_detection_normal_threshold->setValue(settings.value("shape_detection_normal_threshold").toDouble());

  if (settings.contains("hplane_detection_probability"))
    hplane_detection_probability->setValue(settings.value("hplane_detection_probability").toDouble());
  if (settings.contains("hplane_detection_min_points"))
    hplane_detection_min_points->setValue(settings.value("hplane_detection_min_points").toInt());
  if (settings.contains("hplane_detection_epsilon"))
    hplane_detection_epsilon->setValue(settings.value("hplane_detection_epsilon").toDouble());
  if (settings.contains("hplane_detection_cluster_epsilon"))
    hplane_detection_cluster_epsilon->setValue(settings.value("hplane_detection_cluster_epsilon").toDouble());
  if (settings.contains("hplane_detection_normal_threshold"))
    hplane_detection_normal_threshold->setValue(settings.value("hplane_detection_normal_threshold").toDouble());

  settings.endGroup();
}

void Settings_dialog::saveToSettings()
{
  QSettings settings("settings.ini", QSettings::IniFormat);
  settings.beginGroup("AlgVis");

  settings.setValue("shape_detection_min_points", shape_detection_min_points->value());
  settings.setValue("shape_detection_epsilon", shape_detection_epsilon->value());
  settings.setValue("shape_detection_normal_threshold", shape_detection_normal_threshold->value());
  settings.setValue("shape_detection_cluster_epsilon", shape_detection_cluster_epsilon->value());
  settings.setValue("shape_detection_probability", shape_detection_probability->value());

  settings.setValue("hplane_detection_min_points", hplane_detection_min_points->value());
  settings.setValue("hplane_detection_epsilon", hplane_detection_epsilon->value());
  settings.setValue("hplane_detection_normal_threshold", hplane_detection_normal_threshold->value());
  settings.setValue("hplane_detection_cluster_epsilon", hplane_detection_cluster_epsilon->value());
  settings.setValue("hplane_detection_probability", hplane_detection_probability->value());

  settings.endGroup();
}

void Settings_dialog::accept()
{
  saveToSettings();

  QDialog::accept();
}
