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

  if (settings.contains("cplane_detection_probability"))
    cplane_detection_probability->setValue(settings.value("cplane_detection_probability").toDouble());
  if (settings.contains("cplane_detection_min_points"))
    cplane_detection_min_points->setValue(settings.value("cplane_detection_min_points").toInt());
  if (settings.contains("cplane_detection_epsilon"))
    cplane_detection_epsilon->setValue(settings.value("cplane_detection_epsilon").toDouble());
  if (settings.contains("cplane_detection_cluster_epsilon"))
    cplane_detection_cluster_epsilon->setValue(settings.value("cplane_detection_cluster_epsilon").toDouble());
  if (settings.contains("cplane_detection_normal_threshold"))
    cplane_detection_normal_threshold->setValue(settings.value("cplane_detection_normal_threshold").toDouble());

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

  settings.setValue("cplane_detection_min_points", cplane_detection_min_points->value());
  settings.setValue("cplane_detection_epsilon", cplane_detection_epsilon->value());
  settings.setValue("cplane_detection_normal_threshold", cplane_detection_normal_threshold->value());
  settings.setValue("cplane_detection_cluster_epsilon", cplane_detection_cluster_epsilon->value());
  settings.setValue("cplane_detection_probability", cplane_detection_probability->value());

  settings.endGroup();
}

void Settings_dialog::accept()
{
  saveToSettings();

  QDialog::accept();
}
