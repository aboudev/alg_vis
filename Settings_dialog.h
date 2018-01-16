#ifndef SETTINGS_DIALOG_H
#define SETTINGS_DIALOG_H

#include <QtGui>
#include <QtWidgets>
#include "ui_Settings_dialog.h"

class Settings_dialog :
  public QDialog,
  public Ui_Settings_dialog
{
    Q_OBJECT

public:
    Settings_dialog(QWidget *parent = nullptr);
    void accept();

private slots:
    void loadFromSettings();
    void saveToSettings();

private:
};

#endif // SETTINGS_DIALOG_H
