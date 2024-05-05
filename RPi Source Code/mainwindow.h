#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "GPSDriver.h"
#include "bmp280driver.h"
#include <fstream>
#include <iostream>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    int serial_fd;
    int fd;
    std::ofstream datafile;

private slots:
    void updateUI();
    void updateData();

private:
    Ui::MainWindow *ui;
    QTimer *timer;
    QTimer *datatimer;
    bmp280driver *bmpdriver;
    GPSDriver *gpsdriver;
    double altitude; // Altitude in meters
    double speed; // Speed in mph
    double groundAlt;

};
#endif // MAINWINDOW_H
