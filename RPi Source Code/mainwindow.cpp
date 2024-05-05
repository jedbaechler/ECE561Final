#include "mainwindow.h"
#include "./ui_mainwindow.h"
// #include "driver.h"
#include "GPSDriver.h"
#include "bmp280driver.h"
#include <iostream>
#include <fstream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , altitude(13000.0) // initial altitude value
    , speed(0) // initital speed value //
    , bmpdriver(new bmp280driver())
    , gpsdriver(new GPSDriver())
{
    ui->setupUi(this);
    datafile.open("/home/jbawesome099/Documents/test_data.csv");
    const char *serial_port = "/dev/ttyS0";
    gpsdriver->configGPS(serial_port);
    serial_fd = gpsdriver->initGPS(serial_port);
    bmpdriver->init_bmp();
    fd = bmpdriver->fd;
    groundAlt = bmpdriver->readAltitude(fd);
    timer = new QTimer(this);
    datatimer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateUI);
    connect(datatimer, &QTimer::timeout, this, &MainWindow::updateData);
    timer->start(1000); // update the UI every 1000 milliseconds
}

MainWindow::~MainWindow()
{
    delete bmpdriver;
    delete gpsdriver;
    delete ui;
    delete timer;
    delete datatimer;
    datafile.close();
}

void MainWindow::updateData(){

}

void MainWindow::updateUI()
{
    altitude = bmpdriver->readAltitude(fd)-groundAlt;
//    altitude += 1;
    speed = gpsdriver->getGPS(serial_fd);
    if (!datafile.is_open()) {
        printf("Unable to open data csv");
    }
    datafile << altitude << ", " << speed << " \n";

    // Update the UI elements with the fixed values
    ui->AltitudeMeasurement->setText(QString::number(altitude, 'f', 2));
    // speed = gpsDriver->parseSpeed(nmeaData);
    // qDebug() << "Parsed Speed:" << speed;
    ui->SpeedMeasurement->setText(QString::number(speed, 'f', 2));
    //ui->AltitudeMeasurement->setGeometry(910,10,461,351);
    //ui->SpeedMeasurement->setGeometry(910,400,461,381);

    // Test increment


}
