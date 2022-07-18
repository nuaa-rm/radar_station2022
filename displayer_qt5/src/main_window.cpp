/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/displayer_qt5/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace displayer_qt5 {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(&qnode,SIGNAL(loggingCamera()),this,SLOT(updateLogcamera()));
    QObject::connect(&qnode,SIGNAL(loggingCameraCalibrateMainWindow()),this,SLOT(updateLogcameraCalibrateMainWindow()));
    QObject::connect(&qnode,SIGNAL(loggingCameraCalibrateSecondWindow()),this,SLOT(updateLogcameraCalibrateSecondWindow()));
    setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(ui.labelCalibrateCameraMainWindow, SIGNAL(mouseMovePoint(QPoint)), this, SLOT(on_labelCalibrateCameraMainWindow_mouseLocationChanged()));
    if ( !qnode.init() ) {
        showNoMasterMessage();
    }
    initUI();

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::initUI()
{
    ui.comboBoxCalibrateCamera->clear();
    QStringList stringList;
    stringList << qnode.sensorFarImgRaw << qnode.sensorCloseImgRaw;
    ui.comboBoxCalibrateCamera->addItems(stringList);

    ui.tabWidget->setCurrentIndex(0);

    ui.labelCalibrateCameraMainWindow->sensor_far_points = qnode.sensor_far_points;
    ui.labelCalibrateCameraMainWindow->sensor_close_points = qnode.sensor_close_points;

    ui.labelCalibrateCameraMainWindow->sensorFarImgRaw = qnode.sensorFarImgRaw;
    ui.labelCalibrateCameraMainWindow->sensorCloseImgRaw = qnode.sensorCloseImgRaw;
}

void MainWindow::updateLogcamera()
{
    displayCamera(qnode.image);
}

void MainWindow::updateLogcameraCalibrateMainWindow()
{
    displayCameraCalibrateMainWindow(qnode.imageCalibrateMainWindow);
}

void MainWindow::updateLogcameraCalibrateSecondWindow()
{
    cv::Rect r;
    r.width = qnode.calibrateSecondWindowWidth / qnode.calibrateRate;
    r.height = qnode.calibrateSecondWindowHeight / qnode.calibrateRate;
    int halfWidth = (qnode.calibrateSecondWindowWidth * 0.5) / qnode.calibrateRate;
    int halfHeight = (qnode.calibrateSecondWindowHeight * 0.5) / qnode.calibrateRate;
    if(ui.labelCalibrateCameraMainWindow->selectedPoint.x() > qnode.calibrateMainWindowWidth)
    {
        ui.labelCalibrateCameraMainWindow->selectedPoint.setX(qnode.calibrateMainWindowWidth);
    }
    if(ui.labelCalibrateCameraMainWindow->selectedPoint.y() > qnode.calibrateMainWindowHeight)
    {
        ui.labelCalibrateCameraMainWindow->selectedPoint.setY(qnode.calibrateMainWindowHeight);
    }
    cv::Mat m;
    if(ui.labelCalibrateCameraMainWindow->selectedPoint.x() - halfWidth < 0)
    {
        r.x = 0;
    }
    else if((ui.labelCalibrateCameraMainWindow->selectedPoint.x() + halfWidth) > (qnode.calibrateMainWindowWidth))
    {
        r.x = qnode.calibrateMainWindowWidth - qnode.calibrateSecondWindowWidth / qnode.calibrateRate;
    }
    else
    {
        r.x = ui.labelCalibrateCameraMainWindow->selectedPoint.x() - halfWidth;
    }

    if(ui.labelCalibrateCameraMainWindow->selectedPoint.y() - halfHeight < 0)
    {
        r.y = 0;
    }
    else if((ui.labelCalibrateCameraMainWindow->selectedPoint.y() + halfHeight) > (qnode.calibrateMainWindowHeight))
    {
        r.y = qnode.calibrateMainWindowHeight - qnode.calibrateSecondWindowHeight / qnode.calibrateRate;
    }
    else
    {
        r.y = ui.labelCalibrateCameraMainWindow->selectedPoint.y() - halfHeight;
    }
    if (r.x < 0)
    {
        r.x = 0;
    }
    if (r.y < 0)
    {
        r.y = 0;
    }
    if ((r.x + r.width) > qnode.calibrateMainWindowWidth)
    {
        r.width = qnode.calibrateMainWindowWidth - r.x;
    }
    if ((r.y + r.height) > qnode.calibrateMainWindowWidth)
    {
        r.height = qnode.calibrateMainWindowWidth - r.y;
    }
    if(qnode.cameraCelibrating == qnode.sensorFarImgRaw)
    {
        qnode.imgSensorFar(r).copyTo(m);
    }
    else
    {
        qnode.imgSensorClose(r).copyTo(m);
    }
    cv::resize(m, m, cv::Size(qnode.calibrateSecondWindowWidth, qnode.calibrateSecondWindowHeight));
    cv::line(m, cv::Point(0, (ui.labelCalibrateCameraMainWindow->selectedPoint.y() - r.y) * qnode.calibrateRate), cv::Point(qnode.calibrateSecondWindowWidth, (ui.labelCalibrateCameraMainWindow->selectedPoint.y() - r.y) * qnode.calibrateRate), cv::Scalar(255, 255, 255));
    cv::line(m, cv::Point((ui.labelCalibrateCameraMainWindow->selectedPoint.x() - r.x) * qnode.calibrateRate, 0), cv::Point((ui.labelCalibrateCameraMainWindow->selectedPoint.x() - r.x) * qnode.calibrateRate, qnode.calibrateSecondWindowHeight), cv::Scalar(255, 255, 255));
    qnode.imageCalibrateSecondWindow = QImage(m.data,m.cols,m.rows,m.step[0],QImage::Format_RGB888);
    displayCameraCalibrateSecondWindow(qnode.imageCalibrateSecondWindow);
}

void MainWindow::displayCamera(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_ = image.copy();
    ui.label_camera->setPixmap(QPixmap::fromImage(qimage_));
    ui.label_camera->resize(ui.label_camera->pixmap()->size());
    qimage_mutex_.unlock();

}

void MainWindow::displayCameraCalibrateMainWindow(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_calibrate_main_window_ = image.copy();
    ui.labelCalibrateCameraMainWindow->setPixmap(QPixmap::fromImage(qimage_calibrate_main_window_));
    ui.labelCalibrateCameraMainWindow->resize(ui.labelCalibrateCameraMainWindow->pixmap()->size());
    qimage_mutex_.unlock();
}

void MainWindow::displayCameraCalibrateSecondWindow(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_calibrate_second_window_ = image.copy();
    ui.labelCalibrateCameraSecondWindow->setPixmap(QPixmap::fromImage(qimage_calibrate_second_window_));
    ui.labelCalibrateCameraSecondWindow->resize(ui.labelCalibrateCameraSecondWindow->pixmap()->size());
    qimage_mutex_.unlock();
}

void MainWindow::updateLoggingView() {
    QListWidgetItem *item = new QListWidgetItem(ui.view_logging);
    item->setText(qnode.logInformation->qstring);
    QFont font = item->font();
    switch (qnode.logInformation->level) {
        case(Debug) : {
                font.setPointSize(15);
                font.setBold(false);
                item->setFont(font);
                item->setTextColor(Qt::gray);
                break;
        }
        case(Info) : {
                font.setPointSize(18);
                font.setBold(false);
                item->setFont(font);
                item->setTextColor(Qt::black);
                break;
        }
        case(Warn) : {
                font.setPointSize(20);
                font.setBold(false);
                item->setFont(font);
                item->setTextColor(Qt::darkYellow);
                break;
        }
        case(Error) : {
                font.setPointSize(23);
                font.setBold(true);
                item->setFont(font);
                item->setTextColor(Qt::red);
                break;
        }
        case(Fatal) : {
                font.setPointSize(28);
                font.setBold(true);
                font.setItalic(true);
                item->setFont(font);
                item->setTextColor(Qt::darkRed);
                break;
        }
    }
    item->setFont(font);
    ui.view_logging->addItem(item);
    ui.view_logging->scrollToBottom();
}


void MainWindow::closeEvent(QCloseEvent *event)
{

	QMainWindow::closeEvent(event);
}

}




void displayer_qt5::MainWindow::on_comboBoxCalibrateCamera_currentIndexChanged(const QString &arg1)
{
    qnode.cameraCelibrating = arg1;
    ui.labelCalibrateCameraMainWindow->cameraCelibrating = arg1;
}

void displayer_qt5::MainWindow::on_labelCalibrateCameraMainWindow_mouseLocationChanged()
{
    qnode.mouseLoaction = ui.labelCalibrateCameraMainWindow->selectedPoint;
    emit qnode.loggingCameraCalibrateSecondWindow();
}

void displayer_qt5::MainWindow::on_tabWidget_currentChanged(int index)
{
    if(index == 1)
    {
        qnode.if_is_celibrating = true;
    }
    else
    {
        qnode.if_is_celibrating = false;
    }
}

void displayer_qt5::MainWindow::on_pushButtonCalibrate_clicked()
{
    qnode.pubCelibrateResult();
    if(ui.comboBoxCalibrateCamera->currentIndex() == 1)
    {
        ui.comboBoxCalibrateCamera->setCurrentIndex(0);
    }
    else
    {
        ui.comboBoxCalibrateCamera->setCurrentIndex(1);
    }
}
