/**
 * @file /include/displayer_qt5/main_window.hpp
 *
 * @brief Qt based gui for displayer_qt5.
 *
 * @date November 2010
 **/
#ifndef displayer_qt5_MAIN_WINDOW_HPP_
#define displayer_qt5_MAIN_WINDOW_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QImage>
#include <QMutex>
#include <QListWidgetItem>
#include <chrono>
#include <QTimer>
#include <QTime>
#include <QApplication>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace displayer_qt5 {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void initUI();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLogcamera();
    void updateLogcameraSecondWindow();
    void updateLogcameraCalibrateMainWindow();
    void updateLogcameraCalibrateSecondWindow();
    void displayCamera(const QImage& image);
    void displayCameraCalibrateMainWindow(const QImage& image);
    void displayCameraCalibrateSecondWindow(const QImage& image);
    void displayCameraSecondWindow(const QImage& image);
    void updateLoggingView();
    void updateGameState();

private slots:
    void on_comboBoxCalibrateCamera_currentIndexChanged(const QString &arg1);
    void on_labelCalibrateCameraMainWindow_mouseLocationChanged();
    void on_tabWidget_currentChanged(int index);
    void on_pushButtonCalibrate_clicked();
    void updateSmallMap();
    void on_timer_timeout();

private:
	Ui::MainWindowDesign ui;
    QNode qnode;
    QImage qimage_;
    QImage qimage_calibrate_main_window_;
    QImage qimage_calibrate_second_window_;
    QImage qimage_second_window_;
    mutable QMutex qimage_mutex_;
    QTimer *fTimer;
};

}  // namespace displayer_qt5

#endif // displayer_qt5_MAIN_WINDOW_H