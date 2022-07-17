/**
 * @file /include/displayer_qt5/main_window.hpp
 *
 * @brief Qt based gui for displayer_qt5.
 *
 * @date November 2010
 **/
#ifndef displayer_qt5_MAIN_WINDOW_H
#define displayer_qt5_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QImage>
#include <QMutex>
#include <QListWidgetItem>
#include "qnode.hpp"

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

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLogcamera();
    void displayCamera(const QImage& image);
    void updateLoggingView();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QImage qimage_;
    mutable QMutex qimage_mutex_;
};

}  // namespace displayer_qt5

#endif // displayer_qt5_MAIN_WINDOW_H
