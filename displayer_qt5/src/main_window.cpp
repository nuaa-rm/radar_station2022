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
    setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    if ( !qnode.init() ) {
        showNoMasterMessage();
    }
    /*********************
    ** Auto Start
    **********************/

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

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */






void MainWindow::updateLogcamera()
{
    displayCamera(qnode.image);
}

void MainWindow::displayCamera(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_ = image.copy();
    ui.label_camera->setPixmap(QPixmap::fromImage(qimage_));
    ui.label_camera->resize(ui.label_camera->pixmap()->size());
    qimage_mutex_.unlock();

}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
    QListWidgetItem *item = new QListWidgetItem(ui.view_logging);
    item->setText(qnode.logInformation->qstring);
    QFont font = item->font();
    std::cout << qnode.logInformation->level <<std::endl;
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

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/



/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/





void MainWindow::closeEvent(QCloseEvent *event)
{

	QMainWindow::closeEvent(event);
}

}

