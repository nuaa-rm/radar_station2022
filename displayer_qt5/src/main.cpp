/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/displayer_qt5/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    displayer_qt5::MainWindow w(argc,argv);
    w.show();
    int result = app.exec();
    std::cout << w.size().width() << '\t' << w.size().height() << std::endl;
	return result;
}
