#include "displayer_qt5/qopenglwidget_with_image_show.h"
#include <QPainter>
#include <QOpenGLWidget>
#include <iostream>
QOpenGlWidget_with_image_show::QOpenGlWidget_with_image_show(QWidget *parent) : QOpenGLWidget(parent)
{

}

QOpenGlWidget_with_image_show::~QOpenGlWidget_with_image_show()
{

}

//void QOpenGlWidget_with_image_show::paintEvent(QPaintEvent *event)
//{
//    QPainter painter;
//    painter.begin(this);
//    painter.drawImage(QPoint(0, 0), imageToShow);
//    painter.end();

//    imageData_ = nullptr;
//}


void QOpenGlWidget_with_image_show::initializeGL()
{
    initializeOpenGLFunctions();
        glClearColor(0,0,0,1);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHTING);
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
        glEnable(GL_COLOR_MATERIAL);
}

void QOpenGlWidget_with_image_show::resizeGL(int width, int height)
{
    glViewport(0,0,width,height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-width/2,width/2,-height/2,height/2,-1,1);
    glMatrixMode(GL_MODELVIEW);
}

void QOpenGlWidget_with_image_show::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glBegin(GL_TRIANGLES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(-5, -5, 0);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f( 5, -5, 0);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f( 0.0,  5, 0);
   glEnd();
}
