#ifndef displayer_qt5_QOPENGLWIDGET_WITH_IMAGE_SHOW_H
#define displayer_qt5_QOPENGLWIDGET_WITH_IMAGE_SHOW_H

#include <QMainWindow>
#include <QObject>
#include <QWidget>
#include <QOpenGLWidget>
#include <GL/gl.h>
#include <GL/glu.h>
#include <QOpenGLFunctions>

class QOpenGlWidget_with_image_show : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit QOpenGlWidget_with_image_show(QWidget *parent = nullptr);
    ~QOpenGlWidget_with_image_show();
protected:
    void initializeGL() Q_DECL_OVERRIDE;
    void resizeGL(int w, int h) Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;
private:

};

#endif // QOPENGLWIDGET_WITH_IMAGE_SHOW_H
