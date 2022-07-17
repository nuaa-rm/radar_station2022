/**
 * @file /include/displayer_qt5/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef displayer_qt5_QNODE_HPP_
#define displayer_qt5_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <QImage>

#include <QListWidgetItem>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace displayer_qt5 {

/*****************************************************************************
** Class
*****************************************************************************/
enum LogLevel {
         Debug,
         Info,
         Warn,
         Error,
         Fatal
 };
struct log_information{
         LogLevel level;
         QString qstring;
};

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
    void imgShowCallback(const sensor_msgs::ImageConstPtr& msg);//camera callback function
    QImage image;
    QListWidgetItem *listWidgetItem;
    log_information *logInformation;
	/*********************
	** Logging
	**********************/


	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    void loggingCamera();//发出设置相机图片信号

private:
	int init_argc;
	char** init_argv;
    image_transport::Subscriber image_sub;
    cv::Mat img;
};

}  // namespace displayer_qt5

#endif /* displayer_qt5_QNODE_HPP_ */
