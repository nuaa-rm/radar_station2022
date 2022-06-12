#ifndef PROJECT_LIB_H
#define PROJECT_LIB_H

#include <eigen3/Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef Eigen::Matrix<float, 3, 4> Mat34;
typedef Eigen::Matrix<float, 3, 3> Mat33;
typedef Eigen::Vector3f vec3;
typedef Eigen::Vector4f vec4;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcp;

void projectPoints(const pcp& input, const Mat33& cam_matrix, const Mat34& uni_matrix, int cols, int rows, float* output);
void projectInit();
void deInit();

#endif
