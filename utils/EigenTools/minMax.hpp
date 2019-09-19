/*
*   Greedy Search
*   by R. Falque
*   29/11/2018
*/

#ifndef EIGENMINMAX_HPP
#define EIGENMINMAX_HPP

#include <Eigen/Core>

static inline void getMinMax(Eigen::MatrixXd & in_cloud, Eigen::Vector3d & min_point, Eigen::Vector3d & max_point){
    max_point = in_cloud.colwise().maxCoeff();
    min_point = in_cloud.colwise().minCoeff();
};

inline void getScale(Eigen::MatrixXd in_cloud, double & scale){
    Eigen::Vector3d min_point;
    Eigen::Vector3d max_point;

    getMinMax(in_cloud, min_point, max_point);

    scale = (max_point - min_point).norm();
};

#endif
