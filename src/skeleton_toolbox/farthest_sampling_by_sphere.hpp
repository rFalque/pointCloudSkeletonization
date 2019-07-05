/*
*   farthest_sampling_by_sphere
*   by R. Falque
*   29/11/2018
*/

#ifndef NORMALIZATION_HPP
#define NORMALIZATION_HPP

#include <Eigen/Core>
#include <limits> 
#include <iostream>

inline bool normalization(Eigen::MatrixXd & in_cloud){
    // centre in zero
    in_cloud.rowwise() -= (in_cloud.colwise().minCoeff() + in_cloud.colwise().maxCoeff()) / 2;

    // make the diagonal 1.6
    in_cloud *= 1.6/(in_cloud.colwise().maxCoeff() - in_cloud.colwise().minCoeff()).maxCoeff();

    return true;
};

#endif
