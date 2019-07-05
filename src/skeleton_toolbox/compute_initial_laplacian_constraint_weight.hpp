/*
 * ????
 * by R. Falque
 * 27/06/2019
 */

#ifndef COMPUTE_INITIAL_LAPLACIAN_WEIGHT_HPP
#define COMPUTE_INITIAL_LAPLACIAN_WEIGHT_HPP

#include <Eigen/Core>
#include "point_ring.hpp"

// return for each point the mean distance to its neighbours
inline Eigen::VectorXd one_ring_size(Eigen::MatrixXd & cloud, std::vector< OneRing > & one_ring_list) {

    // vector to return
    Eigen::VectorXd out(cloud.rows());

    for (int i=0; i<cloud.rows(); i++) {
        // get the point:
        Eigen::Vector3d vertex;
        vertex = cloud.row(i);

        // get the corresponding one ring structure
        std::vector<int> one_ring = one_ring_list.at(i).get_one_ring();

        // store all the distances between the vertex and its neighbours
        Eigen::VectorXd temp(one_ring.size());
        for (int neighbour_it=0; neighbour_it<one_ring.size(); neighbour_it++) {
            Eigen::Vector3d neighbour;
            neighbour = cloud.row(one_ring.at(neighbour_it));
            temp(neighbour_it) = ( vertex - neighbour ).norm();
        }

        // get the mean of all the distance to the point
        out(i) = temp.mean();
    }

    return out;
};

inline double compute_initial_laplacian_weight(Eigen::MatrixXd & cloud, std::vector< OneRing > & one_ring_list) {
    Eigen::VectorXd ms;
    double wl = 0;

    ms = one_ring_size(cloud, one_ring_list);
    wl = 1.0/(5.0*ms.mean());

    return wl;
};

#endif
