/*
*   edge_collapse_update
*   by R. Falque
*   27/06/2019
*/

#ifndef EDGE_COLLASPE_UPDATE_HPP
#define EDGE_COLLASPE_UPDATE_HPP

#include <Eigen/Core>
#include <iostream>

#include "../graph_lib/graphStructure.hpp"

inline bool edge_collapse_update(Eigen::MatrixXd & nodes, 
                                 Eigen::VectorXi & correspondences,
                                 Eigen::MatrixXi & A)
{
    Graph skeleton(nodes, A);
    skeleton.init();
    skeleton.make_1D_curve();
    
    return true;
};

#endif
