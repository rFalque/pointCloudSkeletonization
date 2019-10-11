/*
*   connect nodes according to their neighbours in the one ring set up
*   by R. Falque
*   27/06/2019
*/

#ifndef CONNECT_BY_INHERIT_NEIGH_HPP
#define CONNECT_BY_INHERIT_NEIGH_HPP

#include <Eigen/Core>
#include <limits> 
#include <iostream>

#include "EigenTools/nanoflannWrapper.hpp"

#include "point_ring.hpp"

inline bool connect_by_inherit_neigh(Eigen::MatrixXd & cloud, 
                                     Eigen::MatrixXd & nodes, 
                                     Eigen::VectorXi & correspondences, 
                                     std::vector< OneRing > & one_ring_list,
                                     Eigen::MatrixXi & A)
{
    A=Eigen::MatrixXi::Zero( nodes.rows(), nodes.rows() );
    for (int i=0; i<cloud.rows(); i++) {
        std::vector<int> one_ring = one_ring_list[i].get_one_ring();
        int point_node = correspondences(i);

        if (point_node == -1)
            std::cout << "Warning: some points have no correspondence"<<std::endl;
        else {
            for (int j=0; j<one_ring.size(); j++)
            {

                int neighbours_node = correspondences(one_ring.at(j));
                if (neighbours_node == -1)
                    std::cout << "Warning: some points have no correspondence"<<std::endl;
                else 
                {
                    if (point_node!=neighbours_node)
                    {
                        A(point_node, neighbours_node) ++;
                        A(neighbours_node, point_node) ++;
                        break;
                    }
                }
            }
        }
    }

    // if there are isolate points, connect it with its nearest neighbors.
    std::vector<int> is_isolated_point;
    for (int i=0; i<nodes.rows(); i++) {
        //
        if ( (A.row(i).array()==0).all() )
        {
            std::cout << "Warning: point " << i<< " is isolated."<<std::endl;
            is_isolated_point.push_back(i);
        }

        A(i,i) =1;
    }
    if (is_isolated_point.size()!=0)
    {
        nanoflann_wrapper knn_search(nodes);

        for (int i=0; i<is_isolated_point.size(); i++) {
            std::vector < int > closest_node;
            closest_node = knn_search.return_k_closest_points(nodes.row(is_isolated_point[i]), 2);
            A(is_isolated_point[i],closest_node[1]) ++;
            A(closest_node[1], is_isolated_point[i]) ++;
        }
    }

    return true;
};


#endif
