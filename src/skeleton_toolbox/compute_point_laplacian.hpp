/*
 * Laplacian from pointcloud
 * 
 * by R. Falque
 * 27/06/2019
 */

#ifndef COMPUTE_POINT_LAPLACIAN_HPP
#define COMPUTE_POINT_LAPLACIAN_HPP

#include <Eigen/Core>
#include <Eigen/Sparse>
#include "point_ring.hpp"
#include <math.h>


/*
 * Definition of cotangent with respect to the dot and cross product 
 * see "The Geometry of the Dot and Cross Products" from Tevian Dray & Corinne A. Manogue
 * (there might be a better citation)
 * 
 * with v and w two vector and theta in between
 * |v x w| = |v|.|w|.sin(theta)
 *  v . w  = |v|.|w|.cos(theta)
 * cot(theta) = cos(theta) / sin(theta)
 *            = (v . w) / |v x w|
 */
inline double cotan(Eigen::Vector3d v, Eigen::Vector3d w) { 
    return( ( v.dot(w) ) / ( (v.cross(w)).norm() ) ); 
};

inline Eigen::SparseMatrix<double> compute_laplacian_weight(Eigen::MatrixXd & cloud, std::vector< OneRing > & one_ring_list) {
    //Eigen::MatrixXd L = Eigen::MatrixXd::Zero(cloud.rows(), cloud.rows());

    Eigen::SparseMatrix<double> L_2(cloud.rows(), cloud.rows());

    for (int i=0; i<cloud.rows(); i++) {
        /* The laplacian operator is defined with the cotangent weights for each edge.
         * E.g., with the two triangles (abd) and (acd), for the edge (ad) we sum cotan(b) and cotan(c).
         * 
         *    a---b
         *    | \ |
         *    c---d
         * 
         */

        Eigen::Vector3d a = cloud.row(i);

        std::vector<int> one_ring = one_ring_list[i].get_one_ring();
        for (int j=0; j<one_ring.size(); j++) {

            double cot_theta = 0;
            
            Eigen::Vector3d d = cloud.row(one_ring[j]);

            std::vector<int> connected_element = one_ring_list[i].get_connected_components(j);
            
            for (int connected_element_it=0; connected_element_it<connected_element.size(); connected_element_it++) {
                Eigen::Vector3d b = cloud.row(connected_element[connected_element_it]);

                Eigen::Vector3d ba = b-a;
                Eigen::Vector3d bd = b-d;

                cot_theta += cotan(ba, bd);
            }

            //cot_theta /= connected_element.size();
            //if (cot_theta>10000)
            //    cot_theta = 10000;
            
            L_2.coeffRef(i, one_ring[j]) += cot_theta;
            L_2.coeffRef(i, i) -= cot_theta;
        }
    }
    
    return L_2;
};

#endif
