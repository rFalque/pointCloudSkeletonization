/*
 * implementation of the one ring neighbors
 * by R. Falque
 * 27/06/2019
 */

#ifndef COMPUTE_POINT_RING_HPP
#define COMPUTE_POINT_RING_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "../mesh_tools/nanoflannWrapper.hpp"

#include <igl/delaunay_triangulation.h>


/*
 *TODO:
 *     - merge the two for loop
 *     - remove the mulitple usage of i
 *     - check the code for bugs
 */
inline Eigen::MatrixXd compute_point_ring(Eigen::MatrixXd & cloud, unsigned int k) {

    // compute k nearest neighbours for each point in the cloud
    nanoflann_wrapper knn_search(cloud);
    Eigen::MatrixXd unsorted_neighbours_index(cloud.rows(), k);


    std::vector <int> temp;
    for (int i=0; i<cloud.rows(); i++) {
        temp = knn_search.return_k_closest_points(cloud.row(i), k);
        unsorted_neighbours_index.row(i) = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(temp.data(), temp.size());
    }

    // line 55
    Eigen::MatrixXd unsorted_neighbours(k+1, 3);
    for (int i=0; i<cloud.rows(); i++) {


        // get the neighbours as 3D points
        unsorted_neighbours(0, 3) = cloud.row(i);
        for (int j=0; j<k; j++) {
            unsorted_neighbours.row(j+1) = cloud.row(unsorted_neighbours_index(i,j));
        }
        
        // projection on a tangent plane using the principal component
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(unsorted_neighbours, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXd coeffs = svd.matrixU();
        Eigen::MatrixXd planar_projection(k, 2);
        planar_projection.col(0) = unsorted_neighbours * coeffs.col(0);
        planar_projection.col(1) = unsorted_neighbours * coeffs.col(1);

        // construct the delaunay triangulation
        Eigen::MatrixXi triangles;
        triangles = delaunay_triangulation_2d(planar_projection);

        // find the one ring neighbours
        std::vector <int> one_ring_triangles;
        int one_ring_size = 0;
        for (int i=0; i<triangles.rows(); i++) {
            if (triangles(i,0) == 0 || triangles(i,1) == 0 || triangles(i,2) == 0) {
                one_ring_triangles.push_back(i);
                one_ring_size ++;
            }
        }

        // get the one ring pairs
        Eigen::MatrixXi one_ring_pairs(one_ring_size, 2);
        for (int i=0; i<one_ring_size; i++)
        {
            if (triangles(one_ring_triangles[i], 0) == 0) {
                one_ring_pairs(i, 0) = triangles(one_ring_triangles[i], 1);
                one_ring_pairs(i, 1) = triangles(one_ring_triangles[i], 2);
            } else if (triangles(one_ring_triangles[i], 1) == 0) {
                one_ring_pairs(i, 0) = triangles(one_ring_triangles[i], 0);
                one_ring_pairs(i, 1) = triangles(one_ring_triangles[i], 2);
            } else if (triangles(one_ring_triangles[i], 2) == 0) {
                one_ring_pairs(i, 0) = triangles(one_ring_triangles[i], 0);
                one_ring_pairs(i, 1) = triangles(one_ring_triangles[i], 1);
            } else {
                std::cout << "wtf" << std::endl;
            }
        }

        /* To compute the laplacian operator, we need to compute the cotangent weights for each edge.
         * E.g., with the two triangles (abd) and (acd), for the edge (ad) we want the neighbours (c) and (b).
         * 
         *    a---b
         *    | \ |
         *    c---d
         * 
         * To find two neighbours for each edge, a closed cycle for the one-ring is needed.
         * This can be verified by checking if all the elements in the one_ring_pairs are present twice:
         *  - if is not the case, then it is not possible to create a one ring defined as a cycle
         *  - if there are two unique elements, it is possible to create a strip of neighbours
         *  - if there are more than two unique elements, then the 
         */ 

        // circulating through the triangles
        std::vector <int> one_ring_neighbours;
        one_ring_neighbours.push_back(one_ring_pairs(0, 0));
        one_ring_neighbours.push_back(one_ring_pairs(0, 1));
        int current_neighbour = one_ring_pairs(0, 1);
        one_ring_triangles.erase( one_ring_triangles.begin() );
        while (one_ring_triangles.size() != 0) {
            bool found_next = false;
            for (int i=0; one_ring_triangles.size(); i++){
                if (one_ring_pairs(one_ring_triangles[i], 0)==current_neighbour) {
                    current_neighbour = one_ring_pairs(one_ring_triangles[i], 1);
                    one_ring_neighbours.push_back(current_neighbour);
                    one_ring_triangles.erase( one_ring_triangles.begin()+i );
                    found_next = true;
                    break;
                } else if (one_ring_pairs(one_ring_triangles[i], 1)==current_neighbour) {
                    current_neighbour = one_ring_pairs(one_ring_triangles[i], 0);
                    one_ring_neighbours.push_back(current_neighbour);
                    one_ring_triangles.erase( one_ring_triangles.begin()+i );
                    found_next = true;
                    break;
                }
            }

            if (found_next == false)
            {
                std::cout << "Warning: one ring circulation not possible";
                break;
            }
        }

        // return the original points from the cloud
        std::vector <int> one_ring;
        for (int i=0; i<one_ring_neighbours.size(); i++) {
            one_ring.push_back();
        }


        // end
        return ;
    }
    


    return unsorted_neighbours;
};

#endif
