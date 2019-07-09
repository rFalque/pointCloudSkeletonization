/*
 * implementation of the one ring neighbors
 * by R. Falque
 * 27/06/2019
 */

#ifndef POINT_RING_HPP
#define POINT_RING_HPP

#include <Eigen/Core>
#include <Eigen/SVD>
#include <vector>
#include <algorithm>
#include "delaunator.hpp"

struct OneRingPoint {
    int id;
    Eigen::Vector3d location;
    std::vector < Eigen::Vector3d > connected_points;
    std::vector < int > connected_points_id;
};

/*
 *TODO:
 *     - add tests
 *     - what to do if the neighbours are not connected?
 *     - sort one_ring_?
 *     - add the LB operator (no?)
 *     - remove the allocation of the Eigen::Vector3d
 */

class OneRing
{
public:

	OneRing(int vertex_id, std::vector <int> neighbours_id, Eigen::MatrixXd cloud);

	~OneRing(){
	}

	// void LB();
    void print();

    // information about the one_ring_
    bool is_cycle_ = false;
    bool is_connected_ = false;

    std::vector< OneRingPoint > get_one_ring();

private:
    int vertex_id_;
    std::vector <int> neighbours_id_;
    int neighbours_number_;

    Eigen::Vector3d vertex_;
    Eigen::MatrixXd neighbours_;
    std::vector< OneRingPoint > one_ring_;

    Eigen::MatrixXd svd_projection(Eigen::MatrixXd & points);
    Eigen::MatrixXi delaunay_triangulation_2d(Eigen::MatrixXd & points);
    void find_one_ring(Eigen::MatrixXi & triangles);
};

OneRing::OneRing(int vertex_id, std::vector <int> neighbours_id, Eigen::MatrixXd cloud) {
    vertex_id_ = vertex_id;
    neighbours_id_ = neighbours_id;
    neighbours_number_ = neighbours_id_.size();

    // set up vertex and neighbours as eigen structures
    vertex_ = cloud.row(vertex_id_);
    neighbours_.resize(neighbours_number_, 3);
    for (int i=0; i<neighbours_number_; i++) {
        neighbours_.row(i) = cloud.row(neighbours_id_.at(i));
    }

    // join all points
    Eigen::MatrixXd all_points(1+neighbours_.rows(), 3);
    all_points << vertex_.transpose(), neighbours_;

    // project the points onto a 2D plane
    Eigen::MatrixXd planar_projection;
    planar_projection = svd_projection(all_points);

    // construct the delaunay triangulation
    Eigen::MatrixXi triangles;
    triangles = delaunay_triangulation_2d(planar_projection);

    // filter the triangles to get the one ring neihbours (and their connected components)
    find_one_ring(triangles);
};

inline Eigen::MatrixXd OneRing::svd_projection(Eigen::MatrixXd & points) {
    // get the single value decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(points, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd coeffs = svd.matrixV();
    Eigen::MatrixXd planar_projection(points.rows(), 2);

    // projection on the two main components
    planar_projection.col(0) = points * coeffs.col(0);
    planar_projection.col(1) = points * coeffs.col(1);

    return planar_projection;
};

inline Eigen::MatrixXi OneRing::delaunay_triangulation_2d(Eigen::MatrixXd & points) {
    if (points.cols() != 2) {
        std::cout << "the dimension of the input are incorrect: it should be 2" << std::endl;
    }

    // std formating
    std::vector< double > std_points;
    for (int i=0; i<points.rows(); i++) {
        std_points.push_back(points(i, 0));
        std_points.push_back(points(i, 1));
    }

    delaunator::Delaunator d(std_points);

    // eigen formating
    Eigen::MatrixXi triangles(d.triangles.size()/3, 3);
    for(int i = 0; i < d.triangles.size(); i+=3) {
        triangles(int(i/3), 0) = int(d.triangles[i]);
        triangles(int(i/3), 1) = int(d.triangles[i+1]);
        triangles(int(i/3), 2) = int(d.triangles[i+2]);
    }

    return triangles;
};

inline void OneRing::find_one_ring(Eigen::MatrixXi & triangles) {

    bool add_point_one;
    bool add_point_two;

    // circulate through each triangle
    for (int i=0; i<triangles.rows(); i++) {

        // copy the triangle for sorting
        Eigen::Vector3i triangle = triangles.row(i);
        std::sort(triangle.data(), triangle.data()+triangle.size());

        // check if the triangle is connected to the first point
        if (triangle(0) == 0)
        {
            add_point_one = true;
            add_point_two = true;

            triangle(1) --; // this is due to line 63 in the same file
            triangle(2) --;
            
            // if one of the point is already in the one ring list
            for (int j = 0; j<one_ring_.size(); j++){
                if (one_ring_[j].id == neighbours_id_.at(triangle(1))) {
                    add_point_one = false;
                    one_ring_[j].connected_points.push_back(neighbours_.row(triangle(2)));
                    one_ring_[j].connected_points_id.push_back(neighbours_id_.at(triangle(2) )) ;
                }
                if (one_ring_[j].id == neighbours_id_.at(triangle(2))) {
                    add_point_two = false;
                    one_ring_[j].connected_points.push_back(neighbours_.row(triangle(1)));
                    one_ring_[j].connected_points_id.push_back(neighbours_id_.at(triangle(1)));
                }
            }

            if (add_point_one) {
                OneRingPoint point;
                point.id = neighbours_id_.at(triangle(1));
                point.location = neighbours_.row(triangle(1));
                point.connected_points.push_back(neighbours_.row(triangle(2)));
                point.connected_points_id.push_back(neighbours_id_.at(triangle(2) ));

                one_ring_.push_back(point);
            }

            if (add_point_two) {
                OneRingPoint point;
                point.id = neighbours_id_.at((triangle(2)));
                point.location = neighbours_.row(triangle(2));
                point.connected_points.push_back(neighbours_.row(triangle(1)));
                point.connected_points_id.push_back(neighbours_id_.at(triangle(1)));

                one_ring_.push_back(point);
            }
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
     *  - if there are more than two unique elements, then the one_ring_ would be disconnected
     */

    int non_double_occurences = 0;
    for (std::vector<OneRingPoint>::iterator it = one_ring_.begin() ; it != one_ring_.end(); ++it) {
        if (it->connected_points.size() != 2)
        {
            non_double_occurences++;
        }
    }

    // set up the flags
    if (non_double_occurences == 0) {
        is_cycle_ = true;
        is_connected_ = true;
    } else if (non_double_occurences == 2) {
        is_cycle_ = false;
        is_connected_ = true;
    } else {
        // this is a shit scenario which should not happen
        is_cycle_ = false;
        is_connected_ = false;
    }

};

inline std::vector< OneRingPoint > OneRing::get_one_ring() {
    return one_ring_;
};

inline void OneRing::print() {
    std::cout << "size of the one ring: " << one_ring_.size() << "\n";
    for (int i=0; i<one_ring_.size(); i++)
        std::cout << "neighbour with id " << one_ring_.at(i).id << " located at : " << one_ring_.at(i).location.transpose() << "\n";
};

#endif
