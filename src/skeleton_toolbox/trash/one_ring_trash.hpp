/*
 * implementation of the one ring neighbors
 * by R. Falque
 * 27/06/2019
 */

#ifndef POINT_RING_HPP
#define POINT_RING_HPP


#include "delaunator.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <algorithm>

/*
 *TODO:
 *     - define the class
 *     - add tests
 *     - what to do if the neighbours are not connected?
 */
class OneRing
{
public:

	// provide the mesh and the graph
	OneRing(Eigen::Vector3d vertex, Eigen::MatrixXd neighbours);

	~OneRing(){
	}

	void LB();
    void print();

    // information about the one_ring
    bool is_cycle = false;
    bool is_connected = false;

private:
    Eigen::Vector3d vertex_;
    Eigen::MatrixXd neighbours_;

    std::vector< int > one_ring_;

    Eigen::MatrixXd svd_projection(Eigen::MatrixXd & points);
    Eigen::MatrixXi delaunay_triangulation_2d(Eigen::MatrixXd & points);

    void find_one_ring(Eigen::MatrixXi & triangles);

};

OneRing::OneRing(Eigen::Vector3d vertex, Eigen::MatrixXd neighbours) {
    // join all points
    Eigen::MatrixXd all_points(1+neighbours.rows(), 3);
    all_points << vertex.transpose(), neighbours;

    // project the points onto a 2D plane
    Eigen::MatrixXd planar_projection;
    planar_projection = svd_projection(all_points);

    // construct the delaunay triangulation
    Eigen::MatrixXi triangles;
    triangles = delaunay_triangulation_2d(planar_projection);

    find_one_ring(triangles);
};

inline Eigen::MatrixXd OneRing::svd_projection(Eigen::MatrixXd & points) {
    // get the single value decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(points, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd coeffs = svd.matrixU();
    Eigen::MatrixXd planar_projection(points.rows(), 2);

    // projection on the two main components
    planar_projection.col(0) = points * coeffs.col(0);
    planar_projection.col(1) = points * coeffs.col(1);

    return planar_projection;
};

inline Eigen::MatrixXi OneRing::delaunay_triangulation_2d(Eigen::MatrixXd & points) {
    if (points.cols() != 2) {
        std::cout << "the dimension of the input are incorrect" << std::endl;
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
    for(std::size_t i = 0; i < d.triangles.size(); i+=3) {
        triangles(i, 0) = d.triangles[i];
        triangles(i, 1) = d.triangles[i+1];
        triangles(i, 2) = d.triangles[i+2];
    }

    return triangles;
};

inline void OneRing::find_one_ring(Eigen::MatrixXi & triangles) {
    /* 
     * Other functions are clean wrappers of other implementations
     * Here, we are using dirty circulation through the data structures. 
     * This is probably sub-optimal.
     */

    // find the triangle connected to the first point
    std::vector <int> one_ring_triangles;
    std::vector <int> neighbours;
    int one_ring_size = 0;
    for (int i=0; i<triangles.rows(); i++) {
        Eigen::Vector3d triangle =  triangles.row(i);
        std::sort(triangle.data(), triangle.data()+triangle.size());
        if (triangle(0) == 0)
        {
            neighbours.push_back(triangle(1));
            neighbours.push_back(triangle(1));

        }




        if (triangles(i,0) == 0 || triangles(i,1) == 0 || triangles(i,2) == 0) {
            one_ring_triangles.push_back(i);
            one_ring_size ++;
        }
    }

    // get the one ring pairs (circulate through each triangles)
    Eigen::MatrixXi one_ring_pairs(one_ring_size, 2);
    for (int i=0; i<one_ring_size; i++)
    {
        Eigen::Vector3d triangle =  triangles.row(one_ring_triangles[i]);
        std::sort(triangle.data(), triangle.data()+triangle.size());

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
     *  - if there are more than two unique elements, then the one_ring would be disconnected
     */

    // list unique elements and occurences
    std::vector<int> all_elements(one_ring_pairs.data(), one_ring_pairs.data() + one_ring_pairs.rows() * one_ring_pairs.cols());
    std::vector<int> unique_elements = all_elements;
    std::sort( unique_elements.begin(), unique_elements.end() );
    unique_elements.erase( std::unique(unique_elements.begin(), unique_elements.end()) );
    int non_double_occurences = 0;
    std::vector<int> occurences;
    for (int i=0; i<unique_elements.size(); i++) {
        occurences.push_back(std::count( std::begin(all_elements), std::end(all_elements), unique_elements.at(i) ));
        if (occurences.at(i) != 2)
            non_double_occurences ++;
    }

    // set up the flags
    if (non_double_occurences == 0) {
        is_cycle = true;
        is_connected = true;
    } else if (non_double_occurences == 2) {
        is_cycle = false;
        is_connected = true;
    } else {
        // this is a shit scenario which should not happen
        is_cycle = false;
        is_connected = false;
    }




    /* FROM HERE, WORK IN PROGRESS */


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


};

inline void OneRing::print() {

};

#endif
