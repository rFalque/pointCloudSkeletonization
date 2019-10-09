/*
 * for k points, this class find the one ring neighbors
 * 
 * by R. Falque
 * 27/06/2019
 */

#ifndef POINT_RING_HPP
#define POINT_RING_HPP

#include <Eigen/Core>
#include <Eigen/SVD>
#include <vector>
#include <algorithm>

/*
 *TODO:
 *     - add tests
 *     - what happen if the neighbours are not connected?
 *     - sort one_ring_?  Is seems to be naturally sorted
 *     - add the LB operator (no?)
 */

class OneRingMesh
{
public:

	OneRingMesh(int vertex_id, Eigen::MatrixXd vertices, Eigen::MatrixXi faces);

	~OneRingMesh(){
	}

	// void LB();
    void print();

    // information about the one_ring_
    bool is_cycle_ = false;
    bool is_connected_ = false;

    std::vector< int > get_one_ring();
    std::vector< int > get_connected_components(int i);

private:
    // input of the class
    int vertex_id_;
    std::vector <int> neighbours_id_;

    // output of the class (see public functions for access)
    std::vector <int> one_ring_;
    std::vector < std::vector <int> >one_ring_connected_componnents_;
    int neighbours_number_;

    void find_one_ring(Eigen::MatrixXi & triangles);
};


OneRingMesh::OneRingMesh(int vertex_id, Eigen::MatrixXd vertices, Eigen::MatrixXi faces) {
    // set internal variables
    vertex_id_ = vertex_id;

    // set up vertex and neighbours as eigen structures
    Eigen::Vector3d vertex = vertices.row(vertex_id_);

    // find faces linked to the vertice
    Eigen::VectorXi col0, col1, col2;
    col0 = find(faces.col(0), vertex_id);
    col1 = find(faces.col(1), vertex_id);
    col2 = find(faces.col(2), vertex_id);

    Eigen::VectorXi one_ring_faces;
    one_ring_faces = col0;
    one_ring_faces = concatenate(one_ring_faces, col1, 1); // here check the direction
    one_ring_faces = concatenate(one_ring_faces, col2, 1); // here check the direction

    std::vector<std::vector <int>> one_ring_faces_to_travers;
    

    // go through the list and find the one_ring_vertices
    std::vector<int> one_ring_vertex;
    one_ring_vertex.push_back()



    // filter the triangles to get the one ring neihbours (and their connected components)
};

inline std::vector< int > OneRingMesh::get_one_ring() {
    return one_ring_;
};

inline void OneRingMesh::print() {
    std::cout << "size of the one ring: " << one_ring_.size() << "\n";
    for (int i=0; i<one_ring_.size(); i++)
        std::cout << "neighbour with id " << one_ring_.at(i) << "\n";
};

#endif
