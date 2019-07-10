/**
 * Author: R. Falque
 * 
 * main for testing the skeleton extraction
 * by R. Falque
 * 27/06/2019
 **/

// dependencies

#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/writePLY.h>
#include <igl/opengl/glfw/Viewer.h>

#include "mesh_tools/plotMesh.hpp"
#include "mesh_tools/nanoflannWrapper.hpp"

#include "utils/loadCSV.hpp"
#include "utils/options.hpp"
#include "utils/EigenConcatenate.hpp"

#include <Eigen/Dense>

//#include "skeleton_toolbox/point_ring.hpp"
//#include "skeleton_toolbox/compute_initial_laplacian_constraint_weight.hpp"
//#include "skeleton_toolbox/compute_point_laplacian.hpp"
//#include "skeleton_toolbox/normalization.hpp"
//#include "skeleton_toolbox/farthest_sampling_by_sphere.hpp"
//#include "skeleton_toolbox/connect_by_inherit_neigh.hpp"
//#include "skeleton_toolbox/edge_collapse_update.hpp"

#include "skeleton_toolbox/pointSkeletonization.hpp"

#include <ctime>

#include <yaml-cpp/yaml.h>
#include <vector>
#include <algorithm>

#include<Eigen/SparseCholesky>



/*
 * List of things that could go wrong:
 *  - in the laplacian computation, line (63), ba instead of ab?
 *  - in the laplacian computation, line (66), division by number of neighbours needed?
 *  - in the point_ring, what happen if the one_ring is not connected?
 *  - does the laplacian need to be symmetric (see compute_point_laplacian.m line 31)
 *  - does the laplacian need to be normalized
 */


/* TODO:
 *  - add upper bound to avoid division by 0
 *  - add stopping criteria
 *  - test the radius search in the nanoflann wrapper
 *  - test without the normalization or inverse it afterwards
 */

int main(int argc, char* argv[])
{
    options opts;
    opts.loadYAML("../config.yaml");

    Eigen::MatrixXd V; // V: vertex of the surface
    Eigen::MatrixXi F; // F: faces of the surface

    std::string file_extension = opts.path_input_obj.substr(opts.path_input_obj.size() - 3);
    
    if (file_extension == "off")
        igl::readOFF(opts.path_input_obj, V, F);
    else if (file_extension == "ply")
        igl::readPLY(opts.path_input_obj, V, F);

    PointSkeletonization skeletonizer(V, F, opts);
    std::cout << "Progress: initialization:\n";
    skeletonizer.init();
    std::cout << "Progress: laplacian contraction:\n";
    skeletonizer.laplacian_contraction();
    std::cout << "Progress: transform contracted cloud into skeleton:\n";
    skeletonizer.skeletonization();



/*
	//plot_mesh(V,F);
    normalization(V);

    int k_for_knn = 30;

    // get the nearest neighbours
    nanoflann_wrapper knn_search(V);

    // get the one ring
    std::vector< OneRing > one_ring_list;
    for (int i=0; i<V.rows(); i++) {
        // (1) search for k closest points and (2) remove the point itself
        std::vector < int > neighbours_id;
        neighbours_id = knn_search.return_k_closest_points(V.row(i), k_for_knn+1);
        neighbours_id.erase(neighbours_id.begin());

        // look for the one_ring neighbors
        OneRing one_ring(i, neighbours_id, V);
        one_ring_list.push_back(one_ring);
    }

    // initialization of: WH, WL, and L
    Eigen::SparseMatrix<double> WH(V.rows(), V.rows());
    for (int i=0; i<V.rows(); i++)
        WH.insert(i,i) = 1;
    
    double initWL = compute_initial_laplacian_weight(V, one_ring_list);

    Eigen::SparseMatrix<double> WL(V.rows(), V.rows());
    for (int i=0; i<V.rows(); i++)
        WL.insert(i,i) = initWL*4;

    Eigen::SparseMatrix<double> L = compute_laplacian_weight(V, one_ring_list);
    Eigen::MatrixXd P, P2;

    // contraction time :)
    Eigen::SparseMatrix<double> A, A2, WlL;
    Eigen::MatrixXd b, b2, WhP, zeros;

    WhP = WH*V;
    WlL = WL*L;
    zeros = Eigen::MatrixXd::Zero(WhP.rows(), 3);

    A = concatenate(WlL, WH, 1);
    b = concatenate(zeros, WhP, 1);
    A2 = A.transpose() * A;
    b2 = A.transpose() * b;


    std::clock_t start;
    double duration;
    start = std::clock();

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver;
    solver.compute(A2);
    if(solver.info()!=Eigen::Success) {
        // decomposition failed
        std::cout << "Error: solver failed\n";
    }
    P = solver.solve(b2);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"SimplicialLDLT runned in : "<< duration <<" s\n";




    //plot_mesh(P,F);

    // next step:

    int iteration_time = 5;
    double termination_criteria = 0.01;

    double sl = 3;
    double WC = 1;

    for (int i=0; i< iteration_time; i++) {
        std::cout<<"contraction step : "<< i+1 <<"\n";

        L = compute_laplacian_weight(P, one_ring_list);
        
        WL *= sl;
        Eigen::VectorXd size, new_size;
        size = one_ring_size(V, one_ring_list, "min");
        new_size = one_ring_size(P, one_ring_list, "min");

        for (int i=0; i<V.rows(); i++)
            WH.coeffRef(i,i) = WC*size(i)/new_size(i);
            //WH.coeffRef(i,i) = sqrt(size(i)/new_size(i));

        WhP = WH*P;
        WlL = WL*L;

        A = concatenate(WlL, WH, 1);
        b = concatenate(zeros, WhP, 1);
        A2 = A.transpose() * A;
        b2 = A.transpose() * b;

        solver.compute(A2);
        if(solver.info()!=Eigen::Success) {
            // decomposition failed
            std::cout << "Error: solver failed\n";
        }
        P = solver.solve(b2);

        plot_mesh(P,F);

    }




    // turn into a Skeletonization part
    double sample_radius = 0.002;
    Eigen::MatrixXd nodes;
    Eigen::VectorXi correspondences;
    farthest_sampling_by_sphere(P, sample_radius, nodes, correspondences);

    Eigen::MatrixXi adjacency_matrix;
    connect_by_inherit_neigh(V, nodes, correspondences, one_ring_list, adjacency_matrix);
    edge_collapse_update(nodes, correspondences, adjacency_matrix);









Eigen::VectorXd correspondences_copy = correspondences.cast <double> ();
double modulo_factor = 2;

Eigen::MatrixXd vertices_color;
vertices_color = Eigen::MatrixXd::Constant(V.rows(), 3, 0.9);

for (int i=0; i<V.rows(); i++)
{
    vertices_color(i, 0) = fmod(correspondences_copy(i), modulo_factor)/modulo_factor;
    vertices_color(i, 1) = 1-fmod(correspondences_copy(i), modulo_factor)/modulo_factor;
    vertices_color(i, 2) = 1-fmod(correspondences_copy(i), modulo_factor)/modulo_factor;
}
plot_mesh (V, F, vertices_color);

*/



    return 0;
}

