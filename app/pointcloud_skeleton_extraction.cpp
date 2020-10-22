/* Author: R. Falque
 * 
 * main for testing the skeleton extraction
 * by R. Falque
 * 27/06/2019
 */

#include "IO/readOBJ.h"
#include "IO/readOFF.h"
#include "IO/readPLY.h"
#include "IO/writePLY.h"

#include "yaml-cpp/yaml.h"

#include <Eigen/Dense>
#include <vector>

#include "polyscope/polyscope.h"

#include "plotMesh.hpp"
#include "options.hpp"

#include "EigenTools/writeToCSV.hpp"
#include "EigenTools/remove_duplicates.h"


#include "skeleton_toolbox/pointSkeletonization.hpp"
#include "libGraphCpp/include/libGraphCpp/graph.hpp"
#include "libGraphCpp/include/libGraphCpp/plotGraph.hpp"

/*
 * List of things that could go wrong:
 *  - in the laplacian computation, line (66), division by number of neighbours needed?
 *  - in the point_ring, what happen if the one_ring is not connected?
 *  - does the laplacian need to be symmetric (see compute_point_laplacian.m line 31)
 *  - does the laplacian need to be normalized
 *  - geometry with genus > 1
 */

/* TODO:
 *  - return the parameters
 */

int main(int argc, char* argv[])
{
    polyscope::init();

    options opts;
    opts.loadYAML("../config.yaml");

    Eigen::MatrixXd V_tmp, V; // V: vertex of the surface
    Eigen::MatrixXi F_tmp, F; // F: faces of the surface (used for plots)

    std::string file_extension = opts.path_input_obj.substr(opts.path_input_obj.size() - 3);
    
    std::cout << "Progress: load file ... ";
    if (file_extension == "off")
        igl::readOFF(opts.path_input_obj, V_tmp, F_tmp);
    else if (file_extension == "ply")
        igl::readPLY(opts.path_input_obj, V_tmp, F_tmp);
    std::cout << " done\n";

    // remove duplicates
    std::cout << "Progress: remove duplicates ... ";
    Eigen::VectorXi I;
    igl::remove_duplicates(V_tmp, F_tmp, V, F, I);
    std::cout << "done\n";
    
    if (F.rows() == 0) {
        opts.cloud_only = true;
        plot_cloud (V);
    } else {
        plot_mesh (V, F);
    }

    PointSkeletonization skeletonizer(V, F, opts);
    skeletonizer.skeletonize();

    // get correspondences between the skeleton and the vertices
    Eigen::VectorXi correspondences;
    correspondences = skeletonizer.get_correspondences();

    EigenWriteToCSVfile(correspondences, "../data/output/map_ids.csv");

    // get skeleton
    libgraphcpp::Graph skeleton = skeletonizer.get_skeleton();
    skeleton.save("../data/output/skeleton_graph.obj");
    //skeleton.plot();

    visualization::plot(skeleton, "pointcloud skeletonized");

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

    if (opts.cloud_only)
        plot_cloud (V, vertices_color);
    else
        plot_mesh (V, F, vertices_color);

    EigenWriteToCSVfile(skeleton.get_nodes(), "../data/output/graph_nodes.csv");
    EigenWriteToCSVfile(skeleton.get_edges(), "../data/output/graph_edges.csv");

    return 0;
}

