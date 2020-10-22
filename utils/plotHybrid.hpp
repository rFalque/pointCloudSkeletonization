/**
 * Author: R. Falque
 * 
 * plot mesh in polyscope
 * by R. Falque
 * 26/09/2019
 **/

#ifndef PLOT_HYBRID_HPP
#define PLOT_HYBRID_HPP

#include <Eigen/Core>

#include <string>

#include "plotMesh.hpp"
#include "plotCloud.hpp"

inline bool plot_mesh_and_cloud (const Eigen::MatrixXd& mesh_V, const Eigen::MatrixXi& mesh_F, const Eigen::MatrixXd& cloud_V) {

    MeshVisualization viz_mesh;
    viz_mesh.add_mesh(mesh_V, mesh_F);

    CloudVisualization viz_cloud;
    viz_cloud.add_cloud(cloud_V);
    
    polyscope::view::resetCameraToHomeView();
    polyscope::show();
    polyscope::removeAllStructures();
    return 0;
};

#endif
