/**
 * creates a mesh of small cylinders and spheres to visualize the graph
 * Inputs:
 *  nodes:        #N by 3 coordinates of the endpoints of the cylinders
 *  edges:        #E by 2 coordinates of the endpoints of the cylinders
 *  nodesColors:  #N by 3 RBG colors per cylinder
 *  edgesColors:  #E by 3 RBG colors per cylinder
 *  res:        The resolution of the cylinder (size of base polygon)
 * Outputs:
 *  plot the graph in the libigl viewer
 * 
 * by R. Falque
 * 14/02/2019
 **/

#ifndef PLOT_GRAPH_HPP
#define PLOT_GRAPH_HPP

#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>

#include "../external_library/directional/line_cylinders.h"
#include "../external_library/directional/point_spheres.h"

#include "getMinMax.hpp"
#include "options.hpp"

inline bool add_graph (igl::opengl::glfw::Viewer & viewer,
                        const Eigen::MatrixXd & nodes,
                        const Eigen::MatrixXi & edges,
                        const Eigen::MatrixXd & nodes_colors,
                        const Eigen::MatrixXd & edges_colors,
                        const double & nodes_radius,
                        const double & edges_radius,
                        const int res)
{
    Eigen::MatrixXd edges_begin, edges_end;
    Eigen::MatrixXi cols;
    cols.resize(1,3);
    cols << 0,1,2;

    edges_begin.resize(edges.rows(), 3);
    edges_end.resize(edges.rows(), 3);

    for(int i=0;i<edges.rows(); i++)
    {
        edges_begin.row(i) << nodes.row(edges(i,0));
        edges_end.row(i) << nodes.row(edges(i,1));
    }

    Eigen::MatrixXd nodes_V, edges_V, concatenated_V;
    Eigen::MatrixXi nodes_T, edges_T, concatenated_T;
    Eigen::MatrixXd nodes_C, edges_C, concatenated_C;

    directional::point_spheres(nodes, nodes_radius, nodes_colors, res, nodes_V, nodes_T, nodes_C);
    directional::line_cylinders(edges_begin, edges_end, edges_radius, edges_colors, res, edges_V, edges_T, edges_C);

    viewer.append_mesh();
    viewer.data().set_mesh(nodes_V, nodes_T);
    viewer.data().set_colors(nodes_C);
    viewer.data().show_lines = false;

    viewer.append_mesh();
    viewer.data().set_mesh(edges_V, edges_T);
    viewer.data().set_colors(edges_C);
    viewer.data().show_lines = false;
    viewer.core().background_color << 1, 1, 1, 1;

    return true;
};

inline bool add_graph (igl::opengl::glfw::Viewer & viewer,
                        const Eigen::MatrixXd & nodes,
                        const Eigen::MatrixXi & edges,
                        graph_options opts)
{
    Eigen::MatrixXd nodes_colors, edges_colors;

    nodes_colors = Eigen::MatrixXd::Constant(nodes.rows(),3,0.1);
    nodes_colors.col(0) = Eigen::MatrixXd::Constant(nodes.rows(),1, opts.nodes_color[0]);
    nodes_colors.col(1) = Eigen::MatrixXd::Constant(nodes.rows(),1, opts.nodes_color[1]);
    nodes_colors.col(2) = Eigen::MatrixXd::Constant(nodes.rows(),1, opts.nodes_color[2]);

    edges_colors = Eigen::MatrixXd::Constant(edges.rows(),3,0.1);
    edges_colors.col(0) = Eigen::MatrixXd::Constant(edges.rows(),1,opts.edges_color[0]);
    edges_colors.col(1) = Eigen::MatrixXd::Constant(edges.rows(),1,opts.edges_color[1]);
    edges_colors.col(2) = Eigen::MatrixXd::Constant(edges.rows(),1,opts.edges_color[2]);

    double scale;
    getScale(nodes, scale);

    double nodes_radius = scale/opts.nodes_ratio;
    double edges_radius = scale/opts.edges_ratio;

    return add_graph (viewer, nodes, edges,nodes_colors, edges_colors, nodes_radius, edges_radius, opts.graph_res);
};


inline bool plot_graph (const Eigen::MatrixXd& nodes,
                        const Eigen::MatrixXi& edges,
                        const Eigen::MatrixXd& nodes_colors,
                        const Eigen::MatrixXd& edges_colors,
                        const double& nodes_radius,
                        const double& edges_radius,
                        const int res)
{
    igl::opengl::glfw::Viewer viewer;
    add_graph (viewer, nodes, edges, nodes_colors, edges_colors, nodes_radius, edges_radius, res);
    viewer.launch();

    return true;
};

inline bool plot_graph (const Eigen::MatrixXd & nodes,
                        const Eigen::MatrixXi & edges,
                        graph_options opts)
{
    Eigen::MatrixXd nodes_colors, edges_colors;

    nodes_colors = Eigen::MatrixXd::Constant(nodes.rows(),3,0.1);
    nodes_colors.col(0) = Eigen::MatrixXd::Constant(nodes.rows(),1, opts.nodes_color[0]);
    nodes_colors.col(1) = Eigen::MatrixXd::Constant(nodes.rows(),1, opts.nodes_color[1]);
    nodes_colors.col(2) = Eigen::MatrixXd::Constant(nodes.rows(),1, opts.nodes_color[2]);

    edges_colors = Eigen::MatrixXd::Constant(edges.rows(),3,0.1);
    edges_colors.col(0) = Eigen::MatrixXd::Constant(edges.rows(),1,opts.edges_color[0]);
    edges_colors.col(1) = Eigen::MatrixXd::Constant(edges.rows(),1,opts.edges_color[1]);
    edges_colors.col(2) = Eigen::MatrixXd::Constant(edges.rows(),1,opts.edges_color[2]);

    double scale;
    getScale(nodes, scale);

    double nodes_radius = scale/opts.nodes_ratio;
    double edges_radius = scale/opts.edges_ratio;

    return plot_graph (nodes, edges,nodes_colors, edges_colors, nodes_radius, edges_radius, opts.graph_res);
};

inline bool plot_graph (const Eigen::MatrixXd & nodes,
                        const Eigen::MatrixXi & edges)
{
    Eigen::MatrixXd nodes_colors, edges_colors;

    nodes_colors = Eigen::MatrixXd::Constant(nodes.rows(),3,0.1);
    edges_colors = Eigen::MatrixXd::Constant(edges.rows(),3,0.1);
    nodes_colors.col(0) = Eigen::MatrixXd::Constant(nodes.rows(),1,1);

    double scale;
    getScale(nodes, scale);

    double nodes_radius = scale/50;
    double edges_radius = scale/200;
    int res = 30;

    return plot_graph (nodes, edges,nodes_colors, edges_colors, nodes_radius, edges_radius, res);
};


#endif
