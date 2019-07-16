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

#include "../utils/EigenMinMax.hpp"
#include "graphOptions.hpp"


namespace directional
{
  // creates a mesh of small cylinders to visualize lines on the overlay of the mesh
  // Inputs:
  //  P1,P2:      #P by 3 coordinates of the endpoints of the cylinders
  //  radius:     Cylinder base radii
  //  cyndColors: #P by 3 RBG colors per cylinder
  //  res:        The resolution of the cylinder (size of base polygon)
  // Outputs:
  //  V   #V by 3 cylinder mesh coordinates
  //  T   #T by 3 mesh triangles
  //  C   #T by 3 face-based colors
  inline bool line_cylinders(const Eigen::MatrixXd& P1,
                                 const Eigen::MatrixXd& P2,
                                 const double& radius,
                                 const Eigen::MatrixXd& cyndColors,
                                 const int res,
                                 Eigen::MatrixXd& V,
                                 Eigen::MatrixXi& T,
                                 Eigen::MatrixXd& C)
  {
    using namespace Eigen;
    int VOffset, TOffset, COffset;
    V.resize(2*res*P1.rows(),3);
    T.resize(2*res*P1.rows(),3);
    int NewColorSize=T.rows();
    C.resize(NewColorSize,3);
    VOffset=TOffset=COffset=0;
   
    RowVector3d ZAxis; ZAxis<<0.0,0.0,1.0;
    RowVector3d YAxis; YAxis<<0.0,1.0,0.0;
    
    MatrixXd PlanePattern(res,2);
    for (int i=0;i<res;i++){
      std::complex<double> CurrRoot=exp(2*M_PI*std::complex<double>(0,1)*(double)i/(double)res);
      PlanePattern.row(i)<<CurrRoot.real(), CurrRoot.imag();
    }
    
    for (int i=0;i<P1.rows();i++){
      RowVector3d NormAxis=(P2.row(i)-P1.row(i)).normalized();
      RowVector3d PlaneAxis1=NormAxis.cross(ZAxis);
      if (PlaneAxis1.norm()<10e-2)
        PlaneAxis1=NormAxis.cross(YAxis).normalized();
      else
        PlaneAxis1=PlaneAxis1.normalized();
      RowVector3d PlaneAxis2=NormAxis.cross(PlaneAxis1).normalized();
      for (int j=0;j<res;j++){
        int v1=2*res*i+2*j;
        int v2=2*res*i+2*j+1;
        int v3=2*res*i+2*((j+1)%res);
        int v4=2*res*i+2*((j+1)%res)+1;
        V.row(v1)<<P1.row(i)+(PlaneAxis1*PlanePattern(j,0)+PlaneAxis2*PlanePattern(j,1))*radius;
        V.row(v2)<<P2.row(i)+(PlaneAxis1*PlanePattern(j,0)+PlaneAxis2*PlanePattern(j,1))*radius;
        
        T.row(2*res*i+2*j)<<VOffset+v3,VOffset+v2,VOffset+v1;
        T.row(2*res*i+2*j+1)<<VOffset+v4,VOffset+v2,VOffset+v3;
        
        C.row(2*res*i+2*j)<<cyndColors.row(i);
        C.row(2*res*i+2*j+1)<<cyndColors.row(i);
        
      }
    }
    return true;
  };
  


  // creates small spheres to visualize points on the overlay of the mesh
  // Input:
  //  P:      #P by 3 coordinates of the centers of spheres
  //  radius: radii of the spheres
  //  C:      #P by 3 - RBG colors per sphere
  //  res:    the resolution of the sphere discretization
  // extendMesh if to extend the V,T,TC, or to overwrite them
  // Output:
  //  V:    #V by 3 sphere mesh coordinates
  //  T     #T by 3 sphere mesh triangles
  //  C:    #T by 3 face-based colors
  inline bool point_spheres(const Eigen::MatrixXd& points,
                                const double& radius,
                                const Eigen::MatrixXd& colors,
                                const int res,
                                Eigen::MatrixXd& V,
                                Eigen::MatrixXi& T,
                                Eigen::MatrixXd& C)
  {
    using namespace Eigen;
    V.resize(res*res*points.rows(),3);
    T.resize(2*(res-1)*res*points.rows(),3);
    C.resize(T.rows(),3);
    
    for (int i=0;i<points.rows();i++){
      RowVector3d center=points.row(i);
      
      //creating vertices
      for (int j=0;j<res;j++){
        double z=center(2)+radius*cos(M_PI*(double)j/(double(res-1)));
        for (int k=0;k<res;k++){
          double x=center(0)+radius*sin(M_PI*(double)j/(double(res-1)))*cos(2*M_PI*(double)k/(double(res-1)));
          double y=center(1)+radius*sin(M_PI*(double)j/(double(res-1)))*sin(2*M_PI*(double)k/(double(res-1)));
          V.row((res*res)*i+j*res+k)<<x,y,z;
        }
      }
      
      
      //creating faces
      for (int j=0;j<res-1;j++){
        for (int k=0;k<res;k++){
          int v1=(res*res)*i+j*res+k;
          int v2=(res*res)*i+(j+1)*res+k;
          int v3=(res*res)*i+(j+1)*res+(k+1)%res;
          int v4=(res*res)*i+j*res+(k+1)%res;
          T.row(2*(((res-1)*res)*i+res*j+k))<<v1,v2,v3;
          T.row(2*(((res-1)*res)*i+res*j+k)+1)<<v4,v1,v3;
          C.row(2*(((res-1)*res)*i+res*j+k))<<colors.row(i);
          C.row(2*(((res-1)*res)*i+res*j+k)+1)<<colors.row(i);
        }
      }
    }
    
    return true;
  };

}



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
