/*
*   open .obj file and read the graph
*   in .obj files, edges are noted "l" (for line)
*                  vertices are noted "v"
*   
*   this file is scavenged from the readOBJ file of libIGL
*   by R. Falque
*   10/05/2019
*/

#ifndef READ_GRAPH_OBJ_HPP
#define READ_GRAPH_OBJ_HPP

#include <Eigen/Core>
#include <vector>
#include "../utils/EigenListToMatrix.hpp"

#include <string>
#include <cstdio>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>

bool readGraphOBJ(const std::string obj_file_name, Eigen::MatrixXd & V_out, Eigen::MatrixXi & E_out){
  bool verbose = false;

	std::vector<std::vector<double > > V;
	std::vector<std::vector<int > > E;

  // Open file, and check for error
  FILE * obj_file = fopen(obj_file_name.c_str(),"r");
  if(NULL==obj_file)
  {
    fprintf(stderr,"IOError: %s could not be opened...\n",
            obj_file_name.c_str());
    return false;
  }

  V.clear();
  E.clear();

  // variables and constants to assist parsing the .obj file
  // Constant strings to compare against
  std::string v("v");
  std::string e("l");
  std::string tic_tac_toe("#");

  #ifndef IGL_LINE_MAX
  #  define IGL_LINE_MAX 2048
  #endif

    char line[IGL_LINE_MAX];
  int line_no = 1;
  while (fgets(line, IGL_LINE_MAX, obj_file) != NULL)
  {
    char type[IGL_LINE_MAX];
    // Read first word containing type
    if(sscanf(line, "%s",type) == 1)
    {
      // Get pointer to rest of line right after type
      char * l = &line[strlen(type)];
      if(type == v)
      {
        std::istringstream ls(&line[1]);
        std::vector<double > vertex{ std::istream_iterator<double >(ls), std::istream_iterator<double >() };

        if (vertex.size() < 3)
        {
          fprintf(stderr,
                  "Error: readOBJ() vertex on line %d should have at least 3 coordinates",
                  line_no);
          fclose(obj_file);
          return false;
        }
      
        V.push_back(vertex);
      }else if(type == e)
      {
        std::istringstream ls(&line[1]);
        std::vector<int > edge{ std::istream_iterator<int >(ls), std::istream_iterator<int >() };

        if (edge.size() != 2)
        {
          fprintf(stderr,
                  "Error: readOBJ() vertex on line %d should have 2 coordinates",
                  line_no);
          fclose(obj_file);
          return false;
        }
      
        E.push_back(edge);
      }else if(strlen(type) >= 1 && (type[0] == '#' ||
            type[0] == 'g'  ||
            type[0] == 's'  ||
            strcmp("usemtl",type)==0 ||
            strcmp("mtllib",type)==0))
      {
        //ignore comments or other shit
      }else
      {
        //ignore any other lines
        if (verbose)
        fprintf(stderr,
                "Warning: readOBJ() ignored non-comment line %d:\n  %s",
                line_no,
                line);
      }
    }else
    {
      // ignore empty line
    }
    line_no++;
  }
  fclose(obj_file);

  list_to_matrix(V, V_out);
  list_to_matrix(E, E_out);

  E_out = E_out - Eigen::MatrixXi::Constant(E_out.rows(),2,1);

  return true;
}

#endif
