#ifndef POINT_SKELETONIZATION_HPP
#define POINT_SKELETONIZATION_HPP

#include <algorithm>           // std::all_of
#include <Eigen/Dense>
#include <limits>
#include "readGraphOBJ.hpp"
#include "plotGraph.hpp"
#include "graphOptions.hpp"


class pointSkeletonization
{
private:
    point_skeletonization_options opts_;                                // Store all visualization infos


    Eigen::MatrixXd V_; // V: vertex of the surface
    Eigen::MatrixXi F_; // F: faces of the surface

	// basic structure for edges and nodes
	Eigen::MatrixXd nodes_;                                             // should be a N by 3 matrix of doubles
	Eigen::MatrixXi edges_;                                             // should be a M by 2 matrix of integers
	int num_nodes_;                                                     // set to N
	int num_edges_;                                                     // set to M
	
	// structures used for fast circulation through data
	std::vector< std::vector<int> > adjacency_list_;                    // contains for each nodes, its nodes neighbors
	std::vector< std::vector<int> > adjacency_edge_list_;               // contains for each nodes, its edges neighbors
	Eigen::VectorXd edges_length_;                                      // used only for Dijkstra

	// connectivity properties
	int is_connected_ = -1;                                             // 0 no, 1 yes, -1 undefined
	int is_biconnected_ = -1;                                           // 0 no, 1 yes, -1 undefined
	int is_triconnected_ = -1;                                          // 0 no, 1 yes, -1 undefined
	int has_bridges_ = -1;                                              // 0 no, 1 yes, -1 undefined

	// storing of the cut sets
	std::vector< int > one_cut_vertices_;                               // set of articulation points
	std::vector< std::pair<int, int> > two_cut_vertices_;               // set of two-cut vertices
	std::vector< std::pair<int, int> > bridges_;                        // set of briges
	
	// internal functions used iteratively (defined at the bottom of the file)
	void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);
	void removeRow(Eigen::MatrixXi& matrix, unsigned int rowToRemove);
	void removeDuplicates(std::vector<std::pair<int, int>>& v);
	void DFSUtil(int u, std::vector< std::vector<int> > adj, std::vector<bool> &visited);
	void APUtil(int u, std::vector<bool> & visited, int disc[], int low[], std::vector<int> & parent, std::vector<bool> & ap);
	void bridgeUtil(int u, std::vector<bool> & visited, int disc[], int low[], std::vector<int> & parent, std::vector< std::pair<int, int> > & bridges);
	bool is_element_in_vector(int a, std::vector<int> & A, int & element_position);

public:

	// overloaded creations of the class (set verbose and load the data)
	Graph(std::string file_name)
	{
		readGraphOBJ(file_name, nodes_, edges_);
		set_default_options();
	}

	Graph(std::string file_name, graph_options opts)
	{
		readGraphOBJ(file_name, nodes_, edges_);
		opts_ = opts;
	}

	Graph(Eigen::MatrixXd nodes, Eigen::Matrix<int, Eigen::Dynamic, 2> edges)
	{
		nodes_ = nodes;
		edges_ = edges;
		set_default_options();
	}

	Graph(Eigen::MatrixXd nodes, Eigen::Matrix<int, Eigen::Dynamic, 2> edges, graph_options opts)
	{
		nodes_ = nodes;
		edges_ = edges;
		opts_ = opts;
	}

	Graph(Eigen::MatrixXd nodes, Eigen::MatrixXi adjacency_matrix)
	{
		if (adjacency_matrix.rows() != adjacency_matrix.cols() || adjacency_matrix.rows() != nodes.rows()) {
			std::cout << "Error: wrong input size when assessing adjacency_matrix.rows() != adjacency_matrix.cols() || adjacency_matrix.rows() != nodes.rows()\n ";
			std::exit(0);
		}
		if (adjacency_matrix.transpose() != adjacency_matrix)
		{
			std::cout << "Error: the adjacency_matrix should be symmetric\n ";
			std::exit(0);
		}

		nodes_ = nodes;
		Eigen::MatrixXi edges((adjacency_matrix.array() == 0).count(), 2);

		// explore the upper triangle of the adjacency_matrix (without the diagonal)
		int num_edges = 0;
		for (int i=0; i<adjacency_matrix.rows(); i++)
			for (int j=i+1; j<adjacency_matrix.cols(); j++) {
				if (adjacency_matrix(i,j) == 1)
				{
					edges(num_edges, 0) = i;
					edges(num_edges, 1) = j;
					num_edges ++;
				}
			}
		edges.conservativeResize(num_edges, 2);

		edges_ = edges;
		set_default_options();
	}


	// destructor
	~Graph()
	{
	}

	// initialisation of the private variables
	bool init()
	{
		if (nodes_.cols()!=3 || edges_.cols()!=2) {
			std::cout << "Error: wrong graph dimensions" << std::endl;
			std::exit(1);
		}

		one_cut_vertices_.clear();
		two_cut_vertices_.clear();
		bridges_.clear();

		// set up properties
		num_nodes_ = nodes_.rows();
		num_edges_ = edges_.rows();

		// set up edges_length
		edges_length_ = Eigen::VectorXd::Zero(num_edges_);
		for (int i=0; i<num_edges_; i++)
			edges_length_(i) = (nodes_.row(edges_(i,0)) - nodes_.row(edges_(i,1)) ).norm();
		
		// set up the adjacency list
		set_adjacency_lists();

		return true;
	}

	bool set_default_options()
	{
		opts_.verbose = false;
		opts_.nodes_ratio = 50.0;
		opts_.edges_ratio = 200.0;
		opts_.graph_res = 30.0;
		opts_.nodes_color = {1.0, 0.1, 0.1};
		opts_.edges_color = {0.1, 0.1, 0.1};
	}

};

#endif
