/*
*   edge_collapse_update
*   by R. Falque
*   27/06/2019
*/

#ifndef EDGE_COLLASPE_UPDATE_HPP
#define EDGE_COLLASPE_UPDATE_HPP

#include <Eigen/Core>
#include <limits> 
#include <iostream>
#include "../mesh_tools/nanoflannWrapper.hpp"
#include "point_ring.hpp"
#include "../utils/EigenConcatenate.hpp"
#include "../utils/EigenFind.hpp"
#include <igl/slice.h>

#include "../graph_lib/graphStructure.hpp"



inline bool edge_collapse_update(Eigen::MatrixXd & nodes, 
                                 Eigen::VectorXi & correspondences,
                                 Eigen::MatrixXi & A)
{
    // makes sure A is full of ones
    A = A.unaryExpr([](int x) { return std::min(x, 1); });

    Graph skeleton(nodes, A);
    skeleton.init();
    skeleton.plot();
    skeleton.collapse_triangles();
    skeleton.plot();
    




/*



    // get each nodes degree
    Eigen::VectorXd degrees = Eigen::VectorXd::Constant(nodes.rows(), 0);
    for (int i=0; i<nodes.rows(); i++)
        degrees(i) = skeleton.get_node_degree(i);

    // find triangles
//skeds = zeros(0,4);% idx1, idx2, average degree of two end points, distance
    int tricount = 0;
    Eigen::MatrixXd skeds;

    for (int i=0; i<nodes.rows(); i++) {
//    ns = find(A(i,:)==1);
        Eigen::VectorXi A_row = A.row(i);
        ns = find(A_row, int(1));

std::cout << "test 3\n";
//    ns = ns( ns>i );%touch every triangle only once (if a edge belong to two triangles, it appears twice!)
        //ns = boolean_selection(ns, ns.array() > i);
        Eigen::VectorXi temp( (ns.array()>i).count() );
        int counter = 0;
        for (int i=0; i<ns.cols(); i++)
            if (ns(i) > i) {
                temp(counter) = ns(i);
                counter ++;
            }
        ns = temp;


std::cout << "test 4\n";
//    lns = length(ns);
        int lns = ns.cols();

//    for j=1:lns
        for (int j=0; j<lns; j++)

//        for k=j+1:lns
            for (int k=j+1; k<lns; k++)

//            if A(ns(j),ns(k)) == 1
                if ( A(ns(j), ns(k)) == 1 ) {

//                tricount = tricount+1;
                    tricount ++;

std::cout << "test 4.01\n";
                    // add three points to skeds
                    Eigen::MatrixXd temp_1, temp_2, temp_3;
                //skeds(end+1,1:3) = [i,ns(j), 0.5*(lns+degrees(ns(j))) ];                                                        %#ok<AGROW>
                //skeds(end,4) = euclidean_distance(spls(i,:), spls(ns(j),:) );
                    temp_1 << i, ns(j), 0.5*(lns+degrees(ns(j))), (nodes.row(i) - nodes.row(ns(j))) .norm();

                //skeds(end+1,1:3) = [ns(j),ns(k), 0.5*(degrees(ns(j)) +degrees(ns(k)) )];                                                %#ok<AGROW>
                //skeds(end,4) = euclidean_distance( spls(ns(j),:), spls(ns(k),:) );
                    temp_2 << ns(j), ns(k), 0.5*( degrees(ns(j))+degrees(ns(k)) ), (nodes.row(ns(j)) - nodes.row(ns(k))) .norm();

                //skeds(end+1,1:3) = [i,ns(k), 0.5*(lns+degrees(ns(k))) ];                                                     %#ok<AGROW>
                //skeds(end,4) = euclidean_distance( spls(ns(k),:), spls(i,:) );
                    temp_3 << i, ns(k), 0.5*(lns+degrees(ns(k))), (nodes.row(i) - nodes.row(ns(k))) .norm();

std::cout << "test 4.1\n";
                    skeds = concatenate(skeds, temp_1, 1);
                    skeds = concatenate(skeds, temp_1, 2);
                    skeds = concatenate(skeds, temp_1, 3);

std::cout << "test 5\n";
                }
//            end
//        end
//    end
//end
    }

std::cout << "skeds: \n" << skeds << std::endl;




    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.plot();
    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.plot();
    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.plot();
    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.collapse_edge(10);
    skeleton.plot();

/*
//%% --- EDGE COLLAPSE 
    while (true) {
//    disp(sprintf('decimating skeletal graph, remaining #%d loops:', size(skeds,1)));
        std::cout << "decimating skeletal graph, remaining " << skeds.size() << " loops:\n";

//    %--- STOP CONDITION
//    % no more triangles? then the structure is 1D
//    if size(skeds,1) == 0, break, end;
        if (skeds.size() == 0)
            break;

//    %--- DECIMATION STEP + UPDATES
//    % collapse the edge with minimum cost, remove the second vertex
//    if collapse_order == 1 % cost is degree + distance
//        mind = min( skeds(:,3) );
//        tmpIdx = find(skeds(:,3)==mind);
//        tmpSkeds = skeds(tmpIdx,4);
//
//        [IGN, idx] = min( tmpSkeds );
//        edge = skeds(tmpIdx(idx),1:2);
//        skeds(tmpIdx(idx),:)=[];
//    else % cost is distance
//        [IGN, idx] = min( skeds(:,4) );
//        edge = skeds(idx,1:2);
//        skeds(idx,:)=[];
//    end

        // collapse_order == 1
        Eigen::VectorXd average_degree = skeds.col(2);
        double mind = average_degree.minCoeff();
        Eigen::VectorXi tmpIdx = find(average_degree, mind);

        Eigen::VectorXd tmpSkeds = index_slice(skeds.col(3), tmpIdx);
        int Idx;
        tmpSkeds.minCoeff(&Idx);

        int edge_to_delete1 = skeds(Idx, 0);
        int edge_to_delete2 = skeds(Idx, 1);
        removeRow(skeds, Idx);

//    disp( sprintf('edge to be delete: %d, %d', edge(1),edge(2)) );
        std::cout << "edge to be delete:" << edge_to_delete1 << "-" << edge_to_delete2 << std::endl;


//        % update the location
//        spls( edge(2),: ) = mean( spls( edge,: ) );
//        spls( edge(1),: ) = NaN;
        nodes.row(edge_to_delete2) = ( nodes.row(edge_to_delete2) + nodes.row(edge_to_delete1) ) / 2;
        nodes.row(edge_to_delete1) = Eigen::Vector3d::Constant(-1); // as nan, replace by removeRow(nodes, edge_to_delete1);?

//        % update the A matrix
//        for k=1:size(A,1)
//            if A(edge(1),k) == 1, 
//                A(edge(2),k)=1; 
//                A(k,edge(2))=1; 
//            end
//        end
        for (int k=0; k<A.rows(); k++) {
            if (A(edge_to_delete1, k) == 1) {
                A(edge_to_delete2, k) = 1;
                A(k, edge_to_delete2) = 1;
            }
        }

//    % remove the row
//    A(edge(1),:) = 0;
//    A(:,edge(1)) = 0;
        removeRow(A, edge_to_delete1);
        removeCol(A, edge_to_delete1);
        
//    % update the correspondents
//    corresp(corresp==edge(1) ) = edge(2);
        for (int i=0; i<correspondences.size(); i++)
            if (correspondences(i) == edge_to_delete1)
                correspondences(i) = edge_to_delete2;

//    %%
//    % 1) remove skeds connect edge(1) and neighbor of edge(2), called 12;
//    % and skeds connect edge(2) and12; 
//    % and update skeds contain edge(1) and non-neighbor of edge(2) to edge(2) and the non-neighbor of edge(2)
//    tmpIdx = skeds( skeds(:,1)==edge(2), 2);% index connected with edge(2) in skeds.
//    tmpIdx = [tmpIdx; skeds( skeds(:,2)==edge(2), 1)];

        std::vector<int> indexes_to_update;
        for (int i=0; i<skeds.rows(); i++) {
            if (skeds(i,0)==edge_to_delete2)
                indexes_to_update.push_back(skeds(i, 1));
            if (skeds(i,1)==edge_to_delete1)
                indexes_to_update.push_back(skeds(i, 0));
        }

//    [rows,cols] = find(skeds(:,1:2)==edge(1));
        std::vector<int> index_to_remove_cols, index_to_remove_rows;
        for (int i=0; i<skeds.rows(); i++) {
            if (skeds(i,0)==edge_to_delete1 )
            {
                index_to_remove_rows.push_back(i);
                index_to_remove_cols.push_back(0);
            }
            if (skeds(i,1)==edge_to_delete1 )
            {
                index_to_remove_rows.push_back(i);
                index_to_remove_cols.push_back(1);
            }
        }
        


















    toBeRemoved =  zeros(0,1);
    for i = 1:length(rows)
        col = 1 + mod(cols(i),2);
        if ismember( skeds(rows(i), col), tmpIdx )%remove
            toBeRemoved(end+1) = rows(i);
        else
            skeds(rows(i), cols(i)) = edge(2);
        end
    end
    if ~isempty(toBeRemoved)
        skeds(toBeRemoved,: ) = [];
    end

    }
    
    
    %% 2) remove skeds which contain edge(2) and nolonger a edge of a triangle and
    % add new triangle edges containing edge(2).
    % 2.1) find all triangles contain edge(2)
    ns = find( A(edge(2),:)==1 );
    ns = ns( ns~=edge(2) );
    lns = length(ns);
    % triangles contain edges(2) include both the two kinds of tmpEdges 
    tmpEdges = zeros(0,2); % edges contain edge(2)
    tmpEdges1 = zeros(0,2); % edges not contain edge(2)
    for j=1:lns
        for k=j+1:lns
            if A(ns(j),ns(k)) == 1
                tmpEdges(end+1,:) = [edge(2),ns(j)];  
                tmpEdges(end+1,:) = [edge(2),ns(k)];
                tmpEdges1(end+1,:) = [ns(j),ns(k)];                
            end
        end
    end
    
    % 2.2) remove all edges do not belong to tmpEdges
    [rows,cols] = find(skeds(:,1:2)==edge(2));
    toBeRemoved =  zeros(0,1);
    tobedel = zeros(0,1); 
    for j = 1:length(rows)
        col = 1 + mod(cols(j),2);
        tmp = find( tmpEdges(:,2) == skeds(rows(j),col) );
        if tmp % is an edge of tmpEdge, then remove it from tmpEdge    
            for k = 1:length(tmp)
                if ~ismember (tmp(k), tobedel)
                    tobedel = [tobedel; tmp(k)];
                end
            end
        else
            toBeRemoved(end+1) = rows(j);
        end
    end
    if ~isempty(toBeRemoved)
        skeds( toBeRemoved,: ) = [];
    end
    if ~isempty(tobedel)
        tmpEdges(tobedel,:) = [];
    end
    
    % 2.3) add triangle edges new formed
    tmpEdges = [tmpEdges; tmpEdges1];
    for j = 1:size(tmpEdges, 1)
        tedge = tmpEdges(j,:);
        [rows,cols] = find(skeds(:,1:2)==tedge(1));
        bin = false;
        for k = 1:length(rows)
            col = 1 + mod(cols(k),2);
            if skeds(rows(k),col)==tedge(2) % the edge already in skeds
                bin = true;
                break;
            end
        end
        if ~bin % add edge to skeds
            ns = find(A(tedge(1),:)==1);
            degrees(tedge(1)) = (length(ns)-1)*0.5;
            ns = find(A(tedge(2),:)==1);
            degrees(tedge(2)) = (length(ns)-1)*0.5;
            skeds(end+1,1:2) = tedge;
            skeds(end,3) = 0.5*(degrees(tedge(1))+degrees(tedge(2)));
            skeds(end,4) = euclidean_distance(spls(tedge(1),:), spls(tedge(2),:) );             
        end
    end
    
    %% 3) update distance and degree of edges contain edge(2)
    [rows,cols] = find(skeds(:,1:2)==edge(2)); 
    ns = find(A(edge(2),:)==1);
    degrees(edge(2)) = (length(ns)-1)*0.5;
    
    for j = 1:length(rows)
        col = 1 + mod(cols(j),2);   
        k = skeds(rows(j),col);
        ns = find(A(k,:)==1);
        degrees(k) = (length(ns)-1)*0.5;
        
        skeds(rows(j),3) = 0.5*(degrees(edge(2))+degrees(k));
        skeds(rows(j),4) = euclidean_distance(spls(edge(2),:), spls(k,:) ); 
    end
end
function dist = euclidean_distance(p1, p2)
v=p1-p2;
dist = sqrt(dot(v,v));
*/


    return true;
};

#endif
