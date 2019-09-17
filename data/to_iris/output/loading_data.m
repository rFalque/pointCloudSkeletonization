%% Description:
% 
% see for documentation:
% https://au.mathworks.com/help/matlab/ref/graph.html
% https://au.mathworks.com/help/matlab/ref/graph.dfsearch.html

clear variables; close all; clc;

%% post processing of the data
% load data
graph_nodes = csvread("graph_nodes.csv");
graph_edges = csvread("graph_edges.csv");

ptCloud_keyframe = pcread("bunny_keypoints.ply");
ptCloud_with_noise = pcread("bunny_less_noise.ply");

maps_id_keyframe = csvread("map_ids.csv");
maps_id_for_noisy_ptCloud = zeros(size(ptCloud_with_noise.Location, 1), 1);

% loop for assignement of the maps_id from the ptCloud_with_noise to
% pt_cloud_keyframe:
for i = 1:size(ptCloud_with_noise.Location, 1)
    point = ptCloud_with_noise.Location(i, :);
    maps_id_for_noisy_ptCloud(i) = maps_id_keyframe(ptCloud_keyframe.findNearestNeighbors(point, 1));
end


pointcloud = ptCloud_keyframe.Location;

% change from 0-index in C++ to 1-index in Matlab:
maps_id_keyframe = maps_id_keyframe +1;
maps_id_for_noisy_ptCloud = maps_id_for_noisy_ptCloud + 1;

graph_edges = graph_edges +1;

%% build skeleton graph
skeleton = graph;
skeleton = addnode(skeleton,size(graph_nodes, 1));
for i=1:size(graph_edges, 1)
    skeleton = addedge(skeleton,graph_edges(i,1), graph_edges(i,2));
end

figure
subplot 121;
plot(skeleton);
title('original graph')

%% build the graph with neighbours
% set depth first search traversal
events = {'edgetonew','startnode'};
T = dfsearch(skeleton,1,events,'Restart',true);

% process each step:
new_graph = graph;
depth = zeros(size(skeleton.Nodes,1), 1);
submaps = [];

used = zeros(size(skeleton.Nodes,1), 1);
for i = 1:size(T,1)
    switch T{i,1}
        case 'startnode'
            current = T{i,2};
            new_graph = addnode(new_graph, {num2str(current)});
            
            % find children
            found_children = find(T.Edge(:,1) == current);
            children =  T.Edge(found_children,2);

            % generate submap with parent and children
            submap = pointcloud(maps_id_keyframe == current, :);
            used(current) = used(current)+1;
            for j = 1:size(children,1)
                submap = [submap;
                          pointcloud(maps_id_keyframe == children(j), :)];
                used(children(j)) = used(children(j))+1;
            end

            submaps{end+1} = submap;
            
        case 'edgetonew'
            parent = T{i,3}(1);
            current = T{i,3}(2);
            depth(current) = depth(parent) + 1;
            
            % this is an even map and should be added to the final graph
            if ~mod(depth(current), 2)
                new_graph = addnode(new_graph, {num2str(current)});
                
                % find parent
                found_ancester = find(T.Edge(:,2) == parent);
                ancester = T.Edge(found_ancester,1);
                
                new_graph = addedge(new_graph, {num2str(ancester)}, {num2str(current)});
                
                % find children
                found_children = find(T.Edge(:,1) == current);
                children =  T.Edge(found_children,2);
                
                % generate submap with parent and children
                submap = pointcloud(maps_id_keyframe == current, :);
                used(current) = used(current)+1;
                submap = [submap;
                          pointcloud(maps_id_keyframe == parent, :)];
                used(parent) = used(parent)+1;
                for j = 1:size(children,1)
                    submap = [submap;
                              pointcloud(maps_id_keyframe == children(j), :)];
                    used(children(j)) = used(children(j))+1;
                end
                
                submaps{end+1} = submap;
            end
            
        otherwise
    end
end

number_of_points = 0;
for i = 1:size(used, 1)
    number_of_points = number_of_points + sum(maps_id_keyframe == i)*used(i);
end

subplot 122;
plot(new_graph);
title('final graph')


%% GP mapping (to be added here):

% set traversal
events = {'edgetonew','startnode'};
T = dfsearch(new_graph,1,events,'Restart',true);

priors = [];

for i = 1:size(T,1)
    switch T{i,1}
        case 'startnode'
            % initial submap:
            
            % Gmapping here
            output = 1;
            priors{end+1} = output;
        case 'edgetonew'
            % next submap:
            
            parent = T{i,3}(1);
            current = T{i,3}(2);
            
            prior_to_map = priors{parent};
            
            % gmapping with parent as a prior
            
            % set current prior
            output = 1;
            priors{end+1} = output;
        otherwise
    end
end


