%% Description:
% load the data
% 
% see for documentation:
% https://au.mathworks.com/help/matlab/ref/graph.html
% https://au.mathworks.com/help/matlab/ref/graph.dfsearch.html

clear variables; close all; clc;

%% post processing of the data
% load data
maps_id = csvread("map_ids.csv");
graph_nodes = csvread("graph_nodes.csv");
graph_edges = csvread("graph_edges.csv");
ptCloud = pcread("bunny_less_noise.ply");
pointcloud = ptCloud.Location;

% change from 0-index in C++ to 1-index in Matlab:
maps_id = maps_id +1;
graph_edges = graph_edges +1;

% build skeleton graph
skeleton = graph;
skeleton = addnode(skeleton,size(graph_nodes, 1));
for i=1:size(graph_edges, 1)
    skeleton = addedge(skeleton,graph_edges(i,1), graph_edges(i,2));
end

figure
subplot 121;
plot(skeleton);
title('original graph')

% set depth first search traversal
events = {'edgetonew','startnode'};
T = dfsearch(skeleton,1,events,'Restart',true);

% process each step:
new_graph = graph;
depth = zeros(size(skeleton.Nodes,1), 1);
submaps = [];
for i = 1:size(T,1)
    switch T{i,1}
        case 'startnode'
            current = T{i,2};
            new_graph = addnode(new_graph, {num2str(current)});
            
            % find children
            found_children = find(T.Edge(:,1) == current);
            children =  T.Edge(found_children,2);

            % generate submap with parent and children
            submap = pointcloud(i == current, :);
            for j = 1:size(children,1)
                submap = [submap;
                          pointcloud(i == children(j), :)];
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
                submap = pointcloud(i == current, :);
                submap = [submap;
                          pointcloud(i == parent, :)];
                for j = 1:size(children,1)
                    submap = [submap;
                              pointcloud(i == children(j), :)];
                end
                
                submaps{end+1} = submap;
            end
            
        otherwise
    end
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


