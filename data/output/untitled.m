% https://au.mathworks.com/help/matlab/ref/graph.html
% https://au.mathworks.com/help/matlab/ref/graph.dfsearch.html

clear variables; close all; clc;

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














% set traversal
events = {'edgetonew','startnode'};
T = dfsearch(skeleton,1,events,'Restart',true);

new_graph = graph;
depth = zeros(size(skeleton.Nodes,1), 1);
submaps = [];
for i = 1:size(T,1)
    switch T{i,1}
        case 'startnode'
            current = T{i,2};
            new_graph = addnode(new_graph, {num2str(current)});
            parent = current;
            root = current;
            depth(current) = 0;
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




%T = dfsearch(skeleton,1,'allevents');



subplot 122;
plot(new_graph);

% build the set of submaps
submaps = [];
new_graph = graph;
ancester = NaN;

this_sumap_is_an_overlap = 0;
i = 1;
tree_travelled = 0;

depth = zeros(size(skeleton.Nodes,1), 1);
visited = zeros(size(skeleton.Nodes,1), 1);

ancester = 0;
for i=1:size(T,1)
    edge_source = T(i,1);
    edge_target = T(i,2);
    
    
    
end

while ~tree_travelled
    
    if ~this_sumap_is_an_overlap
        current = T(i,1);
        
        new_graph = addnode(new_graph, {num2str(current)});
        
        while T(i,1) == current
            

            i = i+1;
        end
        
        
        if ~isnan(ancester)
            new_graph = addedge(new_graph, ancester, current);
        end
        
        ancester = current;
    end
    
    if this_sumap_is_an_overlap
        temp = T(i,1);
        while T(i,1) == temp
            i = i+1;
        end
    end
    
    this_sumap_is_an_overlap = ~this_sumap_is_an_overlap;
    
end


figure
plot(new_graph)

a = 1

if 0
    
    % process edge
    edge_source = T{i,3}(1);
    edge_target = T{i,3}(2);
    
    % every odd element is used for generating a map
    if mod(i, 2) % odd
        new_graph = addnode(new_graph, {num2str(edge_source)});
        new_graph.Nodes
        
        % add edges to ancester
        if ancester ~= 0
            %new_graph = addedge(new_graph, ancester, current);
        end
        if ancester ~= 0
            new_graph = addedge(new_graph, ancester, current);
        end
        ancester = current;
    end
    
    % every even element is used to add overlap
    if ~mod(i, 2) % even
    end
end

figure
plot(new_graph)

a =1 




for i =1:size(T,1)
    
    current = current+1;
    
    % process edge
    edge_source = T{i,3}(1);
    edge_target = T{i,3}(2);
    
    % get the 3D points related to this submap
    pointcloud_subset_current = pointcloud(i == maps_id, :);
    pointcloud_subset_next = pointcloud(i+1 == maps_id, :);
    if i > 1 % handle the case of the root
        pointcloud_subset_previous = pointcloud(i-1 == maps_id, :);
    else
        pointcloud_subset_previous = zeros(0,3);
    end
    
    
    % every odd element is used for generating a map
    if mod(i, 2) % odd
        current = current +1;
        new_graph = addnode(new_graph, 1);
        submaps{end+1} = [pointcloud_subset_previous;
                          pointcloud_subset_current;
                          pointcloud_subset_next];
        
        % add edges to ancester
        if ancester ~= 0
            new_graph = addedge(new_graph, ancester, current);
        end
        
        ancester = current;
    end
    
    % every even element is used to add overlap
    if ~mod(i, 2) % even
        submap_temp = submaps{end}.Location;
        submap_temp = [submap_temp;             % here stack the points 
                       pointcloud_subset];      % into the ancester submap
    end
end

figure
plot(new_graph)



unique_id = unique(maps_id);

reconstruction = zeros(0,3);




















% 
% for i = 1:length(unique_id)
%     locations = ptCloud.Location;
%     submap = locations(i == maps_id, :);
%     
%     reconstruction = [reconstruction; submap];
%     
%     figure(1)
%     subplot 121;
%     
%     ptCloud_submap = pointCloud(submap);
%     
%     pcshow(ptCloud_submap)
%     
%     subplot 122;
%     ptCloud_submap = pointCloud(reconstruction);
%     
%     pcshow(ptCloud_submap)
%     
%     pause(0.2)
% end
