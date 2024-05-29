% Function to implement Dijkstra shortest path algorithm
% Inputs - Adjacency Matrix, Initial Node, Goal Node
function [lowest_cost,prev,path] = dijkstra(G, n_init, n_goal)
    nodesNum = size(G,1); % Number of nodes in graph
    dist = Inf(1,nodesNum); % Initialize all node distances to Inf
    prev = cell(1,nodesNum); % Set all prev elements to Null
    dist(n_init) = 0; % Set distance of initial node to 0
    U = 1:nodesNum; % Initialize open set with
    
    while ismember(n_goal,U)
        [~,nodeIdx] = min(dist(U));
        C = U(nodeIdx); % Set current node to node w/ smallest distance
        U(nodeIdx) = []; % Remove current node from U
        
        % Find all neighbors of C
        neighbors = find(G(C,:)>0);
        
        % Loop through all neighbors to find lowest cost nodes
        for v = neighbors
            % Calculate temp cost of neighbor node
            alt = dist(C) + G(C,v);
       
            % if temp cost is less than current, update
            if alt < dist(v)
             dist(v) = alt;
             prev{v} = C;
            end
        end
        
    end
    path = findPath(prev, n_init,n_goal); % Find shortest path to goal
    lowest_cost = dist(n_goal); % Return cost based of distance
end

% Backtrack from Goal Node to find shortest path
function path = findPath(prev, n_init, n_goal)
    current_node = n_goal;
    path = [];
    while current_node ~= n_init
        path = [current_node, path];
        current_node = prev{current_node};
    end
    path = [n_init,path];
end