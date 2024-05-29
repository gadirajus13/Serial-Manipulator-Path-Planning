function [path, V, E] = manipulator_PRM(qI, qG, n, K, O, link_lengths)
    V = [];
    E = [];
    adj_mat = [];
    
    % Populate V with n random configurations
    while size(V, 1) < n + 2
        qrand = 2*pi*rand(1, length(qI)) - pi; % Joints go from -pi to pi
        
        % Generate q_rand until it is collision free
        if collision_free(qrand, O, link_lengths)
            V = [V; qrand];
        end
    end
    
    % Add initial and goal configurations to vertices list
    V = [V; qI; qG];
    
    % Loop through all configs to find nearest neighbors
    for q = 1:size(V, 1)
        q_neighbors = k_nearest_neighbors(V(q, :), V, K);
        
        for i = 1:size(q_neighbors, 1)
            if path_collision_free(V(q, :), q_neighbors(i, :), O, link_lengths)
                E = [E; [V(q, :); q_neighbors(i, :)]];
                
                % Find corresponding index for each configuration
                q_index = find(ismember(V, V(q, :), 'rows'));
                qprime_index = find(ismember(V, q_neighbors(i, :), 'rows'));
                
                % Update adjacency matrix for given configurations
                adj_mat(q_index, qprime_index) = norm(V(q, :) - q_neighbors(i, :));
                adj_mat(qprime_index, q_index) = norm(V(q, :) - q_neighbors(i, :));
            end
        end
    end
    
    % Run Dijkstra on adjacency matrix to find optimal path
    start_idx = find(ismember(V, qI, 'rows'));
    goal_idx = find(ismember(V, qG, 'rows'));
    [~, ~, path_index] = dijkstra(adj_mat, start_idx, goal_idx);
    
    % Get vertices at given indices in path
    path = V(path_index, :);
end

% Check if qrand is in collision or not with all obstacles
function collision_free = collision_free(qrand, Obs, link_lengths)
    collision_free = true;
    for obs = 1:length(Obs)
        if isCollision(qrand, link_lengths, Obs{obs})
            collision_free = false;
            break;
        end
    end
end

% Check if path q to q_prime (neighbor) is in collision or not with all obstacles
function collision_free = path_collision_free(q, q_prime, Obs, link_lengths)
    collision_free = true;
    numSteps = 20; % Number of intermediate configurations to check
    qs = interpolate(q, q_prime, numSteps);
    
    % Check each obstacle for colllision
    for obs = 1:length(Obs)
        % Check each intermediate step for collision with current obstacle
        for i = 1:numSteps
            if isCollision(qs(i, :), link_lengths, Obs{obs})
                collision_free = false;
                return;
            end
        end
    end
end

% Find k closest neighbors to configuration q in vertex set V
function qNear = k_nearest_neighbors(q, V, K)
    dists = sum((V - q).^2, 2);
    [~, sorted_indices] = mink(dists, K);
    qNear = V(sorted_indices, :);
end

% Function to check collision between manipulator and obstacle
function collision = isCollision(q, link_lengths, obstacle)
    collision = false;
    % Base of Manipulator Located at 0,0
    x = 0;
    y = 0;
    % Number of intermediate points to check along each link
    num_points = 20;
    
    % Forward kinematics to get each joint/link location
    for i = 1:length(q)
        x_next = x + link_lengths(i) * cos(sum(q(1:i)));
        y_next = y + link_lengths(i) * sin(sum(q(1:i)));
        
        % Check intermediate points along the link for each joint
        for j = 1:num_points
            t = (j - 1) / (num_points - 1);
            x_interp = x + t * (x_next - x);
            y_interp = y + t * (y_next - y);
            
            if inpolygon(x_interp, y_interp, obstacle(:, 1), obstacle(:, 2))
                collision = true;
                return;
            end
        end
        
        x = x_next;
        y = y_next;
    end
end

% Function to interpolate between two configurations
function [qs] = interpolate(q1, q2, numSteps)
    qs = zeros(numSteps, length(q1));
    for i = 1:numSteps
        t = (i - 1) / (numSteps - 1);
        qs(i, :) = q1 + t * (q2 - q1);
    end
end