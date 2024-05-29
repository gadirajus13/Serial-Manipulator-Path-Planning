function [path, V, E] = manipulator_RRT(qI, qG, dq, O, link_lengths, tolerance)
    V = qI;
    E = [];
    adj_mat = [];
    solved = false;
    while ~solved
        % Pick a random configuration in C or Cfree
        qrand = 2*pi*rand(1, length(qI)) - pi; % Joints go from -pi to pi
        % Find q_near in G to q_rand
        qnear = nearest_vertex(qrand, V);
        % Select a new configuration qnew from q_near with step size dq
        qnew = newConf(qnear, qrand, dq);
   
        goalPath = true;
        
        % Calculate intermediate steps from qnear to qnew and qnew to goal
        qs = interpolate(qnear, qnew, 20);
        qsGoal = interpolate(qnew, qG, 10);
        
        % Loop through obstacles to check for collisions
        for obs = 1:length(O)

            % Check if Qnew is in Cfree
            if isCollision(qnew, link_lengths, O{obs})
                collision = true;
                break;
            end
            
            % Check collision in intermediate path to new configuration
            % from qnear
            for i=1:length(qs)
                collision = isCollision(qs(i,:), link_lengths, O{obs});
                if collision
                    break;
                end
            end

            % Exit for loop if collision has already been found
            if collision
                break;
            end
            
            % Check if path from qnew to goal exists and dist is within tolerance
            for i=1:length(qsGoal)
                goalCollision = isCollision(qsGoal(i,:), link_lengths, O{obs});
                if goalCollision
                    break;
                end
            end
            if goalCollision || norm(qnew - qG) > tolerance
                goalPath = false;
            end
        end
        
        % If no collisions, add qnew to V and the edge qnew,qnear to E
        if ~collision
            V = [V; qnew];
            E = [E; [qnear; qnew]];
            
            % Add path to adjacency matrix
            qnear_index = find(ismember(V, qnear, 'rows')); % Find index of qnear
            end_index = size(V, 1);
            adj_mat(qnear_index, end_index) = norm(qnear - qnew);
            adj_mat(end_index, qnear_index) = norm(qnear - qnew);
            
            % If path from qnew to qgoal, add qgoal to graph and break
            if goalPath
                V = [V; qG];
                E = [E; [qnew; qG]];
                start_index = size(V, 1) - 1; % Index of qnew
                end_index = size(V, 1); % Index of qG
                adj_mat(start_index, end_index) = norm(qnew - qG);
                adj_mat(end_index, start_index) = norm(qnew - qG);
                solved = true;
                break;
            end
        end
    end
    
    % Run Dijkstra on graph to find optimal path from initial to goal
    [~, ~, path_index] = dijkstra(adj_mat, 1, size(V, 1));
    
    % Get vertices at given indices in path
    path = V(path_index, :);
end

% Function to find nearest vertex in Graph to current vertex
function qNear = nearest_vertex(q, G)
    [~, idx_near] = min(sum((G - q).^2, 2));
    qNear = G(idx_near, :);
end

% Function to find new vertex in direction of qrand from nearest vertex
function qnew = newConf(qnear, qrand, dq)
    dir = (qrand - qnear) / norm(qrand - qnear);
    qnew = qnear + dq * dir;
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