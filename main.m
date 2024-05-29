% Initialize Environment
workspace = [-6, 6, -6, 6]; % Workspace dimensions [xmin, xmax, ymin, ymax]
obstacles = {[2, 2; 4, 2; 4, 4; 2, 4], ... % Obstacle 1
             [-1, 0; -1, 2; -4, 2; -4, 0], ... % Obstacle 2
             [2, -4; 2, -2; 0, -2; 0, -4]}; % Obstacle 3
n = 4; % Number of links
link_lengths = [2, 1.5, 1, 1]; % Link lengths
qI = [-pi/6, 0, 0, -pi/2]; % Initial configuration
qG = [pi/4, pi/3, -pi/6, pi/2]; % Goal configuration

%% RRT for Serial Manipulator
% RRT Parameters
dq = 0.5; % Step size
tolerance = 1; % Tolerance for reaching the goal

% Create a video writer object to create movie file
video_rrt = VideoWriter('rrt_movie','MPEG-4');
video_rrt.FrameRate = 5; % Set the frame rate
open(video_rrt); 

% Plot the environment
figure;
hold on;
xlim(workspace(1:2));
ylim(workspace(3:4));

% Set plot properties
set(gca, 'XTick', workspace(1):1:workspace(2));
set(gca, 'YTick', workspace(3):1:workspace(4));
grid on;
xlabel('X');
ylabel('Y');
title('RRT Path Planning');

% Plot obstacles
obstacle_colors = ['r', 'g', 'c'];
for i = 1:length(obstacles)
    obs = obstacles{i};
    patch(obs(:,1), obs(:,2), obstacle_colors(i));
end

% Plot initial and goal configurations
plotManipulator(qI, link_lengths, 'k', 'LineWidth', 2); % Intial in Black
plotManipulator(qG, link_lengths, 'y', 'LineWidth', 2); % Goal in Yellow

% Build RRT
[path, V, E] = manipulator_RRT(qI, qG, dq, obstacles, link_lengths, tolerance);

% Plot path
if ~isempty(path)
    for i = 1:size(path, 1)
        q = path(i, :);
        plotManipulator(q, link_lengths, 'b');
        pause(1);
        frame = getframe(gcf); % Capture the current frame
        writeVideo(video_rrt, frame); % Write the frame to the video
    end
end

hold off;
axis equal;

% Create obstacle labels for the legend
obstacle_labels = cell(1, length(obstacles));
for i = 1:length(obstacles)
    obstacle_labels{i} = sprintf('Obstacle %d', i);
end
legend([obstacle_labels, 'Initial Configuration', 'Goal Configuration','Manipulator Path']);

close(video_rrt); % Close the video writer

%% PRM for Serial Manipulator
% PRM Parameters
n = 1000; % Number of nodes in the roadmap
K = 10; % Number of nearest neighbors to consider

% Create a video writer object to create movie file
video_prm = VideoWriter('prm_movie','MPEG-4');
video_prm.FrameRate = 1; % Set the frame rate 
open(video_prm);

% Plot the environment
figure;
hold on;
xlim(workspace(1:2));
ylim(workspace(3:4));
% Set plot properties
set(gca, 'XTick', workspace(1):1:workspace(2));
set(gca, 'YTick', workspace(3):1:workspace(4));
grid on;
xlabel('X');
ylabel('Y');
title('PRM Path Planning');

% Plot obstacles
obstacle_colors = ['r', 'g', 'c'];
for i = 1:length(obstacles)
    obs = obstacles{i};
    patch(obs(:,1), obs(:,2), obstacle_colors(i));
end

% Plot initial and goal configurations
plotManipulator(qI, link_lengths, 'k', 'LineWidth', 2); % Intial in Black
plotManipulator(qG, link_lengths, 'y', 'LineWidth', 2); % Goal in Yellow

% Build PRM
[path, V, E] = manipulator_PRM(qI, qG, n, K, obstacles, link_lengths);

% Plot path
if ~isempty(path)
    for i = 1:size(path, 1)
        q = path(i, :);
        plotManipulator(q, link_lengths, 'm', 'LineWidth', 2);
        pause(1);
        frame = getframe(gcf); % Capture the current frame
        writeVideo(video_prm, frame); % Write the frame to the video
    end
end

hold off;
axis equal;

% Create obstacle labels for the legend
obstacle_labels = cell(1, length(obstacles));
for i = 1:length(obstacles)
    obstacle_labels{i} = sprintf('Obstacle %d', i);
end
legend([obstacle_labels, 'Initial Configuration', 'Goal Configuration', 'Manipulator Path']);

close(video_prm); % Close the video writer


%% Function to plot the manipulator
function plotManipulator(q, link_lengths, color, varargin)
    x = 0;
    y = 0;
    plot(x, y, 'o', 'MarkerSize', 5, 'MarkerFaceColor', color);
    % Calculate each joint location using forward kinematics
    for i = 1:length(q)
        x_next = x + link_lengths(i) * cos(sum(q(1:i)));
        y_next = y + link_lengths(i) * sin(sum(q(1:i)));
        % Create line segment to connect previous and current joint
        line([x, x_next], [y, y_next], 'Color', color, varargin{:});
        x = x_next;
        y = y_next;
    end
end