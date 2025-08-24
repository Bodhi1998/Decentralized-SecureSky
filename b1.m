% Clear workspace
clear; clc; close all;

% Parameters for the 3D trajectory
t = linspace(0, 10*pi, 500);  % Time parameter
z_hover = 2;                  % Base Z coordinate for hovering

% Initialize UAV positions
numUAVs = 3;
uavPaths = cell(numUAVs, 1);
colors = lines(numUAVs);

% Energy parameters
max_energy = 100;
energy_consumption_rate = 0.1;
uav_energy = max_energy * ones(numUAVs, 1);

% Wind parameters
wind_speed = 0.5;
wind_direction = [1; 1; 0] / norm([1; 1; 0]);  % Normalized vector

% Obstacle parameters
num_obstacles = 3;
obstacle_positions = 10 * rand(3, num_obstacles) - 5;  % Random positions
obstacle_radii = 0.5 + rand(1, num_obstacles);  % Random sizes

% Create the figure and set up the 3D plot
fig = figure;
hold on;
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('UAV Positions in 3D with Obstacles');
axis([-7 7 -7 7 0 8]);
view(3);

% Add Ground Control Station (GCS) at a fixed position on the ground
GCS_position = [0, 0, 0];  
plot3(GCS_position(1), GCS_position(2), GCS_position(3), 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
text(GCS_position(1), GCS_position(2), GCS_position(3) + 0.2, 'GCS', 'FontSize', 12, 'Color', 'green');

% Plot obstacles
[X, Y, Z] = sphere(20);
obstacle_handles = gobjects(num_obstacles, 1);
for i = 1:num_obstacles
    obstacle_handles(i) = surf(obstacle_radii(i)*X + obstacle_positions(1,i), ...
                               obstacle_radii(i)*Y + obstacle_positions(2,i), ...
                               obstacle_radii(i)*Z + obstacle_positions(3,i));
    set(obstacle_handles(i), 'FaceColor', [0.7 0.7 0.7], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end

% Initialize UAV markers
uavMarkers = gobjects(numUAVs, 1);
for i = 1:numUAVs
    uavMarkers(i) = plot3(0, 0, 0, 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:), 'MarkerSize', 8);
    text(0, 0, 0, sprintf('  UAV %d', i), 'Color', colors(i,:), 'FontWeight', 'bold');
end

legend([uavMarkers; obstacle_handles(1); plot3(NaN,NaN,NaN,'ks','MarkerFaceColor','g')], ...
       {'UAV 1', 'UAV 2', 'UAV 3', 'Obstacle', 'GCS'}, 'Location', 'best');

% Print simulation information in the command window
fprintf('Simulation of %d UAVs with adaptive trajectories and obstacle avoidance\n', numUAVs);
fprintf('Ground Control Station (GCS) position: (%.2f, %.2f, %.2f)\n\n', GCS_position);

% Animation loop
for k = 1:length(t)
    % Update adaptive parameters
    amplitude = 1.5 + 0.5 * sin(0.1 * t(k));
    frequency = 1 + 0.2 * cos(0.05 * t(k));
    
    % Wind effect
    wind_effect = wind_speed * wind_direction * sin(0.1 * t(k));
    
    for i = 1:numUAVs
        % Generate base trajectory
        base_path = [amplitude * sin(frequency * t(k) + (i * pi / numUAVs));
                     amplitude * cos(frequency * t(k) + (i * pi / numUAVs)) + i * 0.5;
                     z_hover + i*0.2 + amplitude * 0.2 * sin(0.5 * frequency * t(k))];
        
        % Apply wind effect
        current_position = base_path + wind_effect;
        
        % Obstacle avoidance
        for j = 1:num_obstacles
            vec_to_obstacle = current_position - obstacle_positions(:,j);
            distance = norm(vec_to_obstacle);
            if distance < obstacle_radii(j) + 1  % If too close to obstacle
                avoidance_vector = vec_to_obstacle / distance;  % Normalized vector away from obstacle
                current_position = current_position + avoidance_vector * (obstacle_radii(j) + 1 - distance);
            end
        end
        
        % Energy-aware behavior
        uav_energy(i) = uav_energy(i) - energy_consumption_rate;
        if uav_energy(i) < 0.3 * max_energy
            % Return to GCS if energy is low
            current_position = current_position + 0.05 * (GCS_position' - current_position);
        end
        
        % Update UAV position
        set(uavMarkers(i), 'XData', current_position(1), 'YData', current_position(2), 'ZData', current_position(3));
        
        % Update UAV label
        delete(findobj(gca, 'Type', 'text', 'String', sprintf('UAV %d', i)));
        text(current_position(1), current_position(2), current_position(3), sprintf('  UAV %d (E: %.0f%%)', i, uav_energy(i)), 'Color', colors(i,:), 'FontWeight', 'bold');
        
        % Store path for later use if needed
        uavPaths{i}(:,k) = current_position;
    end
    
    % Update the plot
    drawnow
    
    % Print current positions every 50 steps
    if mod(k, 50) == 0
        fprintf('Time step: %.2f\n', t(k));
        for i = 1:numUAVs
            fprintf('  UAV %d position: (%.2f, %.2f, %.2f), Energy: %.0f%%\n', i, uavPaths{i}(1,k), uavPaths{i}(2,k), uavPaths{i}(3,k), uav_energy(i));
        end
        fprintf('\n');
    end
    
    % Pause for smoother animation
    pause(0.01);
end

% Print final positions
fprintf('Final positions:\n');
for i = 1:numUAVs
    fprintf('UAV %d: (%.2f, %.2f, %.2f), Final Energy: %.0f%%\n', i, uavPaths{i}(1,end), uavPaths{i}(2,end), uavPaths{i}(3,end), uav_energy(i));
end

fprintf('\nAnimation complete. Close the figure window to end the simulation.\n');

% Wait for the user to close the figure
waitfor(fig);

fprintf('Simulation ended.\n');