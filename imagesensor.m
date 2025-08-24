% Parameters
mapSize = 10000; % 10 km x 10 km area
dronePos = [1000, 2000]; % Initial drone position (x, y)
gcsPos = [5000, 5000]; % GCS position (fixed at center)
sensingRadius = 5000; % GCS sensing radius (5 km)
timeSteps = 100; % Number of simulation steps
droneSpeed = 100; % Drone speed per step (in meters)
dt = 1; % Time step duration

% Create a figure for visualization
figure;
axis([0 mapSize 0 mapSize]);
hold on;
grid on;
xlabel('X (meters)');
ylabel('Y (meters)');
title('Drone and GCS Sensing Simulation');

% Plot the GCS and its sensing radius
viscircles(gcsPos, sensingRadius, 'EdgeColor', 'b', 'LineStyle', '--');
plot(gcsPos(1), gcsPos(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
text(gcsPos(1)+100, gcsPos(2), 'GCS', 'Color', 'blue', 'FontSize', 12);

% Simulation loop
for t = 1:timeSteps
    % Update drone's position (simple random walk or predefined trajectory)
    angle = rand() * 2 * pi; % Random direction
    dronePos = dronePos + droneSpeed * [cos(angle), sin(angle)] * dt;
    
    % Ensure drone stays within bounds
    dronePos = min(max(dronePos, 0), mapSize);
    
    % Plot the drone
    plot(dronePos(1), dronePos(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    
    % Check if drone is within GCS sensing radius
    distanceToGCS = norm(dronePos - gcsPos);
    if distanceToGCS <= sensingRadius
        % Drone detected
        plot(dronePos(1), dronePos(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
        disp(['Drone detected at time step ', num2str(t), ...
              ' at position: ', num2str(dronePos)]);
    else
        % Drone not detected
        disp(['Drone not detected at time step ', num2str(t)]);
    end
    
    % Pause for visualization
    pause(0.1);
end

