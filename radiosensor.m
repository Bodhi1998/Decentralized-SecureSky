% Parameters
mapSize = 10000; % 10 km x 10 km area
dronePos = [3000, 3000]; % Initial drone position (x, y)
gcsPos = [5000, 5000]; % GCS position (fixed at center)
sensingRadius = 5000; % GCS sensing radius (5 km)
timeSteps = 100; % Number of simulation steps
droneSpeed = 1000; % Drone speed per step (in meters)
dt = 1; % Time step duration
radarPulseRate = 5; % Radar pulse every 5 time steps
radarNoiseLevel = 0.2; % Noise factor for radar sensing
signalStrengthThreshold = -70; % Signal strength threshold for RF detection (dBm)

% Radar constants
c = 3e8; % Speed of light (m/s)
frequency = 2.4e9; % Frequency of RF signal (2.4 GHz)

% Free-space path loss model
freeSpacePathLoss = @(d) -20*log10(d) - 20*log10(frequency) + 20*log10(c/(4*pi));

% Create a figure for visualization
figure;
axis([0 mapSize 0 mapSize]);
hold on;
grid on;
xlabel('X (meters)');
ylabel('Y (meters)');
title('Drone and GCS Sensing Simulation with Radar and RF');

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
    
    % Calculate distance between drone and GCS
    distanceToGCS = norm(dronePos - gcsPos);
    
    % Simulate RF transmission from the drone
    if distanceToGCS <= sensingRadius
        % Calculate signal strength using free-space path loss
        signalStrength = freeSpacePathLoss(distanceToGCS);
        % Add random noise to signal
        signalStrength = signalStrength + radarNoiseLevel * randn();
        
        % Check if the signal strength is strong enough to be detected
        if signalStrength > signalStrengthThreshold
            % Drone detected via RF signal
            plot(dronePos(1), dronePos(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
            disp(['Drone detected via RF at time step ', num2str(t), ...
                  ' at position: ', num2str(dronePos), ...
                  ' with signal strength: ', num2str(signalStrength), ' dBm']);
        else
            disp(['Drone RF signal too weak at time step ', num2str(t)]);
        end
    end
    
    % Simulate radar pulse (every few steps)
    if mod(t, radarPulseRate) == 0
        % Simulate radar detection with noise
        radarNoise = radarNoiseLevel * randn();
        if distanceToGCS <= sensingRadius + radarNoise
            % Radar detects the drone
            plot(dronePos(1), dronePos(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
            disp(['Drone detected via radar at time step ', num2str(t), ...
                  ' at position: ', num2str(dronePos)]);
        else
            disp(['Radar did not detect drone at time step ', num2str(t)]);
        end
    end
    
    % Pause for visualization
    pause(0.1);
end

