% --- Step 1: Define the Drone and Ground Station ---
clc;

%----------Encryption and Decryption Key-------------
key = fileread('endeckey.txt');

% Drone properties
drone.ID = 'ABC123';  % Unique ID for the drone (Remote ID)
drone.latitude = 37.7749; % Starting location (San Francisco Latitude)
drone.longitude = -122.4194; % Starting location (San Francisco Longitude)
drone.altitude = 100;  % Altitude in meters

% Increase the drone speed for longer distance movement
drone.speed = 0.0007; % Larger speed for longer distance movement
drone.flightZone = 'Permitted'; % Permitted or Restricted flight zone
drone.operatorID = 'Operator1';   

% Ground Station (GCS) properties
groundStation.range = 1000;  % The range in meters within which it can receive signals
groundStation.latitude = 37.7749; % Location of the ground station
groundStation.longitude = -122.4194; % Location of the ground station

% --- Step 2: Define the Database ---

% Create a simple database of registered drones
droneDatabase = struct('ID', {'ABC123', 'DEF456'}, ...
                       'operator', {'Operator1', 'Operator2'}, ...
                       'registrationStatus', {'Valid', 'Valid'}, ...
                       'allowedZones', {'Permitted', 'Restricted'});

% --- Step 3: Setup the 2D Visualization ---

% Initialize figure
f = figure;
hold on;
grid on;
axis equal;

% Plot the ground station
plot(groundStation.longitude, groundStation.latitude, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
text(groundStation.longitude, groundStation.latitude, ' Ground Station', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

% Plot the ground station's communication range as a circle
theta = linspace(0, 2*pi, 100);
stationRangeLat = groundStation.latitude + (groundStation.range/111320) * cos(theta); % Convert meters to lat
stationRangeLon = groundStation.longitude + (groundStation.range/(111320*cosd(groundStation.latitude))) * sin(theta); % Convert meters to lon
plot(stationRangeLon, stationRangeLat, 'r--', 'LineWidth', 1);

% Initialize the drone plot (with real-time position update)
dronePlot = plot(drone.longitude, drone.latitude, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
flightPath = plot(NaN, NaN, 'b-', 'LineWidth', 2);

% Set labels and title
xlabel('Longitude');
ylabel('Latitude');
title('Drone and Ground Station 2D Plane Visualization');

% --- Create a UI element to display Remote ID on the side ---
% This will display the Remote ID data on the side of the graph
remoteIDDisplay = uicontrol('Style', 'text', ...
    'Units', 'normalized', ...
    'Position', [0.8 0.5 0.18 0.4], ... % Adjust position (side of the graph)
    'BackgroundColor', 'white', ...
    'FontSize', 10, ...
    'HorizontalAlignment', 'left', ...
    'String', 'Remote ID: Waiting for drone in range...');

% --- Step 4: Simulate the Drone Broadcasting its Information ---

% Broadcasting interval (seconds)
broadcastInterval = 1; % Remote ID will be generated every 1 second
smoothStep = 0.05; % Small step for smooth movement (20 Hz)

% Flight simulation time (seconds)
simulationTime = 20000;

% Initialize arrays to store drone's broadcast information over time
time = 0:broadcastInterval:simulationTime;     
latitudeLog = zeros(1, length(time));
longitudeLog = zeros(1, length(time));

% Initialize a counter for full-second broadcast
fullSecondCounter = 0;

% Random movement control
randomDirection = @(maxVal) (rand()*2 - 1) * maxVal;  % Random direction function

disp('Starting Drone Flight and Broadcasting Simulation:');
for t = 0:smoothStep:simulationTime
    % --- Step 5: Random and Smooth Drone Movement ---
    % Simulate drone's random movement in larger steps for longer distances
    drone.latitude = drone.latitude + randomDirection(drone.speed);  % Random latitude change
    drone.longitude = drone.longitude + randomDirection(drone.speed); % Random longitude change
    
    % Update the drone's position on the plot
    set(dronePlot, 'XData', drone.longitude, 'YData', drone.latitude);
    
    % Log the drone's current position (for flight path) every 1 second
    if mod(t, broadcastInterval) == 0
        fullSecondCounter = fullSecondCounter + 1;
        latitudeLog(fullSecondCounter) = drone.latitude;
        longitudeLog(fullSecondCounter) = drone.longitude;
        set(flightPath, 'XData', longitudeLog(1:fullSecondCounter), 'YData', latitudeLog(1:fullSecondCounter));
    end

    % --- Step 6: Simulate Ground Station Receiving Information ---
    
    % Check if drone is within range of the ground station
    distanceToGroundStation = haversine(drone.latitude, drone.longitude, ...
                                        groundStation.latitude, groundStation.longitude);
    
    if distanceToGroundStation <= groundStation.range
        % Ground station receives the broadcast
        set(dronePlot, 'MarkerFaceColor', 'g'); % Change drone color to green (in range)

        % Authenticate the Drone
        authenticated = false;
        for i = 1:length(droneDatabase)
            if strcmp(drone.ID, droneDatabase(i).ID)
                % Check if drone is registered and allowed to fly in the zone
                if strcmp(droneDatabase(i).registrationStatus, 'Valid') && ...
                   strcmp(drone.flightZone, droneDatabase(i).allowedZones)
                    authenticated = true;
                    fprintf('Drone ID: %s authenticated successfully!\n', drone.ID);
                else
                    fprintf('Drone ID: %s is flying in a restricted zone or unregistered!\n', drone.ID);
                end
                break;
            end
        end
        
        % If authenticated, show Remote ID details, else display 'null'
        if authenticated
            % Generate and display the Remote ID at the side of the graph.
            %------------------------------------------------------------------------------------------------------

                
            %**************   time Calculation Starts    ***********************

            % Define the words
            word1 = drone.ID;            word2 = drone.latitude;            word3 = drone.longitude;            word4 = drone.altitude;
            
            set(remoteIDDisplay, 'String', sprintf('Remote ID: %s\nLatitude: %.4f\nLongitude: %.4f\nAltitude: %.2f m', ...
                word1, word2, word3, word4));
            


            %-----------------------------------------------------------------------------------------------
        else
            % If authentication fails, display 'null' values.
            set(remoteIDDisplay, 'String', sprintf('Remote ID: %s\nLatitude: null\nLongitude: null\nAltitude: null', drone.ID));
        end
        
    else
        % Drone is out of range of the ground station.
        set(dronePlot, 'MarkerFaceColor', 'r'); % Change drone color to blue (out of range).
        set(remoteIDDisplay, 'String', 'Remote ID: Drone out of range'); % Clear the Remote ID display when out of range.
    end

    % Pause for smooth movement.
    pause(smoothStep);
end

% --- Helper function: Calculate distance using Haversine formula ---
function distance = haversine(lat1, lon1, lat2, lon2)
    R = 6371000; % Radius of the Earth in meters.
    phi1 = deg2rad(lat1);
    phi2 = deg2rad(lat2);
    deltaPhi = deg2rad(lat2 - lat1);
    deltaLambda = deg2rad(lon2 - lon1);
    
    a = sin(deltaPhi/2) * sin(deltaPhi/2) + ...
        cos(phi1) * cos(phi2) * sin(deltaLambda/2) * sin(deltaLambda/2);
    c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    distance = R * c; % Distance in meters.
end