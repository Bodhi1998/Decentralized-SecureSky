% --- Step 1: Define Drones and Ground Station ---
clc;

%----------Encryption and Decryption Key-------------
key = fileread('endeckey.txt');

% Define properties for 5 drones (3 authorized, 2 unauthorized)
drone(1).ID = 'ABC123'; % Authorized Drone 1
drone(2).ID = 'DEF456'; % Authorized Drone 2
drone(3).ID = 'GHI789'; % Authorized Drone 3
drone(4).ID = 'JKL012'; % Unauthorized Drone 4 (Invalid registration)
drone(5).ID = 'MNO345'; % Unauthorized Drone 5 (No permission)

% Initial positions of the drones (starting around the same area)
initial_latitudes = [37.7749, 37.7740, 37.7750, 37.7730, 37.7729];
initial_longitudes = [-122.4194, -122.4185, -122.4200, -122.4170, -122.4165];
initial_altitudes = [100, 120, 110, 105, 130];

for i = 1:5
    drone(i).latitude = initial_latitudes(i);  % Starting latitude
    drone(i).longitude = initial_longitudes(i);  % Starting longitude
    drone(i).altitude = initial_altitudes(i);  % Altitude in meters
    drone(i).speed = 0.0007;  % Speed for movement
    drone(i).flightZone = 'Permitted';  % Default flight zone
    drone(i).operatorID = ['Operator', num2str(i)];  % Unique operator ID
    drone(i).licensePlate = '';  % License plate-style identifier (to be assigned)
end

% Ground Station (GCS) properties
groundStation.range = 1000;  % Range in meters
groundStation.latitude = 37.7749; % Ground station location
groundStation.longitude = -122.4194; % Ground station location

% --- Step 2: Define the Database ---
droneDatabase = struct('ID', {'ABC123', 'DEF456', 'GHI789', 'JKL012', 'MNO345'}, ...
                       'operator', {'Operator1', 'Operator2', 'Operator3', 'Operator4', 'Operator5'}, ...
                       'registrationStatus', {'Valid', 'Valid', 'Valid', 'InValid', 'Valid'}, ...
                       'allowedZones', {'Permitted', 'Permitted', 'Permitted', 'Permitted', 'No'}, ...
                       'licensePlate', {'', '', '', '', ''}); % License plate storage

% --- Step 3: Setup the 2D Visualization ---
f = figure('Name', 'UAV Simulation', 'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
hold on;
grid on;
axis equal;
set(gca, 'FontSize', 12, 'FontWeight', 'bold', 'GridLineStyle', '--', 'GridAlpha', 0.7);

% Plot the ground station
plot(groundStation.longitude, groundStation.latitude, 'rx', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Ground Station');
text(groundStation.longitude, groundStation.latitude, ' Ground Station', ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 14, 'FontWeight', 'bold');

% Plot the ground station's communication range as a circle
theta = linspace(0, 2*pi, 100);
stationRangeLat = groundStation.latitude + (groundStation.range/111320) * cos(theta); % Convert meters to lat
stationRangeLon = groundStation.longitude + (groundStation.range/(111320*cosd(groundStation.latitude))) * sin(theta); % Convert meters to lon
plot(stationRangeLon, stationRangeLat, 'r--', 'LineWidth', 2, 'DisplayName', 'GCS Range');

% Initialize the drone plots (one for each drone)
for i = 1:5
    drone(i).plot = plot(drone(i).longitude, drone(i).latitude, 'bo', 'MarkerSize', 12, ...
                         'LineWidth', 2, 'MarkerFaceColor', 'b', 'DisplayName', sprintf('Drone %d (Out of Range)', i));
    drone(i).flightPath = plot(NaN, NaN, 'b-', 'LineWidth', 2.5, 'HandleVisibility', 'off');
end

% Set labels and title
xlabel('Longitude', 'FontSize', 16, 'FontWeight', 'bold');
ylabel('Latitude', 'FontSize', 16, 'FontWeight', 'bold');
title('Decentralized SecureSky: Multi-UAV Simulation', 'FontSize', 18, 'FontWeight', 'bold');

% Add legend
legend('show', 'Location', 'northeast', 'FontSize', 12, 'Box', 'on', 'EdgeColor', [0.2 0.2 0.2]);

% Create UI elements for each drone's Remote ID display
for i = 1:5
    drone(i).remoteIDDisplay = uicontrol('Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.8 0.8 - (i-1)*0.1 0.18 0.08], ...
        'BackgroundColor', 'white', ...
        'FontSize', 10, ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', ...
        'String', sprintf('Drone %d: Initializing...', i));
end

% --- Step 4: Simulate Drone Broadcasting ---
broadcastInterval = 1; % Remote ID broadcast every 1 second
smoothStep = 0.05; % Smooth movement (20 Hz)
simulationTime = 2000; % Simulation time (seconds)

% Initialize arrays for each drone's movement
for i = 1:5
    drone(i).time = 0:broadcastInterval:simulationTime;
    drone(i).latitudeLog = zeros(1, length(drone(i).time));
    drone(i).longitudeLog = zeros(1, length(drone(i).time));
    drone(i).fullSecondCounter = 0;  % Counter for full-second broadcast
end

randomDirection = @(maxVal) (rand()*2 - 1) * maxVal;  % Random direction function

disp('Starting Multiple Drone Flight Simulation:');
for t = 0:smoothStep:simulationTime
    % --- Step 5: Random and Smooth Drone Movement ---
    for i = 1:5
        % Simulate random movement for each drone
        drone(i).latitude = drone(i).latitude + randomDirection(drone(i).speed);
        drone(i).longitude = drone(i).longitude + randomDirection(drone(i).speed);

        % Update drone's position on the plot
        set(drone(i).plot, 'XData', drone(i).longitude, 'YData', drone(i).latitude);

        % Log position every 1 second
        if mod(t, broadcastInterval) == 0
            drone(i).fullSecondCounter = drone(i).fullSecondCounter + 1;
            drone(i).latitudeLog(drone(i).fullSecondCounter) = drone(i).latitude;
            drone(i).longitudeLog(drone(i).fullSecondCounter) = drone(i).longitude;
            set(drone(i).flightPath, 'XData', drone(i).longitudeLog(1:drone(i).fullSecondCounter), ...
                                      'YData', drone(i).latitudeLog(1:drone(i).fullSecondCounter));
        end

        % --- Step 6: Simulate Drone Sending ID to GCS ---
        distanceToGroundStation = haversine(drone(i).latitude, drone(i).longitude, ...
                                            groundStation.latitude, groundStation.longitude);
        
        if distanceToGroundStation <= groundStation.range
            % Drone sends its ID to GCS
            authenticated = false;
            for j = 1:length(droneDatabase)
                if strcmp(drone(i).ID, droneDatabase(j).ID)
                    % Check registration status and permission
                    if strcmp(droneDatabase(j).registrationStatus, 'Valid') && ...
                       strcmp(drone(i).flightZone, droneDatabase(j).allowedZones)
                        authenticated = true;
                        set(drone(i).plot, 'MarkerFaceColor', 'g', ...
                            'DisplayName', sprintf('Drone %d (Authorized)', i));  % Authorized: Green
                        fprintf('Drone ID: %s is authorized to fly!\n', drone(i).ID);
                        
                        % Assign license plate-style identifier if not already assigned
                        if isempty(drone(i).licensePlate)
                            drone(i).licensePlate = sprintf('UAV-%s%d', char(65+i-1), 123 + i*111);
                            droneDatabase(j).licensePlate = drone(i).licensePlate; % Update database
                            fprintf('Assigned License Plate to Drone %s: %s\n', drone(i).ID, drone(i).licensePlate);
                        end
                    else
                        set(drone(i).plot, 'MarkerFaceColor', 'r', ...
                            'DisplayName', sprintf('Drone %d (Unauthorized)', i));  % Unauthorized: Red
                        fprintf('Drone ID: %s is unauthorized (invalid registration or zone)!\n', drone(i).ID);
                    end
                    
                    % Update Remote ID display immediately
                    if authenticated
                        word1 = drone(i).licensePlate; % Use license plate for identification
                        word2 = num2str(drone(i).latitude);
                        word3 = num2str(drone(i).longitude);
                        word4 = num2str(drone(i).altitude);
                        word5 = drone(i).ID; % Include original ID for reference
                        s = [word1, ' ', word2, ' ', word3, ' ', word4, ' ', word5];
                        q = ['License- ', word1, ' | Latitude - ', word2, ' | Longitude - ', word3, ...
                             ' | Altitude - ', word4, ' | Drone ID - ', word5];
                        disp(q);

                        encryptedData = aes_encrypt(s, key);
                        fileID = fopen('data_encrypted.txt', 'w');
                        fprintf(fileID, '%s', encryptedData);
                        fclose(fileID);

                        % Add encrypted file to IPFS
                        [status, cmdout] = system('ipfs add data_encrypted.txt');
                        if status == 0
                            parts = strsplit(cmdout);
                            cid = parts{2}; % Adjust index based on IPFS output
                            disp('Present Remote ID Hash value of Blockchain - ');
                            disp(['CID: ', cid]);
                            fid = fopen('blockchainkey.txt', 'w');
                            fwrite(fid, cid);
                            fclose(fid);
                        else
                            disp('Error adding file to IPFS');
                            set(drone(i).remoteIDDisplay, 'String', sprintf('ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nLicense: %s', ...
                                drone(i).ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude), drone(i).licensePlate));
                            continue; % Skip to next drone if IPFS fails
                        end

                        % Simulate GCS retrieving and decrypting data
                        data = fileread('blockchainkey.txt');
                        [status, ss] = system(['ipfs cat ', data]);
                        if status == 0
                            decryptedData = aes_decrypt(ss, key);
                            parts = strsplit(decryptedData, ' ');
                            fprintf('DroneWITH License: %s (ID: %s) authenticated successfully!\n', parts{1}, parts{5});
                            license = parts{1};
                            lat = parts{2};
                            lon = parts{3};
                            alt = parts{4};
                            idd = parts{5};
                            set(drone(i).remoteIDDisplay, 'String', sprintf('License: %s\nLat: %s\nLon: %s\nAlt: %s m\nID: %s', ...
                                license, lat, lon, alt, idd));
                        else
                            disp('Error retrieving file from IPFS');
                            set(drone(i).remoteIDDisplay, 'String', sprintf('ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nLicense: %s', ...
                                drone(i).ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude), drone(i).licensePlate));
                        end
                    else
                        set(drone(i).remoteIDDisplay, 'String', sprintf('ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nLicense: Unauthorized', ...
                            drone(i).ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude)));
                    end
                    break;
                end
            end
        else
            set(drone(i).plot, 'MarkerFaceColor', 'b', ...
                'DisplayName', sprintf('Drone %d (Out of Range)', i));  % Out of range: Blue
            set(drone(i).remoteIDDisplay, 'String', sprintf('ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nLicense: Out of range', ...
                drone(i).ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude)));
        end
    end
    pause(smoothStep);  % Pause for smooth movement
end

% --- Helper function: Haversine Distance Calculation ---
function distance = haversine(lat1, lon1, lat2, lon2)
    R = 6371000; % Radius of Earth in meters
    phi1 = deg2rad(lat1);
    phi2 = deg2rad(lat2);
    deltaPhi = deg2rad(lat2 - lat1);
    deltaLambda = deg2rad(lon2 - lon1);

    a = sin(deltaPhi/2) * sin(deltaPhi/2) + ...
        cos(phi1) * cos(phi2) * sin(deltaLambda/2) * sin(deltaLambda/2);
    c = 2 * atan2(sqrt(a), sqrt(1 - a));

    distance = R * c;  % Output distance in meters
end