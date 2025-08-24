% --- Step 1: Define Drones and Ground Station ---
clc;

%----------Encryption and Decryption Key-------------
key = fileread('endeckey.txt');

% Define properties for 5 drones
drone(1).ID = 'ABC123'; % Drone 1 ID
drone(2).ID = 'DEF456'; % Drone 2 ID
drone(3).ID = 'GHI789'; % Drone 3 ID
drone(4).ID = 'JKL012'; % Drone 4 ID
drone(5).ID = 'MNO345'; % Drone 5 ID

% Initial positions of the drones (starting around the same area)
initial_latitudes = [37.7749, 37.7740, 37.7750, 37.7730, 37.7729];
initial_longitudes = [-122.4194, -122.4185, -122.4200, -122.4170, -122.4165];
initial_altitudes = [100, 120, 110, 105, 130];

for i = 1:5
    drone(i).latitude = initial_latitudes(i);  % Starting latitude
    drone(i).longitude = initial_longitudes(i);  % Starting longitude
    drone(i).altitude = initial_altitudes(i);  % Altitude in meters
    drone(i).speed = 0.0007;  % Speed for movement
    drone(i).flightZone = 'Permitted';  % All drones are in permitted zones for simplicity
    drone(i).operatorID = ['Operator', num2str(i)];  % Unique operator ID for each drone
end

% Ground Station (GCS) properties
groundStation.range = 1000;  % Range in meters
groundStation.latitude = 37.7749; % Ground station location
groundStation.longitude = -122.4194; % Ground station location

% --- Step 2: Define the Database ---
droneDatabase = struct('ID', {'ABC123', 'DEF456', 'GHI789', 'JKL012', 'MNO345'}, ...
                       'operator', {'Operator1', 'Operator2', 'Operator3', 'Operator4', 'Operator5'}, ...
                       'registrationStatus', {'Valid', 'Valid', 'Valid', 'Valid', 'InValid'}, ...
                       'allowedZones', {'Permitted', 'Permitted', 'Permitted', 'Permitted', 'No'});

% --- Step 3: Setup the 2D Visualization ---
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

% Initialize the drone plots (one for each drone)
for i = 1:5
    drone(i).plot = plot(drone(i).longitude, drone(i).latitude, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    drone(i).flightPath = plot(NaN, NaN, 'b-', 'LineWidth', 2);  % Path plot for each drone
end

% Set labels and title
xlabel('Longitude');
ylabel('Latitude');
title('Multiple Drones and Ground Station 2D Visualization');

% Create UI elements for each drone's Remote ID display
for i = 1:5
    drone(i).remoteIDDisplay = uicontrol('Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.8 0.8 - (i-1)*0.1 0.18 0.08], ... % Adjust position for each drone
        'BackgroundColor', 'white', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'left', ...
        'String', sprintf('Remote ID %d: Waiting...', i));
end

% --- Step 4: Simulate Drone Broadcasting ---
broadcastInterval = 1; % Remote ID will be generated every 1 second
smoothStep = 0.05; % Small step for smooth movement (20 Hz)
simulationTime = 2000; % Flight simulation time (seconds)

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

        % --- Step 6: Simulate Ground Station Receiving Information ---
        distanceToGroundStation = haversine(drone(i).latitude, drone(i).longitude, ...
                                            groundStation.latitude, groundStation.longitude);
        
        if distanceToGroundStation <= groundStation.range
            set(drone(i).plot, 'MarkerFaceColor', 'g');  % Change drone color to green (in range)
            % Authenticate the drone
            authenticated = false;
            for j = 1:length(droneDatabase)
                if strcmp(drone(i).ID, droneDatabase(j).ID)
                    if strcmp(droneDatabase(j).registrationStatus, 'Valid') && ...
                       strcmp(drone(i).flightZone, droneDatabase(j).allowedZones)
                        authenticated = true;
                        
                    else
                        fprintf('Drone ID: %s is in a restricted zone or unregistered!\n', drone(i).ID);
                    end
                    break;
                end
            end
            
            if authenticated
               % Generate and display the Remote ID at the side of the graph.
            %------------------------------------------------------------------------------------------------------

                
            %**************   time Calculation Starts    ***********************

            % Define the words
            word1 = num2str(drone(i).ID);            word2 = num2str(drone(i).latitude);            word3 = num2str(drone(i).longitude);            word4 = num2str(drone(i).altitude);
            s=[word1,' ',word2,' ',word3,' ',word4];
            q=['Remote ID- ',word1,' | Latitude - ',word2,' | Longitude - ',word3,' | Altitude - ',word4];
            disp(q);

            encryptedData = aes_encrypt(s, key);
            % Open the file for writing
            fileID = fopen('data_encrypted.txt', 'w');  fprintf(fileID, '%s', encryptedData);   fclose(fileID);

            % Add the encrypted file to IPFS and capture the output
            [status, cmdout] = system('ipfs add data_encrypted.txt');

            if status == 0
                %disp('Encrypted file added to IPFS');
                %disp(cmdout);  % Display the output from the command
                % Extract the CID from the output
                parts = strsplit(cmdout);
                cid = parts{8};  % The CID is the second element
                disp("Present Remote Id Hash value of Blockchain - ");
                disp(['CID: ', cid]);  % Display the CID
                fid = fopen('blockchainkey.txt', 'w');
                fwrite(fid, cid);
                fclose(fid);
            else
                disp('Error adding file to IPFS');
            end

            %%%%%%%%%%%%  Files Comes To The reciever--------------------

            %----------Encryption and Decryption Key-------------
            data = fileread('blockchainkey.txt');
            %--------------------Retrieve the encrypted file using the CID
            [status, ss] = system(['ipfs cat ' data]);
            if status == 0
                %disp('Encrypted file retrieved from IPFS');
                %disp(ss);
                % Decrypt the retrieved file
                decryptedData = aes_decrypt(ss, key);  % Decrypt the data
                %disp('Decrypted content:');
                %disp(['Decrypted text: ', decryptedData]);  % Display the original content
            else
                disp('Error retrieving file from IPFS');
            end
            parts = strsplit(decryptedData, ' ');
            fprintf('Drone ID: %s authenticated successfully!\n', drone(i).ID);
            idd=parts{1};
            lat=parts{2};
            lon=parts{3};
            alt=parts{4};
            set(drone(i).remoteIDDisplay, 'String', sprintf('Remote ID: %s\nLatitude: %s\nLongitude: %s\nAltitude: %s m', ...
                idd, lat, lon, alt));
            
 

            %-----------------------------------------------------------------------------------------------
            else
                set(drone(i).remoteIDDisplay, 'String', sprintf('Remote ID: %s\nLatitude: null\nLongitude: null\nAltitude: null', drone(i).ID));
            end
        else
            set(drone(i).plot, 'MarkerFaceColor', 'r');  % Out of range
            set(drone(i).remoteIDDisplay, 'String', sprintf('Remote ID: %s out of range', drone(i).ID));
        end
        pause(smoothStep);  % Pause for smooth movement
    end

    
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
