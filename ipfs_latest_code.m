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
    drone(i).Session_ID = '';  % Session_ID-style identifier
end

% Ground Station (GCS) properties
groundStation.range = 1000;  % Range in meters
groundStation.latitude = 37.7749; % Ground station location
groundStation.longitude = -122.4194; % Ground station location

% Define No-Fly Zone (small circular area, randomly placed)
noFlyZone.radius = 200; % Radius in meters (smaller size)
noFlyZone.centerLat = 37.7749 + (rand() * 0.002 - 0.001); % Random initial latitude
noFlyZone.centerLon = -122.4194 + (rand() * 0.002 - 0.001); % Random initial longitude
noFlyZone.lastUpdate = 0; % Track time of last no-fly zone movement

% Open log file for no-fly zone violations and unauthorized entries
logFile = fopen('uav_violations_log.txt', 'w');
fprintf(logFile, 'UAV Violation Log\n');
fprintf(logFile, '================\n');

% --- Step 2: Define the Database ---
droneDatabase = struct('ID', {'ABC123', 'DEF456', 'GHI789', 'JKL012', 'MNO345'}, ...
                       'operator', {'Operator1', 'Operator2', 'Operator3', 'Operator4', 'Operator5'}, ...
                       'registrationStatus', {'Valid', 'Valid', 'Valid', 'InValid', 'Valid'}, ...
                       'allowedZones', {'Permitted', 'Permitted', 'Permitted', 'Permitted', 'No'}, ...
                       'Session_ID', {'', '', '', '', ''}); % Session_ID storage

% --- Step 3: Setup the 2D Visualization ---
f = figure('Name', 'UAV Simulation', 'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
hold on;
grid on;
axis equal;
set(gca, 'FontSize', 12, 'FontWeight', 'bold', 'GridLineStyle', '--', 'GridAlpha', 0.7);

% Plot the ground station
plot(groundStation.longitude, groundStation.latitude, 'rx', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Ground Station');

% Plot the ground station's communication range as a circle
theta = linspace(0, 2*pi, 100);
stationRangeLat = groundStation.latitude + (groundStation.range/111320) * cos(theta); % Convert meters to lat
stationRangeLon = groundStation.longitude + (groundStation.range/(111320*cosd(groundStation.latitude))) * sin(theta); % Convert meters to lon
plot(stationRangeLon, stationRangeLat, 'r--', 'LineWidth', 2, 'DisplayName', 'GCS Range');

% Plot the no-fly zone as a circle (initial position)
noFlyZone.plot = plot(NaN, NaN, 'k--', 'LineWidth', 2, 'DisplayName', 'No-Fly Zone');
noFlyZoneLat = noFlyZone.centerLat + (noFlyZone.radius/111320) * cos(theta); % Convert meters to lat
noFlyZoneLon = noFlyZone.centerLon + (noFlyZone.radius/(111320*cosd(noFlyZone.centerLat))) * sin(theta); % Convert meters to lon
set(noFlyZone.plot, 'XData', noFlyZoneLon, 'YData', noFlyZoneLat);

% Initialize the drone plots (one for each drone, no flight path)
for i = 1:5
    drone(i).plot = plot(drone(i).longitude, drone(i).latitude, 'bo', 'MarkerSize', 12, ...
                         'LineWidth', 2, 'MarkerFaceColor', 'b', 'DisplayName', sprintf('UID: %s (Out of Range)', drone(i).ID));
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
noFlyZoneUpdateInterval = 30; % Update no-fly zone every 30 seconds

% Initialize arrays for each drone's movement
for i = 1:5
    drone(i).time = 0:broadcastInterval:simulationTime;
    drone(i).latitudeLog = zeros(1, length(drone(i).time));
    drone(i).longitudeLog = zeros(1, length(drone(i).time));
    drone(i).fullSecondCounter = 0;  % Counter for full-second broadcast
end

randomDirection = @(maxVal) (rand()*2 - 1) * maxVal;  % Random direction function

% --- SHA-256 Hash Function ---
function hash = sha256(data)
    import java.security.MessageDigest
    md = MessageDigest.getInstance('SHA-256');
    md.update(uint8(data));
    hashBytes = md.digest();
    hash = lower(sprintf('%02x', typecast(hashBytes, 'uint8')));
end

%disp('Starting Multiple Drone Flight Simulation:');
for t = 0:smoothStep:simulationTime
    % --- Step 5: Update No-Fly Zone Position ---
    if t - noFlyZone.lastUpdate >= noFlyZoneUpdateInterval
        noFlyZone.centerLat = 37.7749 + (rand() * 0.002 - 0.001); % Random new latitude
        noFlyZone.centerLon = -122.4194 + (rand() * 0.002 - 0.001); % Random new longitude
        noFlyZone.lastUpdate = t;
        % Update no-fly zone plot
        noFlyZoneLat = noFlyZone.centerLat + (noFlyZone.radius/111320) * cos(theta);
        noFlyZoneLon = noFlyZone.centerLon + (noFlyZone.radius/(111320*cosd(noFlyZone.centerLat))) * sin(theta);
        set(noFlyZone.plot, 'XData', noFlyZoneLon, 'YData', noFlyZoneLat);
        fprintf('No-Fly Zone moved to Lat: %.6f, Lon: %.6f at time %.1f s\n', ...
                noFlyZone.centerLat, noFlyZone.centerLon, t);
    end

    % --- Step 6: Random and Smooth Drone Movement ---
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
        end

        % --- Step 7: Check No-Fly Zone ---
        distanceToNoFlyZone = haversine(drone(i).latitude, drone(i).longitude, ...
                                        noFlyZone.centerLat, noFlyZone.centerLon);
        if distanceToNoFlyZone <= noFlyZone.radius
            set(drone(i).plot, 'MarkerFaceColor', 'r', ...
                'DisplayName', sprintf('UID: %s (In No-Fly Zone)', drone(i).ID));  % In no-fly zone: Red
            fprintf('Drone ID: %s entered no-fly zone at time %.1f s!\n', drone(i).ID, t);
            timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
            fprintf(logFile, 'Timestamp: %s | Drone ID: %s | Violation: Entered No-Fly Zone | Lat: %.6f | Lon: %.6f\n', ...
                    timestamp, drone(i).ID, drone(i).latitude, drone(i).longitude);
        else
            % --- Step 8: Simulate Drone Sending ID to GCS ---
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
                                'DisplayName', sprintf('%s', drone(i).Session_ID));  % Authorized: Session_ID
                            fprintf('Drone ID: %s is authorized to fly!\n', drone(i).ID);
                            
                            % Assign Session_ID-style identifier if not already assigned
                            if isempty(drone(i).Session_ID)
                                drone(i).Session_ID = sprintf('UAV-%s%d', char(65+i-1), 123 + i*111);
                                droneDatabase(j).Session_ID = drone(i).Session_ID; % Update database
                                fprintf('Assigned Session_ID to Drone %s: %s\n', drone(i).ID, drone(i).Session_ID);
                            end
                        else
                            set(drone(i).plot, 'MarkerFaceColor', 'r', ...
                                'DisplayName', sprintf('UID: %s (Unauthorized)', drone(i).ID));  % Unauthorized: UID
                            fprintf('Drone ID: %s is unauthorized (invalid registration or zone)!\n', drone(i).ID);
                            timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
                            fprintf(logFile, 'Timestamp: %s | Drone ID: %s | Violation: Unauthorized Entry | Lat: %.6f | Lon: %.6f\n', ...
                                    timestamp, drone(i).ID, drone(i).latitude, drone(i).longitude);
                        end
                        
                        % Update Remote ID display immediately
                        if authenticated
                            word1 = drone(i).Session_ID; % Use Session_ID for identification
                            word2 = num2str(drone(i).latitude);
                            word3 = num2str(drone(i).longitude);
                            word4 = num2str(drone(i).altitude);
                            word5 = drone(i).ID; % Include original ID for reference
                            s = [word1, ' ', word2, ' ', word3, ' ', word4, ' ', word5];
                            q = ['Session_ID: ', word1, '|Latitude: ', word2, '|Longitude: ', word3, ...
                                 '|Altitude: ', word4, '|Drone ID: ', word5];
                            disp(q);

                            % Compute SHA-256 hash of the data
                            dataHash = sha256(s);
                            disp(['SHA-256 Hash: ', dataHash]);
                            
                            % Store hash in a file
                            hashFileID = fopen('data_hash.txt', 'w');
                            fprintf(hashFileID, '%s', dataHash);
                            fclose(hashFileID);

                            % Encrypt the data
                            encryptedData = aes_encrypt(s, key);
                            fileID = fopen('data_encrypted.txt', 'w');
                            fprintf(fileID, '%s', encryptedData);
                            fclose(fileID);

                            % Add encrypted file and hash to IPFS
                            [status, cmdout] = system('ipfs add data_encrypted.txt');
                            if status == 0
                                parts = strsplit(cmdout);
                                cid = parts{2}; % Adjust index based on IPFS output
                                disp('Present Remote ID Hash value of Blockchain - ');
                                disp(['CID (Encrypted Data): ', cid]);
                                fid = fopen('blockchainkey.txt', 'w');
                                fwrite(fid, cid);
                                fclose(fid);

                                % Add hash file to IPFS
                                [hashStatus, hashCmdout] = system('ipfs add data_hash.txt');
                                if hashStatus == 0
                                    hashParts = strsplit(hashCmdout);
                                    hashCid = hashParts{2};
                                    disp(['CID (SHA-256 Hash): ', hashCid]);
                                    fid = fopen('blockchainkey_hash.txt', 'w');
                                    fwrite(fid, hashCid);
                                    fclose(fid);
                                else
                                    disp('Error adding hash file to IPFS');
                                end
                            else
                                disp('Error adding encrypted file to IPFS');
                                if distanceToNoFlyZone <= noFlyZone.radius
                                    set(drone(i).remoteIDDisplay, 'BackgroundColor', [1 0.7 0.7], ...
                                        'String', sprintf('Session_ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nID: %s\nRemark: Entering No-Fly Zone', ...
                                        drone(i).Session_ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude), drone(i).ID));
                                else
                                    set(drone(i).remoteIDDisplay, 'BackgroundColor', 'white', ...
                                        'String', sprintf('Session_ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nID: %s', ...
                                        drone(i).Session_ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude), drone(i).ID));
                                end
                                continue; % Skip to next drone if IPFS fails
                            end

                            % Simulate GCS retrieving and decrypting data
                            data = fileread('blockchainkey.txt');
                            [status, ss] = system(['ipfs cat ', data]);
                            if status == 0
                                decryptedData = aes_decrypt(ss, key);
                                % Verify hash
                                computedHash = sha256(decryptedData);
                                storedHash = fileread('data_hash.txt');
                                hashVerified = strcmp(computedHash, storedHash);
                                if hashVerified
                                    fprintf('SHA-256 Hash Verification: Success\n');
                                else
                                    fprintf('SHA-256 Hash Verification: Failed\n');
                                end
                                parts = strsplit(decryptedData, ' ');
                                fprintf('DroneWITH Session_ID: %s (ID: %s) authenticated successfully!\n', parts{1}, parts{5});
                                session_id = parts{1};
                                lat = parts{2};
                                lon = parts{3};
                                alt = parts{4};
                                idd = parts{5};
                                if distanceToNoFlyZone <= noFlyZone.radius
                                    set(drone(i).remoteIDDisplay, 'BackgroundColor', [1 0.7 0.7], ...
                                        'String', sprintf('Session_ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nID: %s\nRemark: Entering No-Fly Zone', ...
                                        session_id, lat, lon, alt, idd));
                                else
                                    set(drone(i).remoteIDDisplay, 'BackgroundColor', 'white', ...
                                        'String', sprintf('Session_ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nID: %s', ...
                                        session_id, lat, lon, alt, idd));
                                end
                            else
                                disp('Error retrieving file from IPFS');
                                if distanceToNoFlyZone <= noFlyZone.radius
                                    set(drone(i).remoteIDDisplay, 'BackgroundColor', [1 0.7 0.7], ...
                                        'String', sprintf('Session_ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nID: %s\nRemark: Entering No-Fly Zone', ...
                                        drone(i).Session_ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude), drone(i).ID));
                                else
                                    set(drone(i).remoteIDDisplay, 'BackgroundColor', 'white', ...
                                        'String', sprintf('Session_ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nID: %s', ...
                                        drone(i).Session_ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude), drone(i).ID));
                                end
                            end
                        else
                            if distanceToNoFlyZone <= noFlyZone.radius
                                set(drone(i).remoteIDDisplay, 'BackgroundColor', [1 0.7 0.7], ...
                                    'String', sprintf('ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nSession_ID: Unauthorized\nRemark: Entering No-Fly Zone', ...
                                    drone(i).ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude)));
                            else
                                set(drone(i).remoteIDDisplay, 'BackgroundColor', 'white', ...
                                    'String', sprintf('ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nSession_ID: Unauthorized', ...
                                    drone(i).ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude)));
                            end
                        end
                        break;
                    end
                end
            else
                set(drone(i).plot, 'MarkerFaceColor', 'b', ...
                    'DisplayName', sprintf('UID: %s (Out of Range)', drone(i).ID));  % Out of range: Blue
                if distanceToNoFlyZone <= noFlyZone.radius
                    set(drone(i).remoteIDDisplay, 'BackgroundColor', [1 0.7 0.7], ...
                        'String', sprintf('ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nSession_ID: Out of range\nRemark: Entering No-Fly Zone', ...
                        drone(i).ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude)));
                else
                    set(drone(i).remoteIDDisplay, 'BackgroundColor', 'white', ...
                        'String', sprintf('ID: %s\nLat: %s\nLon: %s\nAlt: %s m\nSession_ID: Out of range', ...
                        drone(i).ID, num2str(drone(i).latitude), num2str(drone(i).longitude), num2str(drone(i).altitude)));
                end
            end
        end
    end
    pause(smoothStep);  % Pause for smooth movement
end

% Close the log file
fclose(logFile);

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