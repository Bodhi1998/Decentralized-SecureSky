% Decentralized SecureSky: Multi-UAV Simulation with Multiple Attack Failures
% Based on analysis of IPFS Whole.pdf
% ATTACKS SIMULATED (and failing):
% - Drone 4: ID Spoofing (-> Zone Check Fail) -> Replay Attack (-> Session ID Mismatch Fail)
% - Drone 5: No-Fly Zone Evasion (-> Visual/Log Discrepancy)
% - Drone 3: MitM Data Corruption (-> Decryption/Hash Fail)
% VERSION: Updated attack output text, removed timestamps from attack messages

%% Step 1: Define Drones and Ground Station
clc;
clear; % Clear workspace
close all; % Close all figures

fprintf('Initializing Simulation (Multi-Attack Failure Demo)...\n');

% --- ASSUMPTION: Encryption/Decryption key file exists ---
try
    key = fileread('endeckey.txt');
    fprintf('Encryption key loaded successfully.\n');
catch ME
    error('Failed to load encryption key from endeckey.txt. Please ensure the file exists. Error: %s', ME.message);
end

% Define properties for 5 drones
drone(1).ID = 'ABC123'; % Authorized Drone 1 (Target for spoofing)
drone(2).ID = 'DEF456'; % Authorized Drone 2 (Target for replay capture)
drone(3).ID = 'GHI789'; % Authorized Drone 3 (Target for MitM)
drone(4).ID = 'JKL012'; % Unauthorized Drone 4 (Attacker: Spoofing -> Replay)
drone(5).ID = 'MNO345'; % Unauthorized Drone 5 (Attacker: NFZ Evasion)

% Initial positions
initial_latitudes  = [37.7749, 37.7740, 37.7750, 37.7730, 37.7729];
initial_longitudes = [-122.4194, -122.4185, -122.4200, -122.4170, -122.4165];
initial_altitudes = [100, 120, 110, 105, 130]; % Meters

% Store last known good transmission data for replay attack simulation
global lastGoodTransmission; % Use global or pass struct around
lastGoodTransmission.encryptedData = '';
lastGoodTransmission.dataHash = '';
lastGoodTransmission.sessionID = '';
lastGoodTransmission.originalDroneID = '';

for i=1:5
    drone(i).latitude = initial_latitudes(i);
    drone(i).longitude = initial_longitudes(i);
    drone(i).altitude = initial_altitudes(i);
    drone(i).speed = 0.0001;
    % Assign zones for attack setup
    if i == 4 % Attacker Drone 4 needs a different zone than target Drone 1
       drone(i).flightZone = 'Restricted';
    else
       drone(i).flightZone = 'Permitted'; % Default zone for others
    end
    drone(i).operatorID = ['Operator', num2str(i)];
    drone(i).Session_ID = '';
    drone(i).status = 'Initializing';
    drone(i).lastMessage = '';
    drone(i).attackMode = 'None'; % Track attack attempts
    drone(i).actualLatitude = drone(i).latitude; % For NFZ evasion tracking
    drone(i).actualLongitude = drone(i).longitude; % For NFZ evasion tracking
end
fprintf('INFO: Drone 4 (Attacker) assigned to "Restricted" zone.\n');
fprintf('INFO: Drone 5 (Attacker) will attempt NFZ Evasion.\n');
fprintf('INFO: Drone 3 transmission will be subjected to simulated MitM corruption.\n');

% Ground Station (GCS) properties
groundStation.range = 1000; % meters
groundStation.latitude = 37.7749;
groundStation.longitude = -122.4194;

% Define No-Fly Zone
noFlyZone.radius = 200; % meters
noFlyZone.centerLat = 37.7749 + (rand()*0.002 - 0.001);
noFlyZone.centerLon = -122.4194 + (rand()*0.002 - 0.001);
noFlyZone.lastUpdate = 0;

% Open log file
logFileName = 'uav_multi_attack_failure_log.txt';
logFile = fopen(logFileName, 'w');
if logFile == -1, error('Could not open log file: %s', logFileName); end
fprintf(logFile, 'UAV Multi-Attack Failure Demo Log - %s\n', datestr(now));
fprintf(logFile, '====================================\n');
fprintf(logFile, 'Attacks:\n D4: Spoof D1 -> Replay D2\n D5: NFZ Evasion\n D3: MitM\n');
fprintf(logFile, '====================================\n');

%% Step 2: Define the Database
droneDatabase = struct(...
    'ID', {'ABC123', 'DEF456', 'GHI789', 'JKL012', 'MNO345'}, ...
    'operator', {'Operator1', 'Operator2', 'Operator3', 'Operator4', 'Operator5'}, ...
    'registrationStatus', {'Valid', 'Valid', 'Valid', 'Invalid', 'Valid'}, ... % D4 Invalid, D5 Valid (but No permission below)
    'allowedZones', {'Permitted', 'Permitted', 'Permitted', 'Permitted', 'No'}, ... % D1 needs Permitted, D5 has No permission
    'Session_ID', {'', '', '', '', ''});
fprintf('Database initialized.\n');

%% Step 3: Setup the 2D Visualization
f = figure('Name', 'Decentralized SecureSky: Multi-Attack Failure Demo', 'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
hold on; grid on; axis equal;
set(gca, 'FontSize', 10, 'FontWeight', 'bold');

% Plot GCS, Range, NFZ (using Haversine-based calculations for accuracy)
plot(groundStation.longitude, groundStation.latitude, 'k^', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'y', 'DisplayName', 'Ground Station');
theta = linspace(0, 2*pi, 100);
[gcsRangeLat, gcsRangeLon] = calculateCircle(groundStation.latitude, groundStation.longitude, groundStation.range, theta);
plot(gcsRangeLon, gcsRangeLat, 'r--', 'LineWidth', 1.5, 'DisplayName', 'GCS Range');
[nfzLat, nfzLon] = calculateCircle(noFlyZone.centerLat, noFlyZone.centerLon, noFlyZone.radius, theta);
noFlyZone.plot = plot(nfzLon, nfzLat, 'k-', 'LineWidth', 2, 'DisplayName', 'No-Fly Zone');

% Initialize Drone Plots
colors = {'g', 'c', 'm', 'b', 'b'}; % D4, D5 start blue (unauthorized)
markers = {'o', 'o', 'o', 's', 'd'}; % Square for D4, Diamond for D5
for i=1:5
    drone(i).plot = plot(drone(i).longitude, drone(i).latitude, markers{i}, 'MarkerSize', 8, ...
        'LineWidth', 1.5, 'MarkerFaceColor', colors{i}, 'MarkerEdgeColor', 'k', ...
        'DisplayName', sprintf('UID: %s (%s)', drone(i).ID, drone(i).status));
    % Add plot for actual position for NFZ evader
    if i == 5
         drone(i).actualPlot = plot(drone(i).actualLongitude, drone(i).actualLatitude, 'rx', 'MarkerSize', 10, ...
         'LineWidth', 1.5, 'DisplayName', sprintf('%s Actual Pos', drone(i).ID));
         drone(i).actualPlot.Visible = 'off'; % Initially hidden
    end
end

% Set plot limits
latLim = [min(initial_latitudes)-0.005, max(initial_latitudes)+0.005];
lonLim = [min(initial_longitudes)-0.005, max(initial_longitudes)+0.005];
axis([lonLim latLim]);

xlabel('Longitude'); ylabel('Latitude');
title('Decentralized SecureSky: Multi-Attack Failure Demo', 'FontSize', 14, 'FontWeight', 'bold');
legend('show', 'Location', 'northeastoutside', 'FontSize', 9);

% UI elements for Remote ID Display
uiTextHeight = 0.09; uiWidth = 0.18; uiXPos = 0.81; uiYStart = 0.9;
for i=1:5
    drone(i).remoteIDDisplay = uicontrol('Style', 'text', 'Units', 'normalized', ...
        'Position', [uiXPos, uiYStart - i*uiTextHeight, uiWidth, uiTextHeight], ...
        'BackgroundColor', 'white', 'FontSize', 8, 'HorizontalAlignment', 'left', ...
        'String', sprintf('Drone %d (%s): Init...', i, drone(i).ID));
end

%% Step 4: Simulation Parameters
broadcastInterval = 1; smoothStep = 0.1; simulationTime = 180; % Longer sim time
noFlyZoneUpdateInterval = 45;
randomDirection = @(maxVal) (rand()*2 - 1) * maxVal;
disp('Starting Simulation...');

%% Main Simulation Loop
for t = 0:smoothStep:simulationTime
    currentTimeStr = sprintf('Time: %.1f s', t); % Keep general time display

    %% Step 5: Update No-Fly Zone Position
    if t - noFlyZone.lastUpdate >= noFlyZoneUpdateInterval && t > 0
        noFlyZone.centerLat = 37.7749 + (rand()*0.004 - 0.002);
        noFlyZone.centerLon = -122.4194 + (rand()*0.004 - 0.002);
        noFlyZone.lastUpdate = t;
        [nfzLat, nfzLon] = calculateCircle(noFlyZone.centerLat, noFlyZone.centerLon, noFlyZone.radius, theta);
        set(noFlyZone.plot, 'XData', nfzLon, 'YData', nfzLat);
        fprintf('NFZ moved @ %.1f s\n', t); % Keep timestamp here
    end

    %% Steps 6, 7, 8: Drone Actions
    for i = 1:5 % Loop through each drone

        % --- Step 6: Drone Movement ---
        % Store previous actual position
        prevActualLat = drone(i).actualLatitude;
        prevActualLon = drone(i).actualLongitude;

        % Calculate new actual position
        drone(i).actualLatitude = drone(i).actualLatitude + randomDirection(drone(i).speed * smoothStep * 50);
        drone(i).actualLongitude = drone(i).actualLongitude + randomDirection(drone(i).speed * smoothStep * 50);
        % Basic boundary check on actual position
        drone(i).actualLatitude = min(max(drone(i).actualLatitude, latLim(1)+0.0005), latLim(2)-0.0005);
        drone(i).actualLongitude = min(max(drone(i).actualLongitude, lonLim(1)+0.0005), lonLim(2)-0.0005);

        % --- NFZ Evasion Attack Logic (Drone 5) ---
        reportedLatitude = drone(i).actualLatitude; % Default: report actual position
        reportedLongitude = drone(i).actualLongitude;
        isEvadingNFZ = false;

        if i == 5 % Drone 5 attempts evasion
            drone(i).attackMode = 'NFZ Evasion';
            % Check if ACTUAL position is inside NFZ
            actualDistanceToNFZ = haversine(drone(i).actualLatitude, drone(i).actualLongitude, noFlyZone.centerLat, noFlyZone.centerLon);
            if actualDistanceToNFZ <= noFlyZone.radius
                isEvadingNFZ = true;
                drone(i).status = 'NFZ Evasion Attempt';
                drone(i).actualPlot.Visible = 'on'; % Show actual position marker

                % Calculate FAKE coordinates just outside the NFZ boundary
                angleToCenter = atan2(drone(i).actualLatitude - noFlyZone.centerLat, drone(i).actualLongitude - noFlyZone.centerLon);
                fakeDist = noFlyZone.radius * 1.1; % Report 10% outside radius
                [reportedLatitude, reportedLongitude] = destinationPoint(noFlyZone.centerLat, noFlyZone.centerLon, fakeDist, rad2deg(angleToCenter));

                % Updated Output: No timestamp
                fprintf('*** ATTACK SIM (D5): NFZ Evasion! Actual(%.4f, %.4f) is IN NFZ. Reporting Fake(%.4f, %.4f) ***\n', ...
                        drone(i).actualLatitude, drone(i).actualLongitude, reportedLatitude, reportedLongitude);
            else
                 drone(i).actualPlot.Visible = 'off'; % Hide actual position marker if outside
            end
        end
        % --- End NFZ Evasion Logic ---

        % Update drone's REPORTED position for checks and plotting main marker
        drone(i).latitude = reportedLatitude;
        drone(i).longitude = reportedLongitude;
        set(drone(i).plot, 'XData', drone(i).longitude, 'YData', drone(i).latitude);
        if i == 5 % Also update actual plot position
             set(drone(i).actualPlot, 'XData', drone(i).actualLongitude, 'YData', drone(i).actualLatitude);
        end


        % --- Step 7: Check No-Fly Zone (using REPORTED coordinates) ---
        distanceToNoFlyZone = haversine(drone(i).latitude, drone(i).longitude, noFlyZone.centerLat, noFlyZone.centerLon);
        inNoFlyZone = (distanceToNoFlyZone <= noFlyZone.radius);
        remark = '';
        bgColor = 'white';

        if inNoFlyZone
            if i == 5 && isEvadingNFZ
                 % This case should not happen if evasion logic is correct, but good failsafe
                 remark = 'NFZ Evasion FAILED!';
                 bgColor = [1 0 1]; % Magenta for failed evasion
                 % Updated Output: No timestamp
                 fprintf('*** ATTACK FAIL (D5): NFZ Evasion Failed! Reported position still in NFZ. ***\n');
            else
                remark = 'ALERT: In No-Fly Zone';
                bgColor = [1 0.7 0.7]; % Reddish background
                if ~contains(drone(i).status, 'In NFZ') && ~isEvadingNFZ
                     % Keep timestamp for general NFZ entry log
                     fprintf('ALERT: Drone ID: %s entered no-fly zone @ %.1f s!\n', drone(i).ID, t);
                     fprintf(logFile, '%.1f s | Drone ID: %s | VIOLATION: Entered No-Fly Zone | Lat: %.6f, Lon: %.6f\n', ...
                             t, drone(i).ID, drone(i).latitude, drone(i).longitude); % Log reported position
                     drone(i).status = 'In NFZ';
                     set(drone(i).plot, 'MarkerEdgeColor', 'r', 'LineWidth', 2.5);
                end
            end
        else % Reported position is outside NFZ
             if i == 5 && isEvadingNFZ
                 remark = 'NFZ Evasion Active'; % Indicate evasion is ongoing based on reported pos
                 bgColor = [1 1 0.7]; % Yellowish for active evasion
                 set(drone(i).plot, 'MarkerEdgeColor', 'm', 'LineWidth', 2.5); % Magenta edge for evader marker
             elseif contains(drone(i).status, 'In NFZ') || contains(drone(i).status, 'Evasion') % Reset status if left NFZ or stopped evading
                drone(i).status = 'Nominal'; % Reset status (will be updated by auth)
                 set(drone(i).plot, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5); % Reset marker edge
             end
        end


        % --- Step 8: Simulate Drone Sending ID to GCS & Authentication ---
        if mod(floor(t), broadcastInterval) == 0 || contains(drone(i).status, 'Connecting')

            distanceToGroundStation = haversine(drone(i).latitude, drone(i).longitude, groundStation.latitude, groundStation.longitude);

            if distanceToGroundStation <= groundStation.range
                drone(i).status = 'Connecting';
                authenticated = false; % Reset auth status
                authReason = ''; % Reason for auth outcome

                % --- Attack Attempts ---
                isAttacker = (i == 4 || i == 5); % Drones 4 and 5 are attackers
                idToPresent = drone(i).ID; % Default
                attemptingReplay = false;

                if i == 4 % Drone 4 tries Spoofing first
                    drone(i).attackMode = 'ID Spoofing';
                    targetSpoofedID = drone(1).ID; % Target D1
                    idToPresent = targetSpoofedID;
                    % Updated Output: No timestamp
                    fprintf('*** ATTACK SIM (D4): Trying ID Spoof as [%s] ***\n', idToPresent);
                end

                % --- Authentication Check ---
                dbIndex = find(strcmp(idToPresent, {droneDatabase.ID}), 1);

                if ~isempty(dbIndex)
                    % ID Found - Perform Secondary Checks
                    regValid = strcmp(droneDatabase(dbIndex).registrationStatus, 'Valid');
                    % Use ACTUAL drone's zone vs PRESENTED ID's allowed zone
                    zoneMatch = strcmp(drone(i).flightZone, droneDatabase(dbIndex).allowedZones);

                    if regValid && zoneMatch
                        authenticated = true;
                        authReason = 'Authorized';
                        drone(i).status = 'Authorized';
                        set(drone(i).plot, 'MarkerFaceColor', 'g');
                        if i == 4 % Should not happen for Drone 4 due to zone mismatch
                            authReason = 'Spoof Success (UNEXPECTED)';
                            drone(i).status = 'Auth (Spoofed - Error)';
                            set(drone(i).plot, 'MarkerFaceColor', [1 0.5 0]);
                            % Updated Output: No timestamp
                            fprintf('*** ATTACK ERROR (D4): Spoof as [%s] UNEXPECTEDLY passed auth! Check logic. ***\n', idToPresent);
                        end
                    else
                        % Auth Failed (Reg/Zone Check)
                        authenticated = false;
                        if ~regValid, authReason = 'Invalid Registration';
                        elseif ~zoneMatch, authReason = sprintf('Zone Mismatch (Drone in %s, ID %s needs %s)', drone(i).flightZone, idToPresent, droneDatabase(dbIndex).allowedZones);
                        else authReason = 'Reg/Zone Check Fail'; end
                        drone(i).status = ['Unauthorized (' authReason ')'];
                        set(drone(i).plot, 'MarkerFaceColor', 'r');
                        if i == 4 % Log Spoof Failure
                            % Updated Output: No timestamp
                            fprintf('*** ATTACK FAIL (D4): Spoof as [%s] FAILED Check (%s)! ***\n', idToPresent, authReason);
                            fprintf(logFile, '%.1f s | D4 Spoof Fail | Reason: %s\n', t, authReason); % Keep time in log file
                            % --- Initiate Replay Attack Attempt on next cycle ---
                            if ~isempty(lastGoodTransmission.encryptedData)
                                drone(i).attackMode = 'Replay Pending';
                                % Updated Output: No timestamp
                                fprintf('*** ATTACK SIM (D4): Spoof failed. Will attempt Replay Attack next. ***\n');
                            else
                                drone(i).attackMode = 'None'; % Cannot replay yet
                            end
                        end
                    end
                else
                    % ID Not Found in DB
                    authenticated = false;
                    authReason = 'ID Not Found';
                    drone(i).status = 'Unauthorized (ID Unknown)';
                    set(drone(i).plot, 'MarkerFaceColor', 'r');
                     if i == 4 % Log Spoof Failure (ID not found)
                         % Updated Output: No timestamp
                         fprintf('*** ATTACK FAIL (D4): Spoofed ID [%s] NOT FOUND in DB! ***\n', idToPresent);
                         fprintf(logFile, '%.1f s | D4 Spoof Fail | Reason: Spoofed ID Not Found\n', t); % Keep time in log file
                         drone(i).attackMode = 'None'; % Cannot replay if spoof ID invalid
                     end
                end

                 % --- Replay Attack Attempt (Drone 4, if spoof failed and data captured) ---
                 if i == 4 && strcmp(drone(i).attackMode, 'Replay Pending') && ~isempty(lastGoodTransmission.encryptedData)
                     authenticated = false; % Ensure not authenticated yet
                     drone(i).attackMode = 'Replay Attack';
                     attemptingReplay = true;
                     % Updated Output: No timestamp
                     fprintf('*** ATTACK SIM (D4): Attempting Replay Attack using D%s data ***\n', lastGoodTransmission.originalDroneID);

                     % Simulate GCS receiving replayed data
                     replayedEncryptedData = lastGoodTransmission.encryptedData;
                     replayedHash = lastGoodTransmission.dataHash;
                     replayedSessionID = lastGoodTransmission.sessionID;

                     % GCS Verification Step 1: Decrypt and Verify Hash (should pass if data is intact)
                     replayVerified = false;
                     try
                         decryptedReplay = aes_decrypt(replayedEncryptedData, key);
                         computedReplayHash = sha256(decryptedReplay);
                         if strcmp(computedReplayHash, replayedHash)
                             replayVerified = true;
                             % Updated Output: No timestamp
                             fprintf('--- Replay Attack (D4): Data & Hash Verification PASSED (as expected). ---\n');
                         else
                             % Updated Output: No timestamp
                             fprintf('--- Replay Attack (D4): Data & Hash Verification FAILED (Unexpected)! ---\n');
                         end
                     catch ME_replay_verify
                          fprintf('--- Replay Attack (D4): Decryption/Hash Error during verify: %s ---\n', ME_replay_verify.message);
                     end

                     % GCS Verification Step 2: Check Session ID against *requesting* drone (THE KEY ANTI-REPLAY STEP)
                     if replayVerified
                         % Extract Session ID from decrypted data (assuming format: session,lat,lon,alt,id)
                         parts = strsplit(decryptedReplay, ',');
                         if length(parts) >= 1
                             sessionInReplay = parts{1};
                             % Check if the session ID belongs to the drone that SENT the replay (Drone 4)
                             % It SHOULD belong to the original drone (Drone 2)
                             if strcmp(sessionInReplay, drone(i).Session_ID) % Compare against D4's (empty) Session ID
                                 % This is wrong, replay should not grant session ID
                                 authenticated = false; % Still fail
                                 authReason = 'Replay Detected (Session ID Mismatch)';
                                 drone(i).status = 'Unauthorized (Replay Fail)';
                                 set(drone(i).plot, 'MarkerFaceColor', [0.5 0 0]); % Darkest Red
                                 % Updated Output: No timestamp
                                 fprintf('*** ATTACK FAIL (D4): Replay Attack FAILED! Session ID [%s] in data does not belong to Drone 4. ***\n', sessionInReplay);
                                 fprintf(logFile, '%.1f s | D4 Replay Fail | Reason: Session ID Mismatch\n', t); % Keep time in log file
                             else
                                 authenticated = false; % Fail: Session ID doesn't match sender
                                 authReason = 'Replay Detected (Session ID Mismatch)';
                                 drone(i).status = 'Unauthorized (Replay Fail)';
                                 set(drone(i).plot, 'MarkerFaceColor', [0.5 0 0]); % Darkest Red
                                 % Updated Output: No timestamp
                                 fprintf('*** ATTACK FAIL (D4): Replay Attack FAILED! Session ID [%s] in data does not belong to Drone 4. ***\n', sessionInReplay);
                                 fprintf(logFile, '%.1f s | D4 Replay Fail | Reason: Session ID Mismatch\n', t); % Keep time in log file
                             end
                         else
                            authenticated = false; authReason = 'Replay Data Invalid';
                            % Updated Output: No timestamp
                            fprintf('*** ATTACK FAIL (D4): Replay Attack FAILED! Could not parse Session ID from replayed data. ***\n');
                         end
                     else
                         authenticated = false; authReason = 'Replay Hash/Decrypt Fail';
                         % Updated Output: No timestamp
                         fprintf('*** ATTACK FAIL (D4): Replay Attack FAILED! Hash/Decryption failed during verification. ***\n');
                     end
                     drone(i).attackMode = 'None'; % Reset attack mode after attempt
                 end


                % --- Subsequent Processing (Hashing, Encryption, IPFS) ---
                if authenticated % Only for genuinely authorized drones (or unexpected spoof success)

                    % Assign Session_ID if needed
                    if isempty(drone(i).Session_ID)
                        drone(i).Session_ID = sprintf('UAV-%s%d', char(64+i), randi([100,999]));
                        fprintf('Assigned Session ID to Drone %s: %s (Index %d)\n', drone(i).ID, drone(i).Session_ID, i);
                    end

                    % Prepare data string (using REPORTED coordinates)
                    word1 = drone(i).Session_ID;
                    word2 = num2str(drone(i).latitude, '%.6f');
                    word3 = num2str(drone(i).longitude, '%.6f');
                    word4 = num2str(drone(i).altitude, '%.1f');
                    word5 = drone(i).ID; % Original ID
                    dataString = strjoin({word1, word2, word3, word4, word5}, ',');

                    % Compute SHA-256 hash
                    try dataHash = sha256(dataString); catch, dataHash = 'HASH_ERROR'; end

                    % Encrypt the data
                    try encryptedData = aes_encrypt(dataString, key); catch, encryptedData = 'ENCRYPT_ERROR'; end

                    % --- MitM Attack Simulation (Drone 3) ---
                    isMitMVictim = (i == 3);
                    if isMitMVictim && ~strcmp(encryptedData, 'ENCRYPT_ERROR')
                        % Updated Output: No timestamp
                        fprintf('*** ATTACK SIM (D3): Simulating MitM data corruption ***\n');
                        % Simple corruption: flip a character (if possible)
                        if length(encryptedData) > 5
                           corruptIdx = randi(length(encryptedData));
                           originalChar = encryptedData(corruptIdx);
                           newChar = char(originalChar + 1); % Simple modification
                           encryptedData(corruptIdx) = newChar;
                           fprintf('--- MitM (D3): Corrupted char at index %d from [%s] to [%s] ---\n', corruptIdx, originalChar, newChar);
                        else
                            encryptedData = [encryptedData, 'CORRUPTED']; % Append if too short
                             fprintf('--- MitM (D3): Appended CORRUPTED tag ---\n');
                        end
                    end
                    % --- End MitM ---

                    % Store last good transmission from Drone 2 for replay capture
                    if i == 2 && ~strcmp(encryptedData, 'ENCRYPT_ERROR') && ~strcmp(dataHash, 'HASH_ERROR')
                        lastGoodTransmission.encryptedData = encryptedData;
                        lastGoodTransmission.dataHash = dataHash;
                        lastGoodTransmission.sessionID = drone(i).Session_ID;
                        lastGoodTransmission.originalDroneID = num2str(i); % Store original drone index
                        % Updated Output: No timestamp
                        fprintf('--- Replay Capture: Stored D%s transmission data ---\n', lastGoodTransmission.originalDroneID);
                    end


                    % Add to IPFS (Optional - Simplified)
                    % ... (IPFS code omitted for brevity, assume it stores/retrieves) ...
                    cidEncrypted = 'IPFS_SIM_ENC_CID'; cidHash = 'IPFS_SIM_HASH_CID';


                    % Simulate GCS retrieving and verifying data
                    gcsRetrievedEncrypted = encryptedData; % Use potentially corrupted data
                    gcsRetrievedHash = dataHash; % Use original hash for comparison

                    verified = false;
                    verificationReason = '';
                    try
                        if strcmp(gcsRetrievedEncrypted, 'ENCRYPT_ERROR')
                           verificationReason = 'Encryption Error';
                        else
                           decryptedData = aes_decrypt(gcsRetrievedEncrypted, key);
                           if contains(decryptedData, 'DECRYPT_ERROR')
                               verificationReason = 'Decryption Failed';
                               if isMitMVictim, verificationReason = 'Decryption Failed (MitM?)'; end
                           else
                               computedHash = sha256(decryptedData);
                               if strcmp(computedHash, gcsRetrievedHash)
                                   verified = true; verificationReason = 'Success';
                               else
                                   verificationReason = 'Hash Mismatch';
                                   if isMitMVictim, verificationReason = 'Hash Mismatch (MitM?)'; end
                                   % --- Database Tamper Conceptual Fail Point ---
                                   % Updated Output: No timestamp
                                   fprintf('*** DATA TAMPER / MitM FAIL: Hash verification failed! Computed [%.10s...] != Stored [%.10s...] ***\n', ...
                                           computedHash, gcsRetrievedHash);
                                   fprintf(logFile, '%.1f s | D%d GCS Verify Fail | Reason: %s\n', t, i, verificationReason); % Keep time in log file
                               end
                           end
                        end
                    catch ME_verify
                        verificationReason = sprintf('Verification Error: %s', ME_verify.message);
                    end

                    if verified
                         % Updated Output: No timestamp
                         fprintf('--- GCS Verify (D%d): SUCCESS ---\n', i);
                    else
                         % Updated Output: No timestamp
                         fprintf('--- GCS Verify (D%d): FAILED (%s) ---\n', i, verificationReason);
                         if isMitMVictim
                              % Updated Output: No timestamp
                              fprintf('*** ATTACK FAIL (D3): MitM attack caused verification failure (%s)! ***\n', verificationReason);
                              fprintf(logFile, '%.1f s | D3 MitM Fail | Reason: %s\n', t, verificationReason); % Keep time in log file
                         end
                    end
                    drone(i).lastVerificationStatus = verified;

                end % End if authenticated

            else % Drone is out of GCS range
                if ~strcmp(drone(i).status, 'Out of Range')
                    drone(i).status = 'Out of Range';
                    drone(i).Session_ID = ''; % Clear Session ID
                    set(drone(i).plot, 'MarkerFaceColor', 'b');
                    set(drone(i).plot, 'DisplayName', sprintf('UID: %s (Out of Range)', drone(i).ID));
                    drone(i).attackMode = 'None'; % Reset attack mode
                    if i == 5, drone(i).actualPlot.Visible = 'off'; end % Hide actual marker
                end
            end % End GCS range check

        end % End periodic check


        % --- Update Remote ID Display ---
        statusStr = drone(i).status;
        idStr = drone(i).ID;
        sessionStr = drone(i).Session_ID;
        if isempty(sessionStr), sessionStr = 'N/A'; end
        attackStr = drone(i).attackMode;
        if strcmp(attackStr,'None'), attackStr = ''; else attackStr = ['(',attackStr,')']; end
        if i == 4 || i == 5, idStr = sprintf('%s (Attacker)', drone(i).ID); end % Mark attackers

        % Keep timestamp in UI display for overall context
        displayString = sprintf('D%d %s %s\nSt: %s\nSess: %s\nLat: %.4f\nLon: %.4f\nAlt: %.0f m\n%s\n%s', ...
                                i, idStr, attackStr, statusStr, sessionStr, ...
                                drone(i).latitude, drone(i).longitude, drone(i).altitude, ...
                                remark, currentTimeStr);

        if ~strcmp(displayString, drone(i).lastMessage)
            set(drone(i).remoteIDDisplay, 'String', displayString, 'BackgroundColor', bgColor);
            drone(i).lastMessage = displayString;
        else
             set(drone(i).remoteIDDisplay, 'BackgroundColor', bgColor);
        end

    end % End of loop for i = 1:5 (Drones)

    drawnow limitrate;
    pause(smoothStep);

    if ~isvalid(f), disp('Figure closed, stopping simulation.'); break; end

end % End of simulation time loop (t)

%% Post-Simulation
fprintf('Simulation finished.\n');
fclose(logFile);
fprintf('Log file saved: %s\n', logFileName);
delete('temp_*.txt');

%% Helper Functions (Unchanged)

function distance = haversine(lat1, lon1, lat2, lon2)
    R = 6371000; phi1 = deg2rad(lat1); phi2 = deg2rad(lat2);
    deltaPhi = deg2rad(lat2 - lat1); deltaLambda = deg2rad(lon2 - lon1);
    a = sin(deltaPhi/2)^2 + cos(phi1) * cos(phi2) * sin(deltaLambda/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a)); distance = R * c;
end

function hash = sha256(data)
    if ischar(data), dataBytes = uint8(data);
    elseif isnumeric(data), dataBytes = typecast(data(:)', 'uint8');
    else, error('SHA256 input must be char array or numeric.'); end
    try
        md = java.security.MessageDigest.getInstance('SHA-256');
        md.update(dataBytes); hashBytes = typecast(md.digest(), 'uint8');
        hash = lower(sprintf('%02x', hashBytes));
    catch ME_sha, fprintf('SHA256 Err: %s\n', ME_sha.message); hash = 'HASHING_ERROR';
    end
end

function encryptedData = aes_encrypt(data, key)
    % Placeholder - Replace with actual AES
    encryptedData = ['encrypted(', data, ')_key(', key(1:min(4,end)), ')'];
end

function decryptedData = aes_decrypt(encryptedData, key)
     % Placeholder - Replace with actual AES
     if contains(encryptedData, 'encrypted(')
         decryptedData = extractBetween(encryptedData, 'encrypted(', ')_key');
         if iscell(decryptedData) && ~isempty(decryptedData), decryptedData = decryptedData{1};
         else, decryptedData = 'DECRYPT_ERROR_PLACEHOLDER'; end
     else, decryptedData = 'DECRYPT_ERROR_PLACEHOLDER'; end
end

function [lat2, lon2] = destinationPoint(lat1, lon1, distance, bearing)
    % Calculates destination lat/lon given start, distance (m), bearing (deg)
    R = 6371000; % Earth radius in meters
    lat1 = deg2rad(lat1); lon1 = deg2rad(lon1); bearing = deg2rad(bearing);
    lat2 = asin(sin(lat1)*cos(distance/R) + cos(lat1)*sin(distance/R)*cos(bearing));
    lon2 = lon1 + atan2(sin(bearing)*sin(distance/R)*cos(lat1), cos(distance/R)-sin(lat1)*sin(lat2));
    lat2 = rad2deg(lat2); lon2 = rad2deg(lon2);
end

function [circleLat, circleLon] = calculateCircle(centerLat, centerLon, radius, theta)
    % Calculate points on a circle using destinationPoint for accuracy
    numPoints = length(theta);
    circleLat = zeros(1, numPoints);
    circleLon = zeros(1, numPoints);
    bearings = rad2deg(theta); % Convert angles to bearings
    for k = 1:numPoints
        [circleLat(k), circleLon(k)] = destinationPoint(centerLat, centerLon, radius, bearings(k));
    end
end
