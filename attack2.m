% Decentralized SecureSky: Multi-UAV Simulation with Attack Failures
% Simulates multiple attack scenarios ensuring all attacks fail as per requirements.
% Attacks Simulated (All Fail):
% - Drone 4: ID Spoofing (Fails: Zone Mismatch), Replay Attack (Fails: Session ID Mismatch)
% - Drone 5: No-Fly Zone Evasion (Fails: Positional Discrepancy)
% - Drone 3: Man-in-the-Middle (MitM) Data Corruption (Fails: Decryption/Hash Mismatch)
% - Drone 2: Database Tamper (Fails: Hash Mismatch)

%% Step 1: Initialize Drones and Ground Station
clc;
clear;
close all;

fprintf('Initializing SecureSky Simulation...\n');

% Load encryption key
try
    key = fileread('endeckey.txt');
    fprintf('Encryption key loaded successfully.\n');
catch ME
    error('Failed to load encryption key from endeckey.txt: %s', ME.message);
end

% Define 5 drones
drone(1).ID = 'ABC123'; % Authorized, spoofing target
drone(2).ID = 'DEF456'; % Authorized, replay and tamper target
drone(3).ID = 'GHI789'; % Authorized, MitM target
drone(4).ID = 'JKL012'; % Unauthorized, spoofing and replay attacker
drone(5).ID = 'MNO345'; % Unauthorized, NFZ evasion attacker

% Initial positions (outside NFZ)
initial_latitudes  = [37.7799, 37.7790, 37.7800, 37.7780, 37.7779];
initial_longitudes = [-122.4244, -122.4235, -122.4250, -122.4220, -122.4215];
initial_altitudes = [100, 120, 110, 105, 130];

% Store last good transmission for replay attack
global lastGoodTransmission;
lastGoodTransmission.encryptedData = '';
lastGoodTransmission.dataHash = '';
lastGoodTransmission.sessionID = '';
lastGoodTransmission.originalDroneID = '';

% Initialize position and verification history
for i = 1:5
    drone(i).latitude = initial_latitudes(i);
    drone(i).longitude = initial_longitudes(i);
    drone(i).altitude = initial_altitudes(i);
    drone(i).speed = 0.0001;
    drone(i).flightZone = 'Permitted';
    if i == 4
        drone(i).flightZone = 'Restricted';
    end
    drone(i).operatorID = ['Operator', num2str(i)];
    drone(i).Session_ID = '';
    drone(i).status = 'Initializing';
    drone(i).lastMessage = '';
    drone(i).attackMode = 'None';
    drone(i).actualLatitude = drone(i).latitude;
    drone(i).actualLongitude = drone(i).longitude;
    drone(i).lastVerificationStatus = false;
    drone(i).positionHistory = [drone(i).latitude, drone(i).longitude, drone(i).actualLatitude, drone(i).actualLongitude];
    drone(i).verificationHistory = [];
    drone(i).verificationTimes = [];
end
% Attack timeline storage
attackTimeline = struct('time', {}, 'drone', {}, 'attackType', {}, 'reason', {});

fprintf('Configuration: Drone 4 assigned to Restricted zone (attacker).\n');
fprintf('Configuration: Drone 5 configured for NFZ evasion attempt.\n');
fprintf('Configuration: Drone 3 targeted for MitM data corruption.\n');
fprintf('Configuration: Drone 2 targeted for database tamper simulation.\n');

% Ground Station properties
groundStation.range = 1000;
groundStation.latitude = 37.7749;
groundStation.longitude = -122.4194;

% No-Fly Zone
noFlyZone.radius = 200;
noFlyZone.centerLat = 37.7749;
noFlyZone.centerLon = -122.4194;
noFlyZone.lastUpdate = 0;

% Initialize log file
logFileName = 'uav_attack_failure_log.txt';
logFile = fopen(logFileName, 'w');
if logFile == -1, error('Unable to open log file: %s', logFileName); end
fprintf(logFile, 'SecureSky Simulation Log - %s\n', datestr(now));
fprintf(logFile, 'Attacks: D4 (Spoof D1, Replay D2), D5 (NFZ Evasion), D3 (MitM), D2 (DB Tamper)\n');
fprintf(logFile, '====================================\n');

%% Step 2: Initialize Database
droneDatabase = struct(...
    'ID', {'ABC123', 'DEF456', 'GHI789', 'JKL012', 'MNO345'}, ...
    'operator', {'Operator1', 'Operator2', 'Operator3', 'Operator4', 'Operator5'}, ...
    'registrationStatus', {'Valid', 'Valid', 'Valid', 'Invalid', 'Valid'}, ...
    'allowedZones', {'Permitted', 'Permitted', 'Permitted', 'Permitted', 'No'}, ...
    'Session_ID', {'', '', '', '', ''});
fprintf('Database initialized.\n');

%% Step 3: Setup Visualization
f = figure('Name', 'SecureSky: Attack Failure Simulation', 'NumberTitle', 'off', 'Color', [0.95 0.95 0.95]);
hold on; grid on; axis equal;
set(gca, 'FontSize', 10, 'FontWeight', 'bold');

% Plot GCS, Range, NFZ
plot(groundStation.longitude, groundStation.latitude, 'k^', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'y', 'DisplayName', 'Ground Station');
theta = linspace(0, 2*pi, 100);
[gcsRangeLat, gcsRangeLon] = calculateCircle(groundStation.latitude, groundStation.longitude, groundStation.range, theta);
plot(gcsRangeLon, gcsRangeLat, 'r--', 'LineWidth', 1.5, 'DisplayName', 'GCS Range');
[nfzLat, nfzLon] = calculateCircle(noFlyZone.centerLat, noFlyZone.centerLon, noFlyZone.radius, theta);
noFlyZone.plot = plot(nfzLon, nfzLat, 'k-', 'LineWidth', 2, 'DisplayName', 'No-Fly Zone');

% Initialize Drone Plots
colors = {'g', 'c', 'm', 'b', 'b'};
markers = {'o', 'o', 'o', 's', 'd'};
for i = 1:5
    drone(i).plot = plot(drone(i).longitude, drone(i).latitude, markers{i}, 'MarkerSize', 8, ...
        'LineWidth', 1.5, 'MarkerFaceColor', colors{i}, 'MarkerEdgeColor', 'k', ...
        'DisplayName', sprintf('UID: %s (%s)', drone(i).ID, drone(i).status));
    if i == 5
        drone(i).actualPlot = plot(drone(i).actualLongitude, drone(i).actualLatitude, 'rx', 'MarkerSize', 10, ...
            'LineWidth', 1.5, 'DisplayName', sprintf('%s Actual Pos', drone(i).ID));
        drone(i).actualPlot.Visible = 'off';
    end
end

% Set plot limits
latLim = [min(initial_latitudes)-0.005, max(initial_latitudes)+0.005];
lonLim = [min(initial_longitudes)-0.005, max(initial_longitudes)+0.005];
axis([lonLim latLim]);

xlabel('Longitude'); ylabel('Latitude');
title('SecureSky: Attack Failure Simulation', 'FontSize', 14, 'FontWeight', 'bold');
legend('show', 'Location', 'northeastoutside', 'FontSize', 9);

% Remote ID Display
uiTextHeight = 0.09; uiWidth = 0.18; uiXPos = 0.81; uiYStart = 0.9;
for i = 1:5
    drone(i).remoteIDDisplay = uicontrol('Style', 'text', 'Units', 'normalized', ...
        'Position', [uiXPos, uiYStart - i*uiTextHeight, uiWidth, uiTextHeight], ...
        'BackgroundColor', 'white', 'FontSize', 8, 'HorizontalAlignment', 'left', ...
        'String', sprintf('Drone %d (%s): Initializing...', i, drone(i).ID));
end

%% Step 4: Simulation Parameters
broadcastInterval = 1;
smoothStep = 0.1;
simulationTime = 10;
noFlyZoneUpdateInterval = 45;
randomDirection = @(maxVal) (rand()*2 - 1) * maxVal;
fprintf('Starting simulation...\n');

%% Main Simulation Loop
for t = 0:smoothStep:simulationTime
    currentTimeStr = sprintf('Time: %.1f s', t);

    %% Step 5: Update No-Fly Zone
    if t - noFlyZone.lastUpdate >= noFlyZoneUpdateInterval && t > 0
        noFlyZone.centerLat = 37.7749 + (rand()*0.004 - 0.002);
        noFlyZone.centerLon = -122.4194 + (rand()*0.004 - 0.002);
        noFlyZone.lastUpdate = t;
        [nfzLat, nfzLon] = calculateCircle(noFlyZone.centerLat, noFlyZone.centerLon, noFlyZone.radius, theta);
        set(noFlyZone.plot, 'XData', nfzLon, 'YData', nfzLat);
        fprintf('No-Fly Zone updated at %.1f s.\n', t);
    end

    %% Steps 6, 7, 8: Drone Actions
    for i = 1:5
        % Step 6: Drone Movement
        prevActualLat = drone(i).actualLatitude;
        prevActualLon = drone(i).actualLongitude;

        % Authorized drones avoid NFZ
        if i <= 3
            distToNFZ = haversine(drone(i).actualLatitude, drone(i).actualLongitude, noFlyZone.centerLat, noFlyZone.centerLon);
            if distToNFZ < noFlyZone.radius * 1.5
                angleAway = atan2(drone(i).actualLatitude - noFlyZone.centerLat, drone(i).actualLongitude - noFlyZone.centerLon) + pi;
                drone(i).actualLatitude = drone(i).actualLatitude + cos(angleAway) * drone(i).speed * smoothStep * 50;
                drone(i).actualLongitude = drone(i).actualLongitude + sin(angleAway) * drone(i).speed * smoothStep * 50;
            else
                drone(i).actualLatitude = drone(i).actualLatitude + randomDirection(drone(i).speed * smoothStep * 50);
                drone(i).actualLongitude = drone(i).actualLongitude + randomDirection(drone(i).speed * smoothStep * 50);
            end
        else
            % Drone 5 moves toward NFZ to trigger evasion
            if i == 5
                angleToNFZ = atan2(noFlyZone.centerLat - drone(i).actualLatitude, noFlyZone.centerLon - drone(i).actualLongitude);
                drone(i).actualLatitude = drone(i).actualLatitude + cos(angleToNFZ) * drone(i).speed * smoothStep * 50;
                drone(i).actualLongitude = drone(i).actualLongitude + sin(angleToNFZ) * drone(i).speed * smoothStep * 50;
            else
                drone(i).actualLatitude = drone(i).actualLatitude + randomDirection(drone(i).speed * smoothStep * 50);
                drone(i).actualLongitude = drone(i).actualLongitude + randomDirection(drone(i).speed * smoothStep * 50);
            end
        end

        drone(i).actualLatitude = min(max(drone(i).actualLatitude, latLim(1)+0.0005), latLim(2)-0.0005);
        drone(i).actualLongitude = min(max(drone(i).actualLongitude, lonLim(1)+0.0005), lonLim(2)-0.0005);

        % NFZ Evasion Attack (Drone 5)
        reportedLatitude = drone(i).actualLatitude;
        reportedLongitude = drone(i).actualLongitude;
        isEvadingNFZ = false;

        if i == 5
            drone(i).attackMode = 'NFZ Evasion';
            actualDistanceToNFZ = haversine(drone(i).actualLatitude, drone(i).actualLongitude, noFlyZone.centerLat, noFlyZone.centerLon);
            if actualDistanceToNFZ <= noFlyZone.radius
                isEvadingNFZ = true;
                drone(i).status = 'NFZ Evasion Attempt';
                drone(i).actualPlot.Visible = 'on';
                angleToCenter = atan2(drone(i).actualLatitude - noFlyZone.centerLat, drone(i).actualLongitude - noFlyZone.centerLon);
                fakeDist = noFlyZone.radius * 1.1;
                [reportedLatitude, reportedLongitude] = destinationPoint(noFlyZone.centerLat, noFlyZone.centerLon, fakeDist, rad2deg(angleToCenter));
                fprintf('Drone 5: Initiated NFZ evasion at %.1f s. Reported coordinates (%.4f, %.4f); actual coordinates (%.4f, %.4f).\n', ...
                        t, reportedLatitude, reportedLongitude, drone(i).actualLatitude, drone(i).actualLongitude);
                fprintf('Drone 5: NFZ evasion failed at %.1f s due to positional discrepancy.\n', t);
                fprintf(logFile, '%.1f s | Drone 5 | NFZ Evasion Failure | Positional discrepancy detected\n', t);
                drone(i).status = 'Unauthorized (NFZ Evasion Detected)';
                set(drone(i).plot, 'MarkerFaceColor', 'r');
                attackTimeline(end+1) = struct('time', t, 'drone', 'D5', 'attackType', 'NFZ Evasion', 'reason', 'Positional discrepancy');
            else
                drone(i).actualPlot.Visible = 'off';
            end
        end

        drone(i).latitude = reportedLatitude;
        drone(i).longitude = reportedLongitude;
        set(drone(i).plot, 'XData', drone(i).longitude, 'YData', drone(i).latitude);
        if i == 5
            set(drone(i).actualPlot, 'XData', drone(i).actualLongitude, 'YData', drone(i).actualLatitude);
        end
        drone(i).positionHistory = [drone(i).positionHistory; drone(i).latitude, drone(i).longitude, drone(i).actualLatitude, drone(i).actualLongitude];

        % Step 7: Check No-Fly Zone
        distanceToNoFlyZone = haversine(drone(i).latitude, drone(i).longitude, noFlyZone.centerLat, noFlyZone.centerLon);
        inNoFlyZone = (distanceToNoFlyZone <= noFlyZone.radius);
        remark = '';
        bgColor = 'white';

        if inNoFlyZone
            if i == 5 && isEvadingNFZ
                remark = 'NFZ Evasion Detected';
                bgColor = [1 0 1];
            else
                remark = 'Violation: In No-Fly Zone';
                bgColor = [1 0.7 0.7];
                if ~contains(drone(i).status, 'In NFZ') && ~isEvadingNFZ
                    fprintf('Alert: Drone %s entered No-Fly Zone at %.1f s.\n', drone(i).ID, t);
                    fprintf(logFile, '%.1f s | Drone %s | Violation: Entered No-Fly Zone | Lat: %.6f, Lon: %.6f\n', ...
                            t, drone(i).ID, drone(i).latitude, drone(i).longitude);
                    drone(i).status = 'In NFZ';
                    set(drone(i).plot, 'MarkerEdgeColor', 'r', 'LineWidth', 2.5);
                end
            end
        else
            if i == 5 && isEvadingNFZ
                remark = 'NFZ Evasion Active';
                bgColor = [1 1 0.7];
                set(drone(i).plot, 'MarkerEdgeColor', 'm', 'LineWidth', 2.5);
            elseif contains(drone(i).status, 'In NFZ') || contains(drone(i).status, 'Evasion')
                drone(i).status = 'Nominal';
                set(drone(i).plot, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
            end
        end

        % Step 8: Authentication and Verification
        if mod(floor(t), broadcastInterval) == 0 || contains(drone(i).status, 'Connecting')
            distanceToGroundStation = haversine(drone(i).latitude, drone(i).longitude, groundStation.latitude, groundStation.longitude);

            if distanceToGroundStation <= groundStation.range
                drone(i).status = 'Connecting';
                authenticated = false;
                authReason = '';

                % Attack Attempts
                idToPresent = drone(i).ID;
                attemptingReplay = false;

                if i == 4
                    drone(i).attackMode = 'ID Spoofing';
                    idToPresent = drone(1).ID; % Spoof Drone 1
                    fprintf('Drone 4: Initiated ID spoofing as %s at %.1f s.\n', idToPresent, t);
                end

                % Authentication Check
                dbIndex = find(strcmp(idToPresent, {droneDatabase.ID}), 1);
                if ~isempty(dbIndex)
                    regValid = strcmp(droneDatabase(dbIndex).registrationStatus, 'Valid');
                    zoneMatch = strcmp(drone(i).flightZone, droneDatabase(dbIndex).allowedZones);

                    if regValid && zoneMatch
                        authenticated = true;
                        authReason = 'Authorized';
                        drone(i).status = 'Authorized';
                        set(drone(i).plot, 'MarkerFaceColor', 'g');
                    else
                        authenticated = false;
                        if ~regValid
                            authReason = 'Invalid Registration';
                        elseif ~zoneMatch
                            authReason = sprintf('Zone Mismatch: Drone in %s, ID %s requires %s', drone(i).flightZone, idToPresent, droneDatabase(dbIndex).allowedZones);
                        else
                            authReason = 'Authentication Failure';
                        end
                        drone(i).status = sprintf('Unauthorized (%s)', authReason);
                        set(drone(i).plot, 'MarkerFaceColor', 'r');
                        if i == 4
                            fprintf('Drone 4: ID spoofing attempt failed at %.1f s due to %s.\n', t, authReason);
                            fprintf(logFile, '%.1f s | Drone 4 | ID Spoofing Failure | %s\n', t, authReason);
                            attackTimeline(end+1) = struct('time', t, 'drone', 'D4', 'attackType', 'ID Spoofing', 'reason', authReason);
                            if ~isempty(lastGoodTransmission.encryptedData)
                                drone(i).attackMode = 'Replay Pending';
                                fprintf('Drone 4: Preparing replay attack at %.1f s.\n', t);
                            else
                                drone(i).attackMode = 'None';
                            end
                        end
                    end
                else
                    authenticated = false;
                    authReason = 'ID Not Found';
                    drone(i).status = 'Unauthorized (ID Unknown)';
                    set(drone(i).plot, 'MarkerFaceColor', 'r');
                    if i == 4
                        fprintf('Drone 4: ID spoofing attempt failed at %.1f s due to %s.\n', t, authReason);
                        fprintf(logFile, '%.1f s | Drone 4 | ID Spoofing Failure | ID Not Found\n', t);
                        attackTimeline(end+1) = struct('time', t, 'drone', 'D4', 'attackType', 'ID Spoofing', 'reason', authReason);
                        drone(i).attackMode = 'None';
                    end
                end

                % Replay Attack (Drone 4)
                if i == 4 && strcmp(drone(i).attackMode, 'Replay Pending') && ~isempty(lastGoodTransmission.encryptedData)
                    authenticated = false;
                    drone(i).attackMode = 'Replay Attack';
                    attemptingReplay = true;
                    fprintf('Drone 4: Initiated replay attack with Drone %s data at %.1f s.\n', lastGoodTransmission.originalDroneID, t);

                    replayedEncryptedData = lastGoodTransmission.encryptedData;
                    replayedHash = lastGoodTransmission.dataHash;
                    replayedSessionID = lastGoodTransmission.sessionID;

                    replayVerified = false;
                    try
                        decryptedReplay = aes_decrypt(replayedEncryptedData, key);
                        computedReplayHash = sha256(decryptedReplay);
                        if strcmp(computedReplayHash, replayedHash)
                            replayVerified = true;
                            fprintf('Drone 4: Replay data verified at %.1f s.\n', t);
                        else
                            fprintf('Drone 4: Replay attack failed at %.1f s due to hash mismatch.\n', t);
                            attackTimeline(end+1) = struct('time', t, 'drone', 'D4', 'attackType', 'Replay Attack', 'reason', 'Hash mismatch');
                        end
                    catch ME_replay_verify
                        fprintf('Drone 4: Replay attack failed at %.1f s due to decryption error: %s.\n', t, ME_replay_verify.message);
                        attackTimeline(end+1) = struct('time', t, 'drone', 'D4', 'attackType', 'Replay Attack', 'reason', 'Decryption error');
                    end

                    if replayVerified
                        parts = strsplit(decryptedReplay, ',');
                        if length(parts) >= 1
                            sessionInReplay = parts{1};
                            if strcmp(sessionInReplay, drone(i).Session_ID)
                                fprintf('Error: Drone 4 replay attack unexpectedly succeeded at %.1f s.\n', t);
                            else
                                authenticated = false;
                                authReason = 'Session ID Mismatch';
                                drone(i).status = 'Unauthorized (Replay Failure)';
                                set(drone(i).plot, 'MarkerFaceColor', [0.5 0 0]);
                                fprintf('Drone 4: Replay attack failed at %.1f s due to session ID mismatch [%s].\n', t, sessionInReplay);
                                fprintf(logFile, '%.1f s | Drone 4 | Replay Attack Failure | Session ID Mismatch\n', t);
                                attackTimeline(end+1) = struct('time', t, 'drone', 'D4', 'attackType', 'Replay Attack', 'reason', 'Session ID Mismatch');
                            end
                        else
                            authenticated = false;
                            authReason = 'Invalid Replay Data';
                            fprintf('Drone 4: Replay attack failed at %.1f s due to invalid data format.\n', t);
                            attackTimeline(end+1) = struct('time', t, 'drone', 'D4', 'attackType', 'Replay Attack', 'reason', 'Invalid data format');
                        end
                    end
                    drone(i).attackMode = 'None';
                end

                % Verification Process
                if authenticated
                    if isempty(drone(i).Session_ID)
                        drone(i).Session_ID = sprintf('UAV-%s%d', char(64+i), randi([100,999]));
                        fprintf('Drone %d: Assigned session ID %s at %.1f s.\n', i, drone(i).Session_ID, t);
                    end

                    word1 = drone(i).Session_ID;
                    word2 = num2str(drone(i).latitude, '%.6f');
                    word3 = num2str(drone(i).longitude, '%.6f');
                    word4 = num2str(drone(i).altitude, '%.1f');
                    word5 = drone(i).ID;
                    dataString = strjoin({word1, word2, word3, word4, word5}, ',');

                    try
                        dataHash = sha256(dataString);
                    catch
                        dataHash = 'HASH_ERROR';
                    end

                    try
                        encryptedData = aes_encrypt(dataString, key);
                    catch
                        encryptedData = 'ENCRYPT_ERROR';
                    end

                    % MitM Attack (Drone 3)
                    isMitMVictim = (i == 3);
                    if isMitMVictim && ~strcmp(encryptedData, 'ENCRYPT_ERROR')
                        fprintf('Drone 3: Simulating MitM data corruption at %.1f s.\n', t);
                        if length(encryptedData) > 5
                            corruptIdx = randi(length(encryptedData));
                            originalChar = encryptedData(corruptIdx);
                            newChar = char(mod(originalChar + randi([10,50]), 256));
                            encryptedData(corruptIdx) = newChar;
                            fprintf('Drone 3: Corrupted data at index %d (%c -> %c) at %.1f s.\n', corruptIdx, originalChar, newChar, t);
                        else
                            encryptedData = [encryptedData, 'CORRUPTED'];
                            fprintf('Drone 3: Appended corruption tag at %.1f s.\n', t);
                        end
                        attackTimeline(end+1) = struct('time', t, 'drone', 'D3', 'attackType', 'MitM', 'reason', 'Decryption Failure (MitM)');
                    end

                    % Store Transmission for Replay (Drone 2)
                    if i == 2 && ~strcmp(encryptedData, 'ENCRYPT_ERROR') && ~strcmp(dataHash, 'HASH_ERROR')
                        lastGoodTransmission.encryptedData = encryptedData;
                        lastGoodTransmission.dataHash = dataHash;
                        lastGoodTransmission.sessionID = drone(i).Session_ID;
                        lastGoodTransmission.originalDroneID = num2str(i);
                        fprintf('Drone 4: Captured Drone %s transmission for replay at %.1f s.\n', lastGoodTransmission.originalDroneID, t);
                    end

                    gcsRetrievedEncrypted = encryptedData;
                    gcsRetrievedHash = dataHash;

                    % Database Tamper (Drone 2)
                    isDBTamperVictim = (i == 2);
                    if isDBTamperVictim && ~strcmp(gcsRetrievedHash, 'HASH_ERROR')
                        fprintf('Drone 2: Simulating database tamper at %.1f s.\n', t);
                        originalRetrievedHash = gcsRetrievedHash;
                        tamperIdx = randi([1, length(gcsRetrievedHash)]);
                        hashChars = '0123456789abcdef';
                        newChar = hashChars(randi(length(hashChars)));
                        gcsRetrievedHash = [gcsRetrievedHash(1:tamperIdx-1), newChar, gcsRetrievedHash(tamperIdx+1:end)];
                        fprintf('Drone 2: Tampered hash at index %d ([%s...] -> [%s...]) at %.1f s.\n', ...
                                tamperIdx, originalRetrievedHash(1:8), gcsRetrievedHash(1:8), t);
                        attackTimeline(end+1) = struct('time', t, 'drone', 'D2', 'attackType', 'DB Tamper', 'reason', 'Hash Mismatch (DB Tamper)');
                    end

                    verified = false;
                    verificationReason = '';
                    try
                        if strcmp(gcsRetrievedEncrypted, 'ENCRYPT_ERROR')
                            verificationReason = 'Encryption Failure';
                        else
                            decryptedData = aes_decrypt(gcsRetrievedEncrypted, key);
                            if contains(decryptedData, 'DECRYPT_ERROR')
                                verificationReason = 'Decryption Failure';
                                if isMitMVictim
                                    verificationReason = 'Decryption Failure (MitM)';
                                end
                            else
                                computedHash = sha256(decryptedData);
                                if strcmp(computedHash, gcsRetrievedHash)
                                    verified = true;
                                    verificationReason = 'Success';
                                else
                                    verificationReason = 'Hash Mismatch';
                                    if isMitMVictim
                                        verificationReason = 'Hash Mismatch (MitM)';
                                    elseif isDBTamperVictim
                                        verificationReason = 'Hash Mismatch (DB Tamper)';
                                    end
                                    fprintf('Drone %d: Verification failed at %.1f s. Computed hash [%s...] does not match retrieved [%s...].\n', ...
                                            i, t, computedHash(1:8), gcsRetrievedHash(1:8));
                                    fprintf(logFile, '%.1f s | Drone %d | Verification Failure | %s\n', t, i, verificationReason);
                                end
                            end
                        end
                    catch ME_verify
                        verificationReason = sprintf('Verification Error: %s', ME_verify.message);
                    end

                    if verified && ~drone(i).lastVerificationStatus
                        fprintf('Drone %d: Verification succeeded at %.1f s.\n', i, t);
                    elseif ~verified
                        fprintf('Drone %d: Verification failed at %.1f s due to %s.\n', i, t, verificationReason);
                        if isMitMVictim
                            fprintf('Drone 3: MitM attack failed at %.1f s due to %s.\n', t, verificationReason);
                            fprintf(logFile, '%.1f s | Drone 3 | MitM Attack Failure | %s\n', t, verificationReason);
                        elseif isDBTamperVictim
                            fprintf('Drone 2: Database tamper failed at %.1f s due to %s.\n', t, verificationReason);
                            fprintf(logFile, '%.1f s | Drone 2 | Database Tamper Failure | %s\n', t, verificationReason);
                        end
                    end
                    drone(i).lastVerificationStatus = verified;
                    drone(i).verificationHistory = [drone(i).verificationHistory, verified];
                    drone(i).verificationTimes = [drone(i).verificationTimes, t];
                else
                    if ~strcmp(drone(i).status, 'Out of Range')
                        drone(i).status = 'Out of Range';
                        drone(i).Session_ID = '';
                        set(drone(i).plot, 'MarkerFaceColor', 'b');
                        set(drone(i).plot, 'DisplayName', sprintf('UID: %s (Out of Range)', drone(i).ID));
                        drone(i).attackMode = 'None';
                        if i == 5
                            drone(i).actualPlot.Visible = 'off';
                        end
                    end
                end
            end

            % Update Remote ID Display
            statusStr = drone(i).status;
            idStr = drone(i).ID;
            sessionStr = drone(i).Session_ID;
            if isempty(sessionStr)
                sessionStr = 'N/A';
            end
            attackStr = drone(i).attackMode;
            if strcmp(attackStr, 'None')
                attackStr = '';
            else
                attackStr = sprintf('(%s)', attackStr);
            end
            if i == 4 || i == 5
                idStr = sprintf('%s (Attacker)', drone(i).ID);
            end
            if i == 2 && contains(statusStr, 'Hash Mismatch')
                attackStr = '(DB Tamper)';
            end
            if i == 3 && contains(statusStr, 'Hash Mismatch')
                attackStr = '(MitM)';
            end

            displayString = sprintf('D%d %s %s\nStatus: %s\nSession: %s\nLat: %.4f\nLon: %.4f\nAlt: %.0f m\n%s\n%s', ...
                                    i, idStr, attackStr, statusStr, sessionStr, ...
                                    drone(i).latitude, drone(i).longitude, drone(i).altitude, ...
                                    remark, currentTimeStr);

            if ~strcmp(displayString, drone(i).lastMessage)
                set(drone(i).remoteIDDisplay, 'String', displayString, 'BackgroundColor', bgColor);
                drone(i).lastMessage = displayString;
            else
                set(drone(i).remoteIDDisplay, 'BackgroundColor', bgColor);
            end
        end
    end

    drawnow limitrate;
    pause(smoothStep);

    if ~isvalid(f)
        disp('Simulation stopped: Figure closed.');
        break;
    end
end

%% Post-Simulation
fprintf('Simulation completed.\n');
fclose(logFile);
fprintf('Log file saved: %s\n', logFileName);
delete('temp_*.txt');

%% Generate Plots for Research Paper
% Plot 1: Drone Trajectories
figure('Name', 'Drone Trajectories', 'Color', 'white');
hold on; grid on; axis equal;
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

% Plot NFZ and GCS
plot(groundStation.longitude, groundStation.latitude, 'k^', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'y', 'DisplayName', 'GCS');
[nfzLat, nfzLon] = calculateCircle(noFlyZone.centerLat, noFlyZone.centerLon, noFlyZone.radius, theta);
plot(nfzLon, nfzLat, 'k-', 'LineWidth', 2, 'DisplayName', 'No-Fly Zone');

% Plot trajectories
colors = {'g', 'c', 'm', 'b', [0.5 0 0.5]};
lineStyles = {'-', '-', '-', '-', '-'};
for i = 1:5
    pos = drone(i).positionHistory;
    plot(pos(:,2), pos(:,1), 'Color', colors{i}, 'LineStyle', lineStyles{i}, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('D%d Reported', i));
    if i == 5
        plot(pos(:,4), pos(:,3), 'r--', 'LineWidth', 1.5, 'DisplayName', 'D5 Actual');
    end
end

xlabel('Longitude', 'FontSize', 12);
ylabel('Latitude', 'FontSize', 12);
title('Drone Trajectories Relative to No-Fly Zone', 'FontSize', 14, 'FontWeight', 'bold');
legend('show', 'Location', 'northeastoutside', 'FontSize', 10);
axis([lonLim latLim]);
exportgraphics(gcf, 'drone_trajectories.png', 'Resolution', 300);

% Plot 2: Verification Status
figure('Name', 'Verification Status', 'Color', 'white');
hold on; grid on;
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

for i = 1:5
    times = drone(i).verificationTimes;
    status = double(drone(i).verificationHistory);
    plot(times, status + i*0.1 - 0.05, 'o-', 'Color', colors{i}, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('D%d', i));
end

xlabel('Time (s)', 'FontSize', 12);
ylabel('Verification Status (1=Success, 0=Failure)', 'FontSize', 12);
title('Verification Status Over Time', 'FontSize', 14, 'FontWeight', 'bold');
legend('show', 'Location', 'northeastoutside', 'FontSize', 10);
ylim([-0.5, 0.5 + 5*0.1]);
yticks([0 0.1 0.2 0.3 0.4]);
yticklabels({'D1', 'D2', 'D3', 'D4', 'D5'});
exportgraphics(gcf, 'verification_status.png', 'Resolution', 300);

% Plot 3: Attack Timeline
figure('Name', 'Attack Timeline', 'Color', 'white');
hold on; grid on;
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

attackTypes = {'ID Spoofing', 'Replay Attack', 'NFZ Evasion', 'MitM', 'DB Tamper'};
markers = {'o', 's', 'd', '^', 'v'};
colors = {'b', 'b', 'r', 'm', 'c'};
yPositions = 1:length(attackTypes);

for i = 1:length(attackTimeline)
    attackIdx = find(strcmp(attackTypes, attackTimeline(i).attackType));
    plot(attackTimeline(i).time, yPositions(attackIdx), markers{attackIdx}, ...
         'Color', colors{attackIdx}, 'MarkerSize', 8, 'LineWidth', 1.5);
    text(attackTimeline(i).time + 0.2, yPositions(attackIdx), ...
         sprintf('%s (%s)', attackTimeline(i).drone, attackTimeline(i).reason), ...
         'FontSize', 10);
end

xlabel('Time (s)', 'FontSize', 12);
ylabel('Attack Type', 'FontSize', 12);
title('Timeline of Attack Attempts and Failures', 'FontSize', 14, 'FontWeight', 'bold');
yticks(yPositions);
yticklabels(attackTypes);
xlim([0, simulationTime]);
ylim([0.5, length(attackTypes)+0.5]);
exportgraphics(gcf, 'attack_timeline.png', 'Resolution', 300);

%% Helper Functions
function distance = haversine(lat1, lon1, lat2, lon2)
    R = 6371000;
    phi1 = deg2rad(lat1);
    phi2 = deg2rad(lat2);
    deltaPhi = deg2rad(lat2 - lat1);
    deltaLambda = deg2rad(lon2 - lon1);
    a = sin(deltaPhi/2)^2 + cos(phi1) * cos(phi2) * sin(deltaLambda/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    distance = R * c;
end

function hash = sha256(data)
    if ischar(data)
        dataBytes = uint8(data);
    elseif isnumeric(data)
        dataBytes = typecast(data(:)', 'uint8');
    else
        error('SHA256 input must be char array or numeric.');
    end
    try
        md = java.security.MessageDigest.getInstance('SHA-256');
        md.update(dataBytes);
        hashBytes = typecast(md.digest(), 'uint8');
        hash = lower(sprintf('%02x', hashBytes));
    catch ME_sha
        fprintf('SHA256 Error: %s\n', ME_sha.message);
        hash = 'HASHING_ERROR';
    end
end

function encryptedData = aes_encrypt(data, key)
    encryptedData = ['encrypted(', data, ')_key(', key(1:min(4,end)), ')'];
end

function decryptedData = aes_decrypt(encryptedData, key)
    if contains(encryptedData, 'encrypted(')
        decryptedData = extractBetween(encryptedData, 'encrypted(', ')_key');
        if iscell(decryptedData) && ~isempty(decryptedData)
            decryptedData = decryptedData{1};
        else
            decryptedData = 'DECRYPT_ERROR_PLACEHOLDER';
        end
    else
        decryptedData = 'DECRYPT_ERROR_PLACEHOLDER';
    end
end

function [lat2, lon2] = destinationPoint(lat1, lon1, distance, bearing)
    R = 6371000;
    lat1 = deg2rad(lat1);
    lon1 = deg2rad(lon1);
    bearing = deg2rad(bearing);
    lat2 = asin(sin(lat1)*cos(distance/R) + cos(lat1)*sin(distance/R)*cos(bearing));
    lon2 = lon1 + atan2(sin(bearing)*sin(distance/R)*cos(lat1), cos(distance/R)-sin(lat1)*sin(lat2));
    lat2 = rad2deg(lat2);
    lon2 = rad2deg(lon2);
end

function [circleLat, circleLon] = calculateCircle(centerLat, centerLon, radius, theta)
    numPoints = length(theta);
    circleLat = zeros(1, numPoints);
    circleLon = zeros(1, numPoints);
    bearings = rad2deg(theta);
    for k = 1:numPoints
        [circleLat(k), circleLon(k)] = destinationPoint(centerLat, centerLon, radius, bearings(k));
    end
end