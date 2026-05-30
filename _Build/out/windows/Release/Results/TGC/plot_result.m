%% plot_result.m
% Visualize result.dat in the same directory as this script.

clear;
clc;
close all;

%% 1. Read result.dat
thisDir = fileparts(mfilename('fullpath'));
dataFile = fullfile(thisDir, 'result.dat');

if ~isfile(dataFile)
    error('result.dat was not found next to plot_result.m: %s', dataFile);
end

data = readmatrix(dataFile);

expectedColumnCount = 48;
if isempty(data) || size(data, 2) ~= expectedColumnCount
    error('result.dat must contain exactly %d columns according to FileSaver::save_traj, current column count is %d.', ...
        expectedColumnCount, size(data, 2));
end

cols = getColumnMap();
routeMask = data(:, cols.t) < 0;
routeData = data(routeMask, :);
trajData = data(~routeMask, :);

if isempty(trajData)
    error('result.dat does not contain trajectory rows.');
end

t = trajData(:, cols.t);
kmScale = 1e-3;   % m -> km

%% 2. Trajectory and target
figure('Name', 'TGC Trajectory', 'Color', 'w');
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

% Convert position-related quantities from m to km for visualization.
% Column convention: posN / posE / posU correspond to North / East / Up.
trajN = trajData(:, cols.posN) * kmScale;
trajE = trajData(:, cols.posE) * kmScale;
trajU = trajData(:, cols.posU) * kmScale;

targetN0 = trajData(1, cols.targetN) * kmScale;
targetE0 = trajData(1, cols.targetE) * kmScale;
targetU0 = trajData(1, cols.targetU) * kmScale;

allN = [trajN; targetN0];
allE = [trajE; targetE0];
allU = [trajU; targetU0];

nexttile;
plot3(trajN, trajE, trajU, 'LineWidth', 1.5);
hold on;
plot3(trajN(1), trajE(1), trajU(1), 'bo', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'b');
plot3(targetN0, targetE0, targetU0, 'rp', ...
    'MarkerSize', 12, 'MarkerFaceColor', 'r');

legendEntries = {'Trajectory', 'Launch Point', 'Target Point'};

if ~isempty(routeData)
    routeN = routeData(:, cols.posN) * kmScale;
    routeE = routeData(:, cols.posE) * kmScale;
    routeU = routeData(:, cols.posU) * kmScale;

    plot3(routeN, routeE, routeU, 'rs', ...
        'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineStyle', 'none');

    allN = [allN; routeN];
    allE = [allE; routeE];
    allU = [allU; routeU];

    legendEntries{end + 1} = 'Route Points';
end

grid on;
box on;
xlabel('North (km)');
ylabel('East (km)');
zlabel('Up (km)');
title('3D Trajectory');

% The trajectory has a much larger Up-direction scale than North/East.
% Avoid axis equal here; otherwise the horizontal motion becomes visually compressed.
applyPaddedAxisLimits3D(allN, allE, allU, 0.05);
pbaspect([1 1 1.6]);
view(45, 25);
legend(legendEntries, 'Location', 'best');

nexttile;
groundRange = hypot(trajData(:, cols.posN), trajData(:, cols.posE)) * kmScale;
plot(groundRange, trajData(:, cols.llaAlt) * kmScale, 'LineWidth', 1.5);
grid on;
xlabel('Ground Range (km)');
ylabel('Altitude (km)');
title('Ground Range vs Altitude');

nexttile;
plot(t, trajData(:, cols.speed), 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Speed');

nexttile;
plot(t, trajData(:, cols.targetN) * kmScale, 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.targetE) * kmScale, 'LineWidth', 1.0);
plot(t, trajData(:, cols.targetU) * kmScale, 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Target Position in Launch NUE (km)');
title('Target Position');
legend('North', 'East', 'Up', 'Location', 'best');

sgtitle('TGC Result Visualization', 'Interpreter', 'none');

%% 3. Attitude and guidance
figure('Name', 'TGC Attitude and Guidance', 'Color', 'w');
tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(t, trajData(:, cols.eulerYaw), 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.eulerPitch), 'LineWidth', 1.0);
plot(t, trajData(:, cols.eulerRoll), 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Euler Angle (deg)');
title('Attitude Euler Angles');
legend('Yaw', 'Pitch', 'Roll', 'Location', 'best');

nexttile;
plot(t, trajData(:, cols.velTheta), 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.thetaCmd), '--', 'LineWidth', 1.0);
plot(t, trajData(:, cols.velPsi), 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Velocity and Pitch Command Angles');
legend('\theta_v', '\theta_{cmd}', '\psi_v', 'Location', 'best');

nexttile;
plot(t, trajData(:, cols.accCmdBY),'--', 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.bodyAccY), 'LineWidth', 1.0);
plot(t, trajData(:, cols.accCmdBZ),'--', 'LineWidth', 1.0);
plot(t, trajData(:, cols.bodyAccZ), 'LineWidth', 1.0);
plot(t, trajData(:, cols.bodyAccX), 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Acceleration');
title('Body-axis Command and Response');
legend('a_{cmd,b,y}', 'a_y', 'a_{cmd,b,z}', 'a_z', 'a_x', 'Location', 'best');

nexttile;
plot(t, trajData(:, cols.alpha), 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.alphaCmd), '--', 'LineWidth', 1.0);
plot(t, trajData(:, cols.beta), 'LineWidth', 1.0);
plot(t, trajData(:, cols.betaCmd), '--', 'LineWidth', 1.0);
plot(t, trajData(:, cols.accCmdVY), 'LineWidth', 1.0);
plot(t, trajData(:, cols.accCmdVZ), 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Angle / Acceleration');
title('Aerodynamic Angles and Velocity-axis Commands');
legend('\alpha', '\alpha_{cmd}', '\beta', '\beta_{cmd}', 'a_{cmd,v,y}', 'a_{cmd,v,z}', 'Location', 'best');

nexttile;
plot(t, trajData(:, cols.sigmaElv), 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.sigmaAz), 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('LOS Angles');
legend('\sigma_{elv}', '\sigma_{az}', 'Location', 'best');

nexttile;
yyaxis left;
plot(t, trajData(:, cols.sigmaElvDot), 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.sigmaAzDot), 'LineWidth', 1.0);
ylabel('Rate (deg/s)');

yyaxis right;
stairs(t, trajData(:, cols.phaseId), 'LineWidth', 1.0);
ylabel('Phase ID');
grid on;
xlabel('Time (s)');
title('LOS Rates and Phase');
legend('sigma elv dot', 'sigma az dot', 'phase', 'Location', 'best');

sgtitle('TGC Attitude and Guidance', 'Interpreter', 'none');

%% 4. Control and propulsion
figure('Name', 'TGC Control and Propulsion', 'Color', 'w');
tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(t, trajData(:, cols.rudderX), 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.rudderY), 'LineWidth', 1.0);
plot(t, trajData(:, cols.rudderZ), 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Rudder (deg)');
title('Rudder Deflections');
legend('dx', 'dy', 'dz', 'Location', 'best');

nexttile;
plot(t, trajData(:, cols.bodyAccX), 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.bodyAccY), 'LineWidth', 1.0);
plot(t, trajData(:, cols.bodyAccZ), 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Acceleration');
title('Body Acceleration');
legend('a_x', 'a_y', 'a_z', 'Location', 'best');

nexttile;
plot(t, trajData(:, cols.wX), 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.wY), 'LineWidth', 1.0);
plot(t, trajData(:, cols.wZ), 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Angular Rate');
title('Body Angular Rate');
legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'best');

nexttile;
yyaxis left;
plot(t, trajData(:, cols.mass), 'LineWidth', 1.2);
ylabel('Mass (kg)');

yyaxis right;
plot(t, trajData(:, cols.thrust), 'LineWidth', 1.2);
ylabel('Thrust (N)');
grid on;
xlabel('Time (s)');
title('Mass and Thrust');

nexttile;
plot(t, trajData(:, cols.velN), 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.velE), 'LineWidth', 1.0);
plot(t, trajData(:, cols.velU), 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Velocity NUE (m/s)');
title('Velocity Components');
legend('V_N', 'V_E', 'V_U', 'Location', 'best');

nexttile;
plot(t, trajData(:, cols.llaLon), 'LineWidth', 1.0);
hold on;
plot(t, trajData(:, cols.llaLat), 'LineWidth', 1.0);
plot(t, trajData(:, cols.llaAlt), 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('LLA');
title('Longitude, Latitude, Altitude');
legend('Lon (deg)', 'Lat (deg)', 'Alt (m)', 'Location', 'best');

sgtitle('TGC Control and Propulsion', 'Interpreter', 'none');

fprintf('Plot finished: %s\n', dataFile);

%%  Figure : 3D geographic trajectory on real map background
% This figure uses LLA columns, not the local North-East-Up coordinates.
% It requires MATLAB geoglobe/geoplot3 support for a real 3D globe map.
drawGeographic3DTrajectory(trajData, routeData, cols);

%% ============================ Local functions ============================
function cols = getColumnMap()
cols.t = 1;
cols.posN = 2;
cols.posU = 3;
cols.posE = 4;
cols.speed = 5;
cols.eulerYaw = 6;
cols.eulerPitch = 7;
cols.eulerRoll = 8;
cols.velTheta = 9;
cols.thetaCmd = 10;
cols.velPsi = 11;
cols.alpha = 12;
cols.beta = 13;
cols.alphaCmd = 14;
cols.betaCmd = 15;
cols.bodyAccX = 16;
cols.bodyAccY = 17;
cols.bodyAccZ = 18;
cols.mass = 19;
cols.thrust = 20;
cols.accCmdBY = 21;
cols.accCmdBZ = 22;
cols.accCmdVX = 23;
cols.accCmdVY = 24;
cols.accCmdVZ = 25;
cols.targetN = 26;
cols.targetU = 27;
cols.targetE = 28;
cols.targetLon = 29;
cols.targetLat = 30;
cols.targetAlt = 31;
cols.rudderX = 32;
cols.rudderY = 33;
cols.rudderZ = 34;
cols.velN = 35;
cols.velU = 36;
cols.velE = 37;
cols.wX = 38;
cols.wY = 39;
cols.wZ = 40;
cols.llaLon = 41;
cols.llaLat = 42;
cols.llaAlt = 43;
cols.sigmaElv = 44;
cols.sigmaAz = 45;
cols.sigmaElvDot = 46;
cols.sigmaAzDot = 47;
cols.phaseId = 48;
end

function applyPaddedAxisLimits3D(xData, yData, zData, padRatio)
validMask = isfinite(xData) & isfinite(yData) & isfinite(zData);
xData = xData(validMask);
yData = yData(validMask);
zData = zData(validMask);

if isempty(xData) || isempty(yData) || isempty(zData)
    axis tight;
    return;
end

xRange = max(xData) - min(xData);
yRange = max(yData) - min(yData);
zRange = max(zData) - min(zData);

xPad = max(xRange * padRatio, 1e-3);
yPad = max(yRange * padRatio, 1e-3);
zPad = max(zRange * padRatio, 1e-3);

xlim([min(xData) - xPad, max(xData) + xPad]);
ylim([min(yData) - yPad, max(yData) + yPad]);
zlim([min(zData) - zPad, max(zData) + zPad]);
end

function drawGeographic3DTrajectory(trajData, routeData, cols)
lat = trajData(:, cols.llaLat);
lon = trajData(:, cols.llaLon);
alt = trajData(:, cols.llaAlt);

validMask = isfinite(lat) & isfinite(lon) & isfinite(alt);
lat = lat(validMask);
lon = lon(validMask);
alt = alt(validMask);

if isempty(lat)
    warning('No valid LLA trajectory samples are available for the 3D map.');
    return;
end

launchLat = lat(1);
launchLon = lon(1);
launchAlt = alt(1);
targetLat = trajData(1, cols.targetLat);
targetLon = trajData(1, cols.targetLon);
targetAlt = trajData(1, cols.targetAlt);
hasTargetPoint = all(isfinite([targetLat, targetLon, targetAlt]));

if exist('geoglobe', 'file') == 2 && exist('geoplot3', 'file') == 2
    try
        fig = uifigure('Name', 'TGC 3D Geographic Trajectory', 'Color', 'w');
        gl = geoglobe(fig, 'Basemap', 'satellite');
        hold(gl, 'on');

        geoplot3(gl, lat, lon, alt, 'r', 'LineWidth', 2);
        geoplot3(gl, launchLat, launchLon, launchAlt, 'bo', 'MarkerSize', 8);
        geoplot3(gl, lat(end), lon(end), alt(end), 'kx', 'MarkerSize', 8);
        if hasTargetPoint
            geoplot3(gl, targetLat, targetLon, targetAlt, ...
                'LineStyle', 'none', 'Marker', 'p', 'MarkerSize', 12, ...
                'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
        end

        if ~isempty(routeData)
            routeLat = routeData(:, cols.llaLat);
            routeLon = routeData(:, cols.llaLon);
            routeAlt = routeData(:, cols.llaAlt);
            routeMask = isfinite(routeLat) & isfinite(routeLon) & isfinite(routeAlt);
            if any(routeMask)
                geoplot3(gl, routeLat(routeMask), routeLon(routeMask), routeAlt(routeMask), 'ko', 'MarkerSize', 6);
            end
        end

        fprintf('Generated 3D geographic trajectory on satellite map background.\n');
        return;
    catch mapErr
        warning('3D geoglobe map failed: %s. Falling back to 2D geographic map.', mapErr.message);
    end
end

fig = figure('Name', 'TGC Geographic Trajectory Map', 'Color', 'w');
gx = geoaxes(fig);
geoplot(gx, lat, lon, 'r-', 'LineWidth', 2);
hold(gx, 'on');
geoscatter(gx, launchLat, launchLon, 36, 'b', 'filled');
geoscatter(gx, lat(end), lon(end), 36, 'k');
if hasTargetPoint
    geoscatter(gx, targetLat, targetLon, 80, 'r', 'p', 'filled');
end
try
    geobasemap(gx, 'satellite');
catch
    geobasemap(gx, 'topographic');
end
title(gx, 'Geographic Trajectory');

end
