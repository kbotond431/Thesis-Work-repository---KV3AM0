%% 1 SET UP MAP %%
rng(61, "multFibonacci"); % Set seed and method for random generation
omap = occupancyMap3D;
mapWidth = 250;
mapLength = 250;
omap.FreeThreshold = omap.OccupiedThreshold; % Space is either free or occupied, unknown is treated as occupied

numberOfObstacles = 17;

%% 2 Generate obstacles %%
obstacleNumber = 1;
obstaclePosition = zeros(20, 3);
obstacleSize = zeros(numberOfObstacles,3);

while obstacleNumber <= numberOfObstacles
    width = randi([10 50],1);                                                    
    length = randi([10 50],1);
    height = randi([40 150],1);
    xPosition = randi([0 mapWidth-width],1);
    yPosition = randi([0 mapLength-length],1);
    
    [xObstacle,yObstacle,zObstacle] = meshgrid(xPosition:xPosition+width,yPosition:yPosition+length,0:height);
    xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
    obstaclePosition(obstacleNumber,:) = [xPosition, yPosition, 0];
    obstacleSize(obstacleNumber,:) = [width, length, height];
    
    checkIntersection = false;
    for i = 1:size(xyzObstacles,1)
        if checkOccupancy(omap,xyzObstacles(i,:)) == 1
            checkIntersection = true;
            break
        end
    end
    if checkIntersection
        continue
    end
    
    setOccupancy(omap,xyzObstacles,1)

    obstacleNumber = obstacleNumber + 1;
end

[xGround,yGround,zGround] = meshgrid(0:mapWidth,0:mapLength,0);
xyzGround = [xGround(:) yGround(:) zGround(:)];
setOccupancy(omap,xyzGround,1)

%% 3 Define state space %%
omap3D = copy(omap);
inflate(omap3D, 3.0); % Inflate obstacles for added security

% Define state space bounds: [x; y; z; qw; qx; qy; qz]
bounds = [0 250;    % x
          0 250;    % y
          0 100;    % z
         -1 1;      % qw
         -1 1;      % qx
         -1 1;      % qy
         -1 1];     % qz
ss = stateSpaceSE3(bounds);

% Validator
sv = validatorOccupancyMap3D(ss);
sv.Map = omap3D;
inflate(sv.Map, 3.0);
sv.ValidationDistance = 0.5;

%% 4 Start and goal poses %%
% Convert Euler [yaw pitch roll] to quaternion
startQuat = eul2quat([pi/2 0 0]);  % [yaw pitch roll]
goalQuat  = eul2quat([pi/2 0 0]);

startPose = [0 0 50 startQuat];
goalPose  = [250 250 50 goalQuat];

%% 5 RRT %%
planner1 = plannerRRT(ss, sv);
planner1.MaxConnectionDistance = 20;
[pth1, info1] = plan(planner1, startPose, goalPose);
pthSmoothed1 = shortenpath(pth1, sv);
lenRRT = sum(vecnorm(diff(pthSmoothed1.States(:,1:3))'));
waypoints_RRT_smoothed = [pthSmoothed1.States(:,1), pthSmoothed1.States(:,2), pthSmoothed1.States(:,3)];

%% 6 BiRRT %%
planner2 = plannerBiRRT(ss, sv);
planner2.MaxConnectionDistance = 20;
[pth2, info2] = plan(planner2, startPose, goalPose);
pthSmoothed2 = shortenpath(pth2, sv);
lenBiRRT = sum(vecnorm(diff(pthSmoothed2.States(:,1:3))'));
waypoints_BiRRT_smoothed = [pthSmoothed2.States(:,1), pthSmoothed2.States(:,2), pthSmoothed2.States(:,3)];

%% 7 RRT* %%
planner3 = plannerRRTStar(ss, sv);
planner3.MaxConnectionDistance = 20;
[pth3, info3] = plan(planner3, startPose, goalPose);
pthSmoothed3 = shortenpath(pth3, sv);
lenRRTstar = sum(vecnorm(diff(pthSmoothed3.States(:,1:3))'));
waypoints_RRTstar_smoothed = [pthSmoothed3.States(:,1), pthSmoothed3.States(:,2), pthSmoothed3.States(:,3)];

%% 8 PRM %%
planner4 = plannerPRM(ss,sv);
[pth4,info4] = plan(planner4,startPose,goalPose);
pthSmoothed4 = shortenpath(pth4, sv);
lenPRM = sum(vecnorm(diff(pthSmoothed4.States(:,1:3))'));
waypoints_PRM_smoothed = [pthSmoothed4.States(:,1), pthSmoothed4.States(:,2), pthSmoothed4.States(:,3)];

%% 9 Plot paths %%
figure;
show(omap); hold on;
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Path Planning Comparison');

% Plot all planned paths
plot3(pth1.States(:,1), pth1.States(:,2), pth1.States(:,3), 'r:', 'LineWidth', 1.3, 'DisplayName', 'RRT');
plot3(pth2.States(:,1), pth2.States(:,2), pth2.States(:,3), 'g:', 'LineWidth', 1.3, 'DisplayName', 'BiRRT');
plot3(pth3.States(:,1), pth3.States(:,2), pth3.States(:,3), 'c:', 'LineWidth', 1.3, 'DisplayName', 'RRT*');
plot3(pth4.States(:,1), pth4.States(:,2), pth4.States(:,3), 'y:', 'LineWidth', 1.3, 'DisplayName', 'PRM');


plot3(pthSmoothed1.States(:,1), pthSmoothed1.States(:,2), pthSmoothed1.States(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'RRT smoothed');
plot3(pthSmoothed2.States(:,1), pthSmoothed2.States(:,2), pthSmoothed2.States(:,3), 'g-', 'LineWidth', 2, 'DisplayName', 'BiRRT smoothed');
plot3(pthSmoothed3.States(:,1), pthSmoothed3.States(:,2), pthSmoothed3.States(:,3), 'c-', 'LineWidth', 2, 'DisplayName', 'RRT* smoothed');
plot3(pthSmoothed4.States(:,1), pthSmoothed4.States(:,2), pthSmoothed4.States(:,3), 'y-', 'LineWidth', 2, 'DisplayName', 'PRM smoothed');

% Start and goal
plot3(startPose(1), startPose(2), startPose(3), 'wo', 'MarkerSize', 8, 'DisplayName', 'Start');
plot3(goalPose(1), goalPose(2), goalPose(3), 'wx', 'MarkerSize', 8, 'DisplayName', 'Goal');

legend('','RRT', 'BiRRT', 'RRT*', 'PRM', 'RRT smoothed', 'BiRRT smoothed', 'RRT* smoothed', 'PRM smoothed', 'Start', 'Goal');

%% 10 SIMULATION %%
simTimeRRT = (lenRRT / 30) - 0.12; % Simulation time calculated based on the length of the path and the speed of the UAV, added finetuning
RRTData = sim('RRT_guidance', 'StopTime', num2str(simTimeRRT));

simTimeBiRRT = (lenBiRRT / 30) - 0.475;
BiRRTData = sim('BiRRT_guidance', 'StopTime', num2str(simTimeBiRRT));

simTimeRRTstar = lenRRTstar / 30 - 0.04;
RRTstarData = sim('RRTstar_guidance', 'StopTime', num2str(simTimeRRTstar));

simTimePRM = (lenPRM / 30) - 0.48;
PRMData = sim('PRM_guidance', 'StopTime', num2str(simTimePRM));

%% Error calculation - first steps %%

% === Data ===
actualPath_RRT = RRTData.posRRT;      % Nx3 actual trajectory
refPath_RRT = pthSmoothed1.States(:,1:3);         % Mx3 reference waypoints

actualPath_BiRRT = BiRRTData.posBiRRT;     
refPath_BiRRT = pthSmoothed2.States(:,1:3);         

actualPath_RRTstar = RRTstarData.posRRTstar;     
refPath_RRTstar = pthSmoothed3.States(:,1:3);

actualPath_PRM = PRMData.posPRM;     
refPath_PRM = pthSmoothed4.States(:,1:3);

% --- Interpolate reference path to match actual path length ---

% Helper function for arc-length parameterization
function refInterp = interpolatePathByArcLength(refPath, Npoints)
    % Compute cumulative arc length of reference path
    d = sqrt(sum(diff(refPath).^2, 2));
    s = [0; cumsum(d)];       % cumulative distance
    sq = linspace(0, s(end), Npoints); % target arc-length positions
    % Interpolate each coordinate against arc length
    refInterp = interp1(s, refPath, sq, 'linear');
end

% RRT
refInterp_RRT = interpolatePathByArcLength(refPath_RRT, size(actualPath_RRT,1));

% BiRRT
refInterp_BiRRT = interpolatePathByArcLength(refPath_BiRRT, size(actualPath_BiRRT,1));

% RRT*
refInterp_RRTstar = interpolatePathByArcLength(refPath_RRTstar, size(actualPath_RRTstar,1));

% PRM
refInterp_PRM = interpolatePathByArcLength(refPath_PRM, size(actualPath_PRM,1));

%% 11 Index-based MSE calculation %%

% --- Compute error vectors ---
errors_RRT = actualPath_RRT - refInterp_RRT;
distances_RRT = sqrt(sum(errors_RRT.^2,2));

errors_BiRRT = actualPath_BiRRT - refInterp_BiRRT;
distances_BiRRT = sqrt(sum(errors_BiRRT.^2,2));

errors_RRTstar = actualPath_RRTstar - refInterp_RRTstar;
distances_RRTstar = sqrt(sum(errors_RRTstar.^2,2));

errors_PRM = actualPath_PRM - refInterp_PRM;
distances_PRM = sqrt(sum(errors_PRM.^2,2));

% --- Metrics ---
MSE_RRT = mean(distances_RRT.^2);
RMSE_RRT = sqrt(MSE_RRT);
MaxDev_RRT = max(distances_RRT);

MSE_BiRRT = mean(distances_BiRRT.^2);
RMSE_BiRRT = sqrt(MSE_BiRRT);
MaxDev_BiRRT = max(distances_BiRRT);

MSE_RRTstar = mean(distances_RRTstar.^2);
RMSE_RRTstar = sqrt(MSE_RRTstar);
MaxDev_RRTstar = max(distances_RRTstar);

MSE_PRM = mean(distances_PRM.^2);
RMSE_PRM = sqrt(MSE_PRM);
MaxDev_PRM = max(distances_PRM);

% --- Display results ---
fprintf('Trajectory Tracking Metrics for RRT path:\n');
fprintf('  Length= %.3f m\n', lenRRT);
fprintf('  MSE   = %.3f m^2\n', MSE_RRT);
fprintf('  RMSE  = %.3f m\n', RMSE_RRT);
fprintf('  MaxDev= %.3f m\n', MaxDev_RRT);

fprintf('Trajectory Tracking Metrics for BiRRT path:\n');
fprintf('  Length= %.3f m\n', lenBiRRT);
fprintf('  MSE   = %.3f m^2\n', MSE_BiRRT);
fprintf('  RMSE  = %.3f m\n', RMSE_BiRRT);
fprintf('  MaxDev= %.3f m\n', MaxDev_BiRRT);

fprintf('Trajectory Tracking Metrics for RRTstar path:\n');
fprintf('  Length= %.3f m\n', lenRRTstar);
fprintf('  MSE   = %.3f m^2\n', MSE_RRTstar);
fprintf('  RMSE  = %.3f m\n', RMSE_RRTstar);
fprintf('  MaxDev= %.3f m\n', MaxDev_RRTstar);

fprintf('Trajectory Tracking Metrics for PRM path:\n');
fprintf('  Length= %.3f m\n', lenPRM);
fprintf('  MSE   = %.3f m^2\n', MSE_PRM);
fprintf('  RMSE  = %.3f m\n', RMSE_PRM);
fprintf('  MaxDev= %.3f m\n', MaxDev_PRM);

%% Show index-based comparison %%

figure;
hold on;
grid on;
axis equal;
view(3);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Indexed Path Pairing Visualization');

chosen_reference_path = refInterp_RRT; %Change value to either refInterp_RRT, refInterp_BiRRT, refInterp_RRTstar, refInterp_PRM
chosen_actual_path = actualPath_RRT; %Change value to either actualPath_RRT, actualPath_BiRRT, actualPath_RRTstar, actualPath_PRM

% === Plot paths ===
plot3(chosen_reference_path(:,1), chosen_reference_path(:,2), chosen_reference_path(:,3), 'b-o', 'DisplayName', 'Reference Path');
plot3(chosen_actual_path(:,1), chosen_actual_path(:,2), chosen_actual_path(:,3), 'r-o', 'DisplayName', 'Actual Path');

% === Connect corresponding points ===
minLen = min(size(chosen_actual_path,1), size(chosen_reference_path,1));
for i = 1:5:minLen  % every 5th for clarity (reduce to 1:1:minLen for full density)
    plot3([chosen_actual_path(i,1), chosen_reference_path(i,1)], ...
          [chosen_actual_path(i,2), chosen_reference_path(i,2)], ...
          [chosen_actual_path(i,3), chosen_reference_path(i,3)], ...
          'g-', 'LineWidth', 0.7);
end

%% Hausdorff distance %%
D = pdist2(chosen_reference_path, chosen_actual_path);

% Directed Hausdorff distances
hAB = max(min(D,[],2));
hBA = max(min(D,[],1));

% Symmetric Hausdorff distance
hausdorffDist = max(hAB, hBA);

fprintf('Hausdorff distance = %.3f m\n', hausdorffDist);
%% --- Compute and Visualize Hausdorff Distance Between Two Paths ---
% pathA = Nx3 matrix (e.g. actual path)
% pathB = Mx3 matrix (e.g. planned/smoothed path)

% Example input (replace with your actual data)
pathA = chosen_actual_path;
pathB = chosen_reference_path;

figure;
hold on; grid on; axis equal;
view(3);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Hausdorff Distance');
plot3(pathA(:,1), pathA(:,2), pathA(:,3), 'r-o', 'LineWidth', 1, 'DisplayName', 'Path A (Actual)');
plot3(pathB(:,1), pathB(:,2), pathB(:,3), 'b-o', 'LineWidth', 1, 'DisplayName', 'Path B (Planned)');

%% Step 1: Compute Directed Distances A→B and B→A

% --- A to B ---
D_AtoB = pdist2(pathA, pathB);               % Pairwise distance matrix
[minAtoB, idxB] = min(D_AtoB, [], 2);        % Closest point in B for each point in A
hAB = max(minAtoB);                          % Directed Hausdorff distance A→B
[~, idxHA] = max(minAtoB);                   % Index of point in A defining this distance
pairAB = [pathA(idxHA,:); pathB(idxB(idxHA),:)];

% --- B to A ---
D_BtoA = pdist2(pathB, pathA);               % Pairwise distance matrix
[minBtoA, idxA] = min(D_BtoA, [], 2);        % Closest point in A for each point in B
hBA = max(minBtoA);                          % Directed Hausdorff distance B→A
[~, idxHB] = max(minBtoA);                   % Index of point in B defining this distance
pairBA = [pathB(idxHB,:); pathA(idxA(idxHB),:)];

%% Step 2: True Hausdorff Distance (max of both)
H = max(hAB, hBA);

if hAB > hBA
    hausdorff_pair = pairAB;
    direction = 'A→B';
else
    hausdorff_pair = pairBA;
    direction = 'B→A';
end

%% Step 3: Plot all pair connections (optional but nice visual)
for i = 1:size(pathA,1)
    line([pathA(i,1), pathB(idxB(i),1)], ...
         [pathA(i,2), pathB(idxB(i),2)], ...
         [pathA(i,3), pathB(idxB(i),3)], ...
         'Color', 'green', 'LineStyle', '-');
end

%% Step 4: Highlight the Hausdorff pair
plot3(hausdorff_pair(:,1), hausdorff_pair(:,2), hausdorff_pair(:,3), ...
      'y-', 'LineWidth', 2.5, 'DisplayName', 'Hausdorff Pair');
scatter3(hausdorff_pair(1,1), hausdorff_pair(1,2), hausdorff_pair(1,3), ...
         80, 'filled', 'c', 'DisplayName', 'Point on Path A');
scatter3(hausdorff_pair(2,1), hausdorff_pair(2,2), hausdorff_pair(2,3), ...
         80, 'filled', 'm', 'DisplayName', 'Point on Path B');

%% Fréchet distance %%
function [dist, pairs] = frechetDistancePairs(P, Q)
% frechetDistancePairs  Compute discrete Fréchet distance between two 3D paths
% and return the matched point pairs.
%
%   [dist, pairs] = frechetDistancePairs(P, Q)
%
% Inputs:
%   P - Mx3 array of 3D points (reference path)
%   Q - Nx3 array of 3D points (actual path)
%
% Outputs:
%   dist  - scalar, discrete Fréchet distance
%   pairs - Kx2 indices into P and Q showing the correspondence path

    M = size(P,1);
    N = size(Q,1);
    
    % DP cache
    ca = -ones(M,N);
    
    % Recursive helper
    function d = c(i,j)
        if ca(i,j) > -1
            d = ca(i,j);
        elseif i == 1 && j == 1
            d = norm(P(1,:) - Q(1,:));
        elseif i > 1 && j == 1
            d = max(c(i-1,1), norm(P(i,:) - Q(1,:)));
        elseif i == 1 && j > 1
            d = max(c(1,j-1), norm(P(1,:) - Q(j,:)));
        else
            d = max(min([c(i-1,j), c(i-1,j-1), c(i,j-1)]), ...
                    norm(P(i,:) - Q(j,:)));
        end
        ca(i,j) = d;
    end
    
    % Final distance
    dist = c(M,N);
    
    % === Backtrack to get the matched pairs ===
    i = M; j = N;
    pairs = [i, j];
    
    while i > 1 || j > 1
        if i > 1 && j > 1
            [~, idx] = min([ca(i-1,j), ca(i-1,j-1), ca(i,j-1)]);
            if idx == 1
                i = i-1;
            elseif idx == 2
                i = i-1; j = j-1;
            else
                j = j-1;
            end
        elseif i > 1
            i = i-1;
        else
            j = j-1;
        end
        pairs = [i, j; pairs];
    end
end

% Reference (planned path)
P = chosen_reference_path;
% Actual (simulated UAV path)
Q = chosen_actual_path;

[dist, pairs] = frechetDistancePairs(P, Q);

fprintf('Discrete Fréchet Distance = %.3f m\n', dist);

%% Plot Fréchet %%
% === Plot results ===
figure; hold on; grid on; axis equal;
plot3(P(:,1), P(:,2), P(:,3), 'b-o', 'DisplayName','Reference Path');
plot3(Q(:,1), Q(:,2), Q(:,3), 'r-o', 'DisplayName','Actual Path');

% Draw matched pairs (leash)
for k = 1:size(pairs,1)
    i = pairs(k,1);
    j = pairs(k,2);
    plot3([P(i,1) Q(j,1)], [P(i,2) Q(j,2)], [P(i,3) Q(j,3)], 'g-');
end

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('Discrete Fréchet Distance = %.2f m', dist));
view(3);

%% === Nearest Neighbor Deviation Function ===
function [rmse, maxDev, dists, closestIdx, actualPairs, refPairs, maxIdx] = nearestDeviationWithPairs(actual, ref)
% nearestDeviationWithPairs Compute nearest-point deviations and return paired coordinates
%
% Inputs:
%   actual - Nx3 actual trajectory
%   ref    - Mx3 reference path
%
% Outputs:
%   rmse        - Root Mean Square Error
%   maxDev      - Maximum deviation
%   dists       - Nx1 vector of per-point deviations
%   closestIdx  - Nx1 indices of matched points on the reference
%   actualPairs - Nx3 array of actual points
%   refPairs    - Nx3 array of matched reference points
%   maxIdx      - index of maximum deviation point

    N = size(actual,1);
    dists = zeros(N,1);
    closestIdx = zeros(N,1);
    actualPairs = actual;       % copy actual points
    refPairs = zeros(N,3);      % allocate matched ref points
    
    for i = 1:N
        % Compute Euclidean distances from actual point to all reference points
        [dists(i), closestIdx(i)] = min(vecnorm(ref - actual(i,:), 2, 2));
        refPairs(i,:) = ref(closestIdx(i),:);
    end
    
    % Compute metrics
    rmse = sqrt(mean(dists.^2));
    [maxDev, maxIdx] = max(dists);
end


%% === Run Function ===
[rmse_RRT, maxDev_RRT, dists_RRT, closestIdx_RRT, actualPairs_RRT, refPairs_RRT, maxIdx_RRT] = ...
    nearestDeviationWithPairs(actualPath_RRT, refInterp_RRT);
[rmse_BiRRT, maxDev_BiRRT, dists_BiRRT, closestIdx_BiRRT, actualPairs_BiRRT, refPairs_BiRRT, maxIdx_BiRRT] = ...
    nearestDeviationWithPairs(actualPath_BiRRT, refInterp_BiRRT);
[rmse_RRTstar, maxDev_RRTstar, dists_RRTstar, closestIdx_RRTstar, actualPairs_RRTstar, refPairs_RRTstar, maxIdx_RRTstar] = ...
    nearestDeviationWithPairs(actualPath_RRTstar, refInterp_RRTstar);
[rmse_PRM, maxDev_PRM, dists_PRM, closestIdx_PRM, actualPairs_PRM, refPairs_PRM, maxIdx_PRM] = ...
    nearestDeviationWithPairs(actualPath_PRM, refInterp_PRM);
%% === Plot Nearest Neighbor Deviations ===
figure;
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
title(sprintf('Nearest-Neighbor Deviations RRT (RMSE = %.2f m, MaxDev = %.2f m)', rmse_RRT, maxDev_RRT));

% Plot actual and reference paths
plot3(actualPath_RRT(:,1), actualPath_RRT(:,2), actualPath_RRT(:,3), ...
      'r-', 'LineWidth', 1.4, 'DisplayName','Actual Path');
plot3(refInterp_RRT(:,1), refInterp_RRT(:,2), refInterp_RRT(:,3), ...
      'b-', 'LineWidth', 1.4, 'DisplayName','Reference Path');

% Plot all deviation pairs (use fewer lines to keep plot readable)
for i = 1:5:size(actualPairs_RRT,1) % adjust step for density
    plot3([actualPairs_RRT(i,1), refPairs_RRT(i,1)], ...
          [actualPairs_RRT(i,2), refPairs_RRT(i,2)], ...
          [actualPairs_RRT(i,3), refPairs_RRT(i,3)], ...
          'g--', 'LineWidth', 0.7);
end

% Highlight maximum deviation pair
maxActual_RRT = actualPairs_RRT(maxIdx_RRT,:);
maxRef_RRT = refPairs_RRT(maxIdx_RRT,:);

% Points
plot3(maxActual_RRT(1), maxActual_RRT(2), maxActual_RRT(3), 'o', ...
      'MarkerSize', 8, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Actual (Max Dev)');
plot3(maxRef_RRT(1), maxRef_RRT(2), maxRef_RRT(3), 'o', ...
      'MarkerSize', 8, 'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Reference (Max Dev)');

% Connecting line (highlight)
plot3([maxActual_RRT(1), maxRef_RRT(1)], ...
      [maxActual_RRT(2), maxRef_RRT(2)], ...
      [maxActual_RRT(3), maxRef_RRT(3)], ...
      'y-', 'LineWidth', 2.5, 'DisplayName','Max Deviation Line');

fprintf('RRT Maximum deviation = %.3f m\n', maxDev_RRT);
%=====================================================================%
figure;
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
title(sprintf('Nearest-Neighbor Deviations BiRRT (RMSE = %.2f m, MaxDev = %.2f m)', rmse_BiRRT, maxDev_BiRRT));

% Plot actual and reference paths
plot3(actualPath_BiRRT(:,1), actualPath_BiRRT(:,2), actualPath_BiRRT(:,3), ...
      'r-', 'LineWidth', 1.4, 'DisplayName','Actual Path');
plot3(refInterp_BiRRT(:,1), refInterp_BiRRT(:,2), refInterp_BiRRT(:,3), ...
      'b-', 'LineWidth', 1.4, 'DisplayName','Reference Path');

% Plot all deviation pairs (use fewer lines to keep plot readable)
for i = 1:5:size(actualPairs_BiRRT,1) % adjust step for density
    plot3([actualPairs_BiRRT(i,1), refPairs_BiRRT(i,1)], ...
          [actualPairs_BiRRT(i,2), refPairs_BiRRT(i,2)], ...
          [actualPairs_BiRRT(i,3), refPairs_BiRRT(i,3)], ...
          'g--', 'LineWidth', 0.7);
end

% Highlight maximum deviation pair
maxActual_BiRRT = actualPairs_BiRRT(maxIdx_BiRRT,:);
maxRef_BiRRT = refPairs_BiRRT(maxIdx_BiRRT,:);

% Points
plot3(maxActual_BiRRT(1), maxActual_BiRRT(2), maxActual_BiRRT(3), 'o', ...
      'MarkerSize', 8, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Actual (Max Dev)');
plot3(maxRef_BiRRT(1), maxRef_BiRRT(2), maxRef_BiRRT(3), 'o', ...
      'MarkerSize', 8, 'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Reference (Max Dev)');

% Connecting line (highlight)
plot3([maxActual_BiRRT(1), maxRef_BiRRT(1)], ...
      [maxActual_BiRRT(2), maxRef_BiRRT(2)], ...
      [maxActual_BiRRT(3), maxRef_BiRRT(3)], ...
      'y-', 'LineWidth', 2.5, 'DisplayName','Max Deviation Line');

fprintf('BiRRT Maximum deviation = %.3f m\n', maxDev_BiRRT);
%=====================================================================%
figure;
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
title(sprintf('Nearest-Neighbor Deviations RRT* (RMSE = %.2f m, MaxDev = %.2f m)', rmse_RRTstar, maxDev_RRTstar));

% Plot actual and reference paths
plot3(actualPath_RRTstar(:,1), actualPath_RRTstar(:,2), actualPath_RRTstar(:,3), ...
      'r-', 'LineWidth', 1.4, 'DisplayName','Actual Path');
plot3(refInterp_RRTstar(:,1), refInterp_RRTstar(:,2), refInterp_RRTstar(:,3), ...
      'b-', 'LineWidth', 1.4, 'DisplayName','Reference Path');

% Plot all deviation pairs (use fewer lines to keep plot readable)
for i = 1:5:size(actualPairs_RRTstar,1) % adjust step for density
    plot3([actualPairs_RRTstar(i,1), refPairs_RRTstar(i,1)], ...
          [actualPairs_RRTstar(i,2), refPairs_RRTstar(i,2)], ...
          [actualPairs_RRTstar(i,3), refPairs_RRTstar(i,3)], ...
          'g--', 'LineWidth', 0.7);
end

% Highlight maximum deviation pair
maxActual_RRTstar = actualPairs_RRTstar(maxIdx_RRTstar,:);
maxRef_RRTstar = refPairs_RRTstar(maxIdx_RRTstar,:);

% Points
plot3(maxActual_RRTstar(1), maxActual_RRTstar(2), maxActual_RRTstar(3), 'o', ...
      'MarkerSize', 8, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Actual (Max Dev)');
plot3(maxRef_RRTstar(1), maxRef_RRTstar(2), maxRef_RRTstar(3), 'o', ...
      'MarkerSize', 8, 'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Reference (Max Dev)');

% Connecting line (highlight)
plot3([maxActual_RRTstar(1), maxRef_RRTstar(1)], ...
      [maxActual_RRTstar(2), maxRef_RRTstar(2)], ...
      [maxActual_RRTstar(3), maxRef_RRTstar(3)], ...
      'y-', 'LineWidth', 2.5, 'DisplayName','Max Deviation Line');

fprintf('RRTstar Maximum deviation = %.3f m\n', maxDev_RRTstar);
%=====================================================================%
figure;
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
title(sprintf('Nearest-Neighbor Deviations PRM (RMSE = %.2f m, MaxDev = %.2f m)', rmse_PRM, maxDev_PRM));

% Plot actual and reference paths
plot3(actualPath_PRM(:,1), actualPath_PRM(:,2), actualPath_PRM(:,3), ...
      'r-', 'LineWidth', 1.4, 'DisplayName','Actual Path');
plot3(refInterp_PRM(:,1), refInterp_PRM(:,2), refInterp_PRM(:,3), ...
      'b-', 'LineWidth', 1.4, 'DisplayName','Reference Path');

% Plot all deviation pairs (use fewer lines to keep plot readable)
for i = 1:5:size(actualPairs_PRM,1) % adjust step for density
    plot3([actualPairs_PRM(i,1), refPairs_PRM(i,1)], ...
          [actualPairs_PRM(i,2), refPairs_PRM(i,2)], ...
          [actualPairs_PRM(i,3), refPairs_PRM(i,3)], ...
          'g--', 'LineWidth', 0.7);
end

% Highlight maximum deviation pair
maxActual_PRM = actualPairs_PRM(maxIdx_PRM,:);
maxRef_PRM = refPairs_PRM(maxIdx_PRM,:);

% Points
plot3(maxActual_PRM(1), maxActual_PRM(2), maxActual_PRM(3), 'o', ...
      'MarkerSize', 8, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Actual (Max Dev)');
plot3(maxRef_PRM(1), maxRef_PRM(2), maxRef_PRM(3), 'o', ...
      'MarkerSize', 8, 'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Reference (Max Dev)');

% Connecting line (highlight)
plot3([maxActual_PRM(1), maxRef_PRM(1)], ...
      [maxActual_PRM(2), maxRef_PRM(2)], ...
      [maxActual_PRM(3), maxRef_PRM(3)], ...
      'y-', 'LineWidth', 2.5, 'DisplayName','Max Deviation Line');

fprintf('PRM Maximum deviation = %.3f m\n', maxDev_PRM);
%% New error calculation %%
% === Calculate deviations for each planner ===
[rmseRRT, maxDevRRT, distsRRT] = nearestDeviationWithPairs(actualPath_RRT, refInterp_RRT);
mseRRT = mean(distsRRT.^2);

[rmseBiRRT, maxDevBiRRT, distsBiRRT] = nearestDeviationWithPairs(actualPath_BiRRT, refInterp_BiRRT);
mseBiRRT = mean(distsBiRRT.^2);

[rmseRRTstar, maxDevRRTstar, distsRRTstar] = nearestDeviationWithPairs(actualPath_RRTstar, refInterp_RRTstar);
mseRRTstar = mean(distsRRTstar.^2);

[rmsePRM, maxDevPRM, distsPRM] = nearestDeviationWithPairs(actualPath_PRM, refInterp_PRM);
msePRM = mean(distsPRM.^2);

% === Collect results into a table ===
Algorithm = {'RRT'; 'BiRRT'; 'RRT*'; 'PRM'};
MSE = [mseRRT; mseBiRRT; mseRRTstar; msePRM];
RMSE = [rmseRRT; rmseBiRRT; rmseRRTstar; rmsePRM];
MaxDeviation = [maxDevRRT; maxDevBiRRT; maxDevRRTstar; maxDevPRM];

resultsTable = table(Algorithm, MSE, RMSE, MaxDeviation);

% === Display ===
disp('Path Tracking Error Metrics:');
disp(resultsTable);
