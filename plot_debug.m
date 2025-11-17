%RRT%
unsmoothed_RRT = [pth1.States(:,1), pth1.States(:,2), pth1.States(:,3)];
x_RRT = squeeze(RRTData.posRRT(:,1)); 
y_RRT = squeeze(RRTData.posRRT(:,2)); 
z_RRT = squeeze(RRTData.posRRT(:,3));

%BiRRT%
unsmoothed_BiRRT = [pth2.States(:,1), pth2.States(:,2), pth2.States(:,3)];
x_BiRRT = squeeze(BiRRTData.posBiRRT(:,1)); 
y_BiRRT = squeeze(BiRRTData.posBiRRT(:,2)); 
z_BiRRT = squeeze(BiRRTData.posBiRRT(:,3));

%RRT*%
unsmoothed_RRTstar = [pth3.States(:,1), pth3.States(:,2), pth3.States(:,3)];
x_RRTstar = squeeze(RRTstarData.posRRTstar(:,1)); 
y_RRTstar = squeeze(RRTstarData.posRRTstar(:,2)); 
z_RRTstar = squeeze(RRTstarData.posRRTstar(:,3));

%PRM%
unsmoothed_PRM = [pth4.States(:,1), pth4.States(:,2), pth4.States(:,3)];
x_PRM = squeeze(PRMData.posPRM(:,1)); 
y_PRM = squeeze(PRMData.posPRM(:,2)); 
z_PRM = squeeze(PRMData.posPRM(:,3));

figure;
plot3(x_RRT, y_RRT, z_RRT, 'r-', 'LineWidth', 2); hold on;
%plot3(unsmoothed_RRT(:,1), unsmoothed_RRT(:,2), unsmoothed_RRT(:,3), 'r:', 'LineWidth', 1); hold on;
plot3(waypoints_RRT_smoothed(:,1), waypoints_RRT_smoothed(:,2), waypoints_RRT_smoothed(:,3), 'r--', 'LineWidth', 1); hold on;
plot3(x_RRT(1), y_RRT(1), z_RRT(1), 'r*'); hold on;
plot3(x_RRT(end), y_RRT(end), z_RRT(end), 'rx'); hold on;

plot3(x_BiRRT, y_BiRRT, z_BiRRT, 'g-', 'LineWidth', 2); hold on;
%plot3(unsmoothed_BiRRT(:,1), unsmoothed_BiRRT(:,2), unsmoothed_BiRRT(:,3), 'g:', 'LineWidth', 1); hold on;
plot3(waypoints_BiRRT_smoothed(:,1), waypoints_BiRRT_smoothed(:,2), waypoints_BiRRT_smoothed(:,3), 'g--', 'LineWidth', 1); hold on;
plot3(x_BiRRT(1), y_BiRRT(1), z_BiRRT(1), 'g*'); hold on;
plot3(x_BiRRT(end), y_BiRRT(end), z_BiRRT(end), 'gx'); hold on;

plot3(x_RRTstar, y_RRTstar, z_RRTstar, 'b-', 'LineWidth', 2); hold on;
%plot3(unsmoothed_RRTstar(:,1), unsmoothed_RRTstar(:,2), unsmoothed_RRTstar(:,3), 'b:', 'LineWidth', 1); hold on;
plot3(waypoints_RRTstar_smoothed(:,1), waypoints_RRTstar_smoothed(:,2), waypoints_RRTstar_smoothed(:,3), 'b--', 'LineWidth', 1); hold on;
plot3(x_RRTstar(1), y_RRTstar(1), z_RRTstar(1), 'b*'); hold on;
plot3(x_RRTstar(end), y_RRTstar(end), z_RRTstar(end), 'bx'); hold on;

plot3(x_PRM, y_PRM, z_PRM, 'y-', 'LineWidth', 2); hold on;
%plot3(unsmoothed_PRM(:,1), unsmoothed_PRM(:,2), unsmoothed_PRM(:,3), 'y:', 'LineWidth', 1); hold on;
plot3(waypoints_PRM_smoothed(:,1), waypoints_PRM_smoothed(:,2), waypoints_PRM_smoothed(:,3), 'y--', 'LineWidth', 1); hold on;
plot3(x_PRM(1), y_PRM(1), z_PRM(1), 'y*'); hold on;
plot3(x_PRM(end), y_PRM(end), z_PRM(end), 'yx'); hold on;

scatter3(goalPose(1,1),goalPose(1,2),goalPose(1,3),'magenta','o','filled','LineWidth',8); hold on;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('UAV path tracking');
grid on;
axis equal;
view(3);  % 3D perspective
