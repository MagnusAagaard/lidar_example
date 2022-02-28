%% Lidar data example
% Install 'Navigation Toolbox' and 'ROS Toolbox' before running the example
clear;clc;close all;

% Load bag
bag = rosbag('lidar_data_long.bag');
% Get info about bag (topics etc., equivalent of running rosbag info 'bag_name')
bagInfo = rosbag('info','lidar_data_long.bag');

% Select a topic (map)
mapTopic = select(bag,'Topic','/map');
% Parse messages to a matlab struct
msgStructsMap = readMessages(mapTopic,'DataFormat','struct');
% Create map from message
map = rosReadOccupancyGrid(msgStructsMap{mapTopic.NumMessages});

% Get Lidar data and plot
lidarTopic = select(bag,'Topic','/scan');
msgStructsLidar = readMessages(lidarTopic,'DataFormat','struct');
lidarMsg = msgStructsLidar{1};
%rosPlot(lidarMsg)

% To play with Lidar data different parameters can be obtained from the
% lidarMsg
minAngle = lidarMsg.AngleMin;
maxAngle = lidarMsg.AngleMax;
incAngle = lidarMsg.AngleIncrement;
minRange = lidarMsg.RangeMin;
maxRange = lidarMsg.RangeMax;
% And actual data is in:
data = lidarMsg.Ranges;
figure(1);
hold on;
for a = minAngle:incAngle:maxAngle
    xmin = minRange * cos(a);
    ymin = minRange * sin(a);
    xmax = maxRange * cos(a);
    ymax = maxRange * sin(a);
    plot(xmin,ymin,'g.');
    plot(xmax,ymax,'b.');
end
title('Visualization of min. and max. range circle of Lidar')
xlabel('X [m]')
ylabel('Y [m]')
legend({'Green: Minimum range circle','Blue Maximum range circle'})
%%

% Visualization of Lidar data over time, transformed to map frame
close all;
% Get odom (robot location)
odomTopic = select(bag,'Topic','/odom');
msgStructsOdom = readMessages(odomTopic,'DataFormat','struct');
odomMsg = msgStructsOdom{1};

% Transform Lidar data into map frame (coordinate system)
tf = getTransform(bag,'map','base_scan');
% Transform rotation
lidarMsgTransformed = transformLidar(tf, lidarMsg);
% Transform translation
cart = rosReadCartesian(lidarMsgTransformed);
cart(:,1) = cart(:,1) + tf.Transform.Translation.X;
cart(:,2) = cart(:,2) + tf.Transform.Translation.Y;
% Setup plot result
figure(2);
show(map)   % Uncomment this line to not overlay the data on top of the map
hold on;    
%xlim([-5 5]); ylim([-5 5]);

tStart = bag.StartTime;
tEnd = bag.EndTime;
idxLidar = 1;
idxOdom = 1;
% Visualize Lidar data over time
for t = tStart:0.01:tEnd
    % If new Lidar data available, increment index
    if t >= lidarTopic.MessageList.Time(idxLidar)
        if idxLidar < lidarTopic.NumMessages
            idxLidar = idxLidar + 1;
        end
    end
    if t >= odomTopic.MessageList.Time(idxOdom)
        if idxOdom < odomTopic.NumMessages
            idxOdom = idxOdom + 1;
        end
    end
    % Get robot pose
    robotPoseMsg = msgStructsOdom{idxOdom}.Pose.Pose.Position;
    plt1 = plot(robotPoseMsg.X, robotPoseMsg.Y, 'c*',MarkerSize=10);
    % Get Lidar data
    lidarMsg = msgStructsLidar{idxLidar};
    % Check if we can transform from base_scan frame to map frame
    if canTransform(bag,'map','base_scan',rostime(t))
        tf = getTransform(bag,'map','base_scan',rostime(t));
        % Transform rotation
        lidarMsgTransformed = transformLidar(tf, lidarMsg);
        % Transform translation
        cart = rosReadCartesian(lidarMsgTransformed);
        cart(:,1) = cart(:,1) + tf.Transform.Translation.X;
        cart(:,2) = cart(:,2) + tf.Transform.Translation.Y;
    end
    % Plot result
    plt2 = plot(cart(:,1), cart(:,2),'r.');
    pause(0.003);
    delete(plt1);
    delete(plt2);
end

function lidarMsgTransformed = transformLidar(tf, lidarMsg)
q = [tf.Transform.Rotation.W, tf.Transform.Rotation.X, tf.Transform.Rotation.Y, tf.Transform.Rotation.Z];
ang = round(rad2deg(atan2(q(4),q(1))*2));
if ang > 180
    ang = -(360-ang);
elseif ang < -180
    ang = 360 + ang;
end
if ang == 0
    ang = 1;
end
lidar_ranges = zeros(360,1,'single');
lidar_intensities = zeros(360,1,'single');
% Shift data corresponding to rotation angle
if ang >= 0
    lidar_ranges(ang+1:ang*2) = lidarMsg.Ranges(1:ang);
    lidar_ranges(ang*2:360) = lidarMsg.Ranges(ang:360-ang);
    lidar_ranges(1:ang) = lidarMsg.Ranges(360-ang+1:360);
    lidar_intensities(ang+1:ang*2) = lidarMsg.Intensities(1:ang);
    lidar_intensities(ang*2:360) = lidarMsg.Intensities(ang:360-ang);
    lidar_intensities(1:ang) = lidarMsg.Intensities(360-ang+1:360);
else
    ang = -ang;
    lidar_ranges(360-ang+1:360) = lidarMsg.Ranges(1:ang);
    lidar_ranges(1:360-ang) = lidarMsg.Ranges(ang+1:360);
    lidar_intensities(360-ang+1:360) = lidarMsg.Intensities(1:ang);
    lidar_intensities(1:360-ang) = lidarMsg.Intensities(ang+1:360);
end
lidarMsgTransformed = lidarMsg;
lidarMsgTransformed.Ranges = lidar_ranges;
lidarMsgTransformed.Intensities = lidar_intensities;

end
