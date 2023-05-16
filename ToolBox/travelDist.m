function travelDist(Robot, speed, distance)
% get the current odometry
latestOdom = receive(Robot.subOdom,3);
lastPoseX = latestOdom.pose.pose.position.x;
lastPoseY = latestOdom.pose.pose.position.y;
lastOrien = latestOdom.pose.pose.orientation;
[startTheta, ~, ~] = quaternion2EulerBodyZYX(lastOrien.w,lastOrien.x,lastOrien.y,lastOrien.z);
% go forward until it made it or timeout
timeOut = 200;
startTime = toc;
remainDist = distance;
while (toc-startTime)<timeOut
    % latest orientation
    latestOdom = receive(Robot.subOdom,3);
    currPoseX = latestOdom.pose.pose.position.x;
    currPoseY = latestOdom.pose.pose.position.y;
    % Robot orientation vector
    headVector = [cos(startTheta) sin(startTheta)];
    % distance traveled
    direction = dot(headVector,[currPoseX-lastPoseX currPoseY-lastPoseY]);
    traveledDist = sign(direction)*norm([currPoseX-lastPoseX currPoseY-lastPoseY]);
    remainDist = remainDist-traveledDist;
    % for debug
    msgStr = "Remain distance: " + string(remainDist);
    disp(msgStr);
    % check if it get there
    if remainDist*distance < 0
        % Stop the robot
        twist = ros2message("geometry_msgs/Twist");
        twist.linear.x = 0;
        twist.linear.y = 0;
        send(Robot.pubLAVL, twist);
        % job done
        msgStr = "Done traveling distance: " + string(distance);
        disp(msgStr);
        return;
    end
    % if not, keep turning
    twist = ros2message("geometry_msgs/Twist");
    twist.linear.x = speed*sign(distance);
    send(Robot.pubLAVL, twist);
    % update the last data
    lastPoseX = currPoseX;
    lastPoseY = currPoseY;
end
disp("Time up. Traveling Failed.");
% Stop the robot
twist = ros2message("geometry_msgs/Twist");
twist.linear.x = 0;
twist.linear.y = 0;
send(Robot.pubLAVL, twist);
end