function turnAngle(Robot, speed, angle)
% get the current odometry Pose
latestOdom = receive(Robot.subOdom,3);
lastOrien = latestOdom.pose.pose.orientation;
[lastTheta, ~, ~] = quaternion2EulerBodyZYX(lastOrien.w,lastOrien.x,lastOrien.y,lastOrien.z);
% rotate until it made it or timeout
timeOut = 200;
startTime = toc;
remainAngle = angle;
while (toc-startTime)<timeOut
    % latest orientation
    latestOdom = receive(Robot.subOdom,3);
    currOrien = latestOdom.pose.pose.orientation;
    [currTheta, ~, ~] = quaternion2EulerBodyZYX(currOrien.w,currOrien.x,currOrien.y,currOrien.z);
    % turned
    turnedAngle = real(log(exp(1i*currTheta)/exp(1i*lastTheta))/(1i));
    remainAngle = remainAngle-turnedAngle;
    % for debug
    msgStr = "Current Orientation: " + string(currTheta)...
        + ", remained angle: " + string(remainAngle);
    disp(msgStr);
    % check if it get there
    if remainAngle*angle < 0
        % Stop the robot
        twist = ros2message("geometry_msgs/Twist");
        twist.angular.z = 0;
        send(Robot.pubLAVL, twist);
        % job done
        msgStr = "Done turning angle: " + string(angle);
        disp(msgStr);
        return;
    end
    % if not, keep turning
    twist = ros2message("geometry_msgs/Twist");
    twist.angular.z = speed*sign(angle);
    send(Robot.pubLAVL, twist);
    % update the last data
    lastTheta = currTheta;
end
disp("Time up. Turning Failed.");
% Stop the robot
twist = ros2message("geometry_msgs/Twist");
twist.angular.z = 0;
send(Robot.pubLAVL, twist);
end
