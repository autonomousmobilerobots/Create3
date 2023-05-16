function SetFwdVelAngVelCreate(Robot, FwdVel, AngVel)
twist = ros2message("geometry_msgs/Twist");
twist.linear.x = FwdVel;
twist.angular.z = AngVel;
send(Robot.pubLAVL, twist);
end

