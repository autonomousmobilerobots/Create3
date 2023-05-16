function [yaw, pitch, roll] = quaternion2EulerBodyZYX(w,x,y,z)
% reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
roll  = atan2(2*(w*x+y*z),1-2*(x*x+y*y));
pitch = -pi/2+2*atan2(sqrt(1+2*(w*y-x*z)),sqrt(1-2*(w*y-x*z)));
yaw   = atan2(2*(w*z+x*y),1-2*(y*y+z*z));
end
