function floatTime = ros2StampToFloat(stamp, floatEpoch)
floatNow = double(stamp.sec)+(10^-9)*double(stamp.nanosec);
floatTime = floatNow-floatEpoch;
end