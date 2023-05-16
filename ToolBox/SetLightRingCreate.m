function SetLightRingCreate(Robot, rgb)
% rgb: [6x3] [r1 g1 b1; r2 g2 b2; ...]
% if rgb is empty, the light ring goes to the default
msslr = ros2message("irobot_create_msgs/LightringLeds");
if ~isempty(rgb)
    msslr.override_system = true;
    msslr.leds = [];
    for i=1:6
        col.red = uint8(rgb(i,1));
        col.green = uint8(rgb(i,2));
        col.blue = uint8(rgb(i,3));
        msslr.leds = [msslr.leds col];
    end
end
send(Robot.pubLRng, msslr);
end
