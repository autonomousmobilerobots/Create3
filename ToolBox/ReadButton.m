function buttonState = ReadButton(Robot)
% buttonState: [3x1] [button1;buttonPower;button2]
latestBttn = receive(Robot.subBttn,3);
bttn1 = latestBttn.button_1.is_pressed;
bttnp = latestBttn.button_power.is_pressed;
bttn2 = latestBttn.button_2.is_pressed;
buttonState = [bttn1;bttnp;bttn2];
end