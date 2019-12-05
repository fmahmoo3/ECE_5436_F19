Maze Solving Robot
This robot will solve a maze navigating it using the right-hand rule. The user will need to connect to the robot using Bluetooth. To do this it is recommended the user install PuTTY or some other Bluetooth connecting software. 

Commands
    Once the user connects to the robot using PuTTY they can use a series of 2 character input commands to control the robot.
      • tr -- toggle red led
      • tg -- toggle green led
      • tb -- toggle blue led
      • df – read value from front sensor
      • dr – read value from right sensor
      •	go – start maze navigation
      •	re -- runs both motors in robot’s backwards direction
      •	st -- turn motors off
      •	hs -- increase duty cycle to 100%, no specific direction
      •	rr -- rotates robot towards right, no specific speed
      •	rl -- rotates robot towards left, no specific speed
    These commands are for altering the PID values the robot uses to follow walls.
      •	ip -- increases kp by .2
      •	dp -- decreases kp by .2
      •	ii -- increases ki by .2
      •	di -- decreases ki by .2
      •	id -- increases kd by .2
      •	dd -- decreases kd by .2
      
Functionality
When the user enters ‘go’ the robot will begin navigating the maze following the wall using the PID values (kp = 1.4, ki = 0.8, kd = 1.4 by default). 
To ensure the robot is working correctly the user will see a heartbeat light on the robot that will toggle every half a second.
If the robot runs over a single black line (1”) the robot will begin transmitting the sensor data to the command prompt.
The maze end should be denoted by a double black line (2”) when that is detected the robot will turn stop, output the time it was running, print the PID values, and will await its next command.

