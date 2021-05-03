# grbl-polar

***
Improved version of the polar grbl from [ilaro-org](https://github.com/ilaro-org/grbl-polar) in order to smooth the movement of the servo motor.
Main changes are in the file spindle_control.c. Smoothness is improved from the original version by:
  * increasing the number of steps that divide 0° to 180°
  * moving the servo by stepping through all the possible steps
  * inserting a little pause (controlled by $29 parameter) before moving to the next step
The need of smoothing the servo comes from the fact that I am using a polar machine to control a pen on a sheet rather than a spray.

**This improved version of the polar grbl is intended to work and has been tested only on polar machines controlled by Arduino Uno (ATmega 328p)**

<br/>
<br/>

Graffiti robot firmware base on [Grbl v0.9](https://github.com/grbl/grbl)

The implemented kinematics allow a 2 string + gravity system (as in [hektor](http://juerglehni.com/works/hektor)), and the pwm support allows triggering the spray using a servo-motor.

additional features:
  * define POLAR: swaps from cartesian to polar kinematics. It's required to set up the distance between the motors. Homing at startup is essential, otherwise positioning can not be achieved.
  * define RC_SERVO: Use PIN D11 to drive the servo. Use the commands M03 Sxxx (xxx between RC_SERVO_MIN and RC_SERVO_MAX) to rotate the servo between 0-180. The command M05 turns the servo to zero degrees. [source](https://github.com/robottini/grbl-servo)
  * $29 parameter controls the delay (milliseconds) to be waited before servo steps to the next step when moving

<br/>
  


##Configuring Grbl-polar
The Grbl-polar's configuration is the same as in [Grbl v0.9] (https://github.com/ilaro-org/grbl-polar/wiki/Configuring-Grbl-v0.9). But we have added a new setting: 'distance'. You can define it through the GUI settings or by the command line:   

     $28=1000 (distance, mm)

It defines the distance between the two motors and it is needed in order to achieve machine's positioning.

![alt text](https://github.com/ilaro-org/grbl-polar/blob/master/grbl-polar.JPG)

##Gcode

To generate the G-code you can use any slicing program and slice a vectorial drawing. We used Inkscape because it is opensource and you can do vectorial drawings and slice them directly with the [Laser Plug-In](https://jtechphotonics.com/?page_id=2012). You have to take into account when sittuating the drawing in the page that the center (0,0) of the robot is situated on the left motor.

To send G-code to the robot we had used http://chilipeppr.com/grbl or [Universal G-code Sender](https://github.com/winder/Universal-G-Code-Sender)



GPLv3 license
