# Robot-trial

 This is a trial code for my Robotics project<br />

 I first planned to set the robot in initial position<br />

 Then run Image recognition to Identify the April Tags<br />

Then get the centroid of those Tags individually<br />

Adjust the point in image coordinate to camera coordinate using calibrated camera parameters in a <br />
 Then calculate and convert the points in camera coordinate to robot's base configuration <br />
 I would call a function which already has my robot's congifiguration and joint limits 
In the form of a Rigid Body 7<br />

And solve a basic invKin problem in robots base configuartion <br />

Use potentiometer values to get feedback on whether a joint angle is reached <br />
 Include all this for a single pick and place motion in Robot base coordinate<br />

