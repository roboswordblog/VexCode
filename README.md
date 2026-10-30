THIS IS FOR VEX IQ ONLY 

The goal of this is to use a linear slingshot(previosly a flywheel) + claw(previously roller intake), to take the beanbag, aim the catapult, and then shoot at the target. 

There are 5 Different Main Classes to the Code:
1. Controller
    a. This gets the input from the controller and moves the robot. We are using this instead of the normal drivebase controls because we want to customise the movement
    b. This also lets us choose when we're shooting the beanbag

2. Drivebase
    a. This is the class that has the 2 drivebase motors.
    b. This class is mostly for organization and easiness to use.
    c. It will have odometry to calculate its position from motor encoders. 

3. Claw
    a. This is the claw, which uses 2 pnematic cylenders to pick up the object, and it uses one motor to turn the entire claw into the linear slingshot
    b. This is a class for organization and so that it can be passed into the controller object
4. Linear Slingshot
    a. This class has 2 motors(the pull back string motor, and the ratchet motor to release the slingshot and fire it)
    b. This class will be passed to both the controller and the autoaim

4. Robot
    a. This class instantiates the other 4 classes, and runs them all. 
    b. It also passes the Drivebase, Claw, and Linear slingshot class into the Controller class, and updates all of them. 