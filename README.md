# Submission Readme #

This is the readme for my project submission. The original project README has be moved to PROJ_README for reference.

## The Tasks ##
Below I outline my 
* approach to the tasks
* a video / image demo 
* and finally the performance metrics.

### scenario 1: Intro (Hover) ###
Upon looking at the thrust as hinted in the outline one can see that the only parameter that can be changed is **mass** which is located in the _QuadControls.txt_. 

```
Mass = 0.49
```

```C++
cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right
```

<p align="center">
<img src="animations/scenario_1.gif" width="500"/>
</p>

**Evaluation:**
```
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #4 (../config/1_Intro.txt)
```

### scenario 2: Body rate and roll/pitch control ###
**1. Implement`GenerateMotorCommands()`**

 - _implement the code in the function `GenerateMotorCommands()`_<br>
 This was pretty tricky for me as it was different to the what we implemented in the coursework. However taking a logical approach to this problem was key. It came down to solving the force matrix (below) by first calculating the Roll, Pitch and Yaw commands from the moments.
 
 
 _Collective Thrust: F1 + F2 + F3 + F4 = Tf<br>
  Roll Command:      F1 + F2 + F3 + F4 = Tx/l<br>
 Pitch Command:     F1 + F2 + F3 + F4 = Ty/l<br>
 Yaw Command:       F1 + F2 + F3 + F4 = Tz/kappa_<br>
 
 
 It was important to pay attention to the rotor positions. The above is base on the following:
 <p align="center">
   <img src="animations/drone1_1.png" width="500">
 </p>
 
 Finally the *CONSTRAIN* command was utilised to keep motor thrust within the configuration limits.
 
 ```C++
    cmd.desiredThrustsN[0] = CONSTRAIN(0.25f * (collThrustCmd + fx + fy + fz), minMotorThrust, maxMotorThrust); // front left
    cmd.desiredThrustsN[1] = CONSTRAIN(0.25f * (collThrustCmd - fx + fy - fz), minMotorThrust, maxMotorThrust); // front right
    cmd.desiredThrustsN[2] = CONSTRAIN(0.25f * (collThrustCmd + fx - fy - fz), minMotorThrust, maxMotorThrust); // rear left
    cmd.desiredThrustsN[3] = CONSTRAIN(0.25f * (collThrustCmd - fx - fy + fz), minMotorThrust, maxMotorThrust); // rear right
    
 ```
 
 **2. Implement `BodyRateControl()`** 
 
 - Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot
 
 **3. Implement `RollPitchControl()`**
 
 <p align="center">
<img src="animations/scenario_2.gif" width="500"/>
</p>
 
**Evaluation:**
   - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
   - roll rate should less than 2.5 radian/sec for 0.75 seconds
```
Simulation #30 (../config/2_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
```

 ### scenario 3: Position/velocity and yaw angle control ###
 
 **Evaluation:**
   - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
   - Quad2 yaw should be within 0.1 of the target for at least 1 second

 ### scenario 4: Non-idealities and robustness ###
 
 **Evaluation:**
   - position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds

 ### scenario 5: Tracking trajectories ###
 
 **Evaluation:**
   - position error of the quad should be less than 0.25 meters for at least 3 seconds

### Body rate and roll/pitch control (scenario 2) ###


First, you will implement the body rate and roll / pitch control.  For the simulation, you will use `Scenario 2`.  In this scenario, you will see a quad above the origin.  It is created with a small initial rotation speed about its roll axis.  Your controller will need to stabilize the rotational motion and bring the vehicle back to level attitude.

To accomplish this, you will:

1. Implement body rate control

 - implement the code in the function `GenerateMotorCommands()`
 - implement the code in the function `BodyRateControl()`
 - Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot

If successful, you should see the rotation of the vehicle about roll (omega.x) get controlled to 0 while other rates remain zero.  Note that the vehicle will keep flying off quite quickly, since the angle is not yet being controlled back to 0.  Also note that some overshoot will happen due to motor dynamics!.

If you come back to this step after the next step, you can try tuning just the body rate omega (without the outside angle controller) by setting `QuadControlParams.kpBank = 0`.

2. Implement roll / pitch control
We won't be worrying about yaw just yet.

 - implement the code in the function `RollPitchControl()`
 - Tune `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot

If successful you should now see the quad level itself (as shown below), though it’ll still be flying away slowly since we’re not controlling velocity/position!  You should also see the vehicle angle (Roll) get controlled to 0.

<p align="center">
<img src="animations/scenario2.gif" width="500"/>
</p>


### Position/velocity and yaw angle control (scenario 3) ###

Next, you will implement the position, altitude and yaw control for your quad.  For the simulation, you will use `Scenario 3`.  This will create 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

 - implement the code in the function `LateralPositionControl()`
 - implement the code in the function `AltitudeControl()`
 - tune parameters `kpPosZ` and `kpPosZ`
 - tune parameters `kpVelXY` and `kpVelZ`

If successful, the quads should be going to their destination points and tracking error should be going down (as shown below). However, one quad remains rotated in yaw.

 - implement the code in the function `YawControl()`
 - tune parameters `kpYaw` and the 3rd (z) component of `kpPQR`

Tune position control for settling time. Don’t try to tune yaw control too tightly, as yaw control requires a lot of control authority from a quadcopter and can really affect other degrees of freedom.  This is why you often see quadcopters with tilted motors, better yaw authority!

<p align="center">
<img src="animations/scenario3.gif" width="500"/>
</p>

**Hint:**  For a second order system, such as the one for this quadcopter, the velocity gain (`kpVelXY` and `kpVelZ`) should be at least ~3-4 times greater than the respective position gain (`kpPosXY` and `kpPosZ`).

### Non-idealities and robustness (scenario 4) ###

In this part, we will explore some of the non-idealities and robustness of a controller.  For this simulation, we will use `Scenario 4`.  This is a configuration with 3 quads that are all are trying to move one meter forward.  However, this time, these quads are all a bit different:
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual

1. Run your controller & parameter set from Step 3.  Do all the quads seem to be moving OK?  If not, try to tweak the controller parameters to work for all 3 (tip: relax the controller).

2. Edit `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.

3. Tune the integral control, and other control parameters until all the quads successfully move properly.  Your drones' motion should look like this:

<p align="center">
<img src="animations/scenario4.gif" width="500"/>
</p>


### Tracking trajectories ###

Now that we have all the working parts of a controller, you will put it all together and test it's performance once again on a trajectory.  For this simulation, you will use `Scenario 5`.  This scenario has two quadcopters:
 - the orange one is following `traj/FigureEight.txt`
 - the other one is following `traj/FigureEightFF.txt` - for now this is the same trajectory.  For those interested in seeing how you might be able to improve the performance of your drone by adjusting how the trajectory is defined, check out **Extra Challenge 1** below!

How well is your drone able to follow the trajectory?  It is able to hold to the path fairly well?


### Extra Challenge 1 (Optional) ###

You will notice that initially these two trajectories are the same. Let's work on improving some performance of the trajectory itself.

1. Inspect the python script `traj/MakePeriodicTrajectory.py`.  Can you figure out a way to generate a trajectory that has velocity (not just position) information?

2. Generate a new `FigureEightFF.txt` that has velocity terms
Did the velocity-specified trajectory make a difference? Why?

With the two different trajectories, your drones' motions should look like this:

<p align="center">
<img src="animations/scenario5.gif" width="500"/>
</p>


### Extra Challenge 2 (Optional) ###

For flying a trajectory, is there a way to provide even more information for even better tracking?

How about trying to fly this trajectory as quickly as possible (but within following threshold)!


## Evaluation ##

To assist with tuning of your controller, the simulator contains real time performance evaluation.  We have defined a set of performance metrics for each of the scenarios that your controllers must meet for a successful submission.

There are two ways to view the output of the evaluation:

 - in the command line, at the end of each simulation loop, a **PASS** or a **FAIL** for each metric being evaluated in that simulation
 - on the plots, once your quad meets the metrics, you will see a green box appear on the plot notifying you of a **PASS**


### Performance Metrics ###

The specific performance metrics are as follows:

 

## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.
