# ECE 470 Motion Demonstration Project
### Group Members: _Michael Shea_ & _Jacob Heglund_
## Purpose
This document is meant to explain how to obtain the results that are displayed in [this video.](https://www.youtube.com/watch?v=4WNGfvI6qjA&app=desktop) The first steps were to choose a robot and a simulation enviroment. We chose to use [_V-Rep_](http://www.coppeliarobotics.com/) for our simulation enviroment and [Baxter](http://www.rethinkrobotics.com/baxter/) as the robot. The following steps are a guide to reproduce what we did in a _Windows 10_ enviroment. There are many ways to control a robot in V-Rep. We chose to use the [Remote API](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm) with [Python 3](). We used the [Spyder IDE](https://www.python.org/downloads/) for all the programming.
# Deliverable 5 - Demonstrate Motion Planning
## The Job
1. Write code that either returns a collision-free path between given start and goal configurations (i.e., a sequence of straight-line segments that is described by a list of nodes q1,…,qn where q1=θstart and qn=θgoal) or that returns failure if such a path could not be found. Your code must consider both self-collision and collision with obstacles.

2. Create a short video (at most 120 seconds) that shows the robot moving along at least 3 different collision-free paths. Each path must be non-trivial, in the sense that it (1) consists of more than one straight-line segment, and (2) could not have been replaced by a path consisting of only one straight-line segment from θstart to θgoal. In other words, your video should demonstrate that your path planner is really working, that it really does allow your robot to avoid both self-collision and obstacles.
## The Work
1. The inverse kinematics are implemented in _pathPlanner.py_.
2. _functions.py_ and _variables.py_ implement most of the functions from the [previous deliverables](https://github.com/smike1210/VRepBaxterRoballer/tree/master) to help obtain information such as the skew of a matrix, the different screw axis' of Baxter, etc.  This serves the purpose of keeping the main function _pathPlanner.py_ clean and readable.
3. When run, the program will prompt the user to input the spatial coordinates they want the end effector of the robot arm to reach.  Choosing the spatial cooddinate can be tricky, since there are obstacles in the environment, but the [video](https://www.youtube.com/watch?v=f9xUGDt-kLo) we made shows a few configurations that the robot can reach and plan a path for.  

## How to Reproduce What is Shown in the Video
1. First, [download _V-Rep_.](http://www.coppeliarobotics.com/) This guide assumes you will download the x64 _Windows 10_ version of _V-Rep_.
2. After the download is complete, open the _V-REP_PRO_EDU_version number_Setup.exe_ file and go through the instructions to install the program on your machine.
3. Once the program is fully installed, open _V-Rep_. When you first open it, it should look like [this](https://preview.ibb.co/cfZzNn/First_Time_Opening_VRep.png).
4. Next, it is time to focus on the programming of Baxter. Before we start programming, create a folder in a directory of your choice that is a copy of this [GitHub directory](https://github.com/smike1210/VRepBaxterRoballer/tree/master/deliverable4).
5. You directory should look something like [this](https://preview.ibb.co/kmKPNn/Directory.png).
6. The first step is to obtain the scene used that is shown in the video. To do this, open the file called _Baxter_no_scripts.ttt_. To do this, in _V-Rep_, go to _file_ -> _open scene_ and then select the _Baxter_no_scripts.ttt_ file. The scene should look like [this](https://preview.ibb.co/bGNiF7/scene.png).
7. Open _pathPlanner.py_ in _Spyder_. If you have not downloaded _Spyder_ already, you can do so [here](https://pythonhosted.org/spyder/installation.html). Downloading it is outside the scope of this document, but can be done very easily, and [this link](https://pythonhosted.org/spyder/installation.html) has a great guide on how to do so.
8. Make sure that _V-Rep_ is already open, and then [press the green play button on the top bar of _Spyder_](https://preview.ibb.co/bC4r2n/play_Button.png). After pressing the play button, if you have _Spyder_ and _V-Rep_ filling up two different halves of your screen, you should be able to interact with the program and see Baxter going through the motions seen in the video linked above.
9. Fin! (For now, at least...)

# Deliverable 4 - Demonstrate Collision Detection
## The Job
1. Write code that decides if a given set of joint variables (i.e., a configuration) places the robot in collision, either with itself or with something else in the environment. Note that one type of "self-collision" is violating joint limits (i.e., bounds on the value of a joint variable).

2. Create a short video (at most 120 seconds) that shows the robot in many different configurations and that indicates, in some way, which of these configurations place the robot in collision. Your video must show at least a few configurations each that result in no collision, in self-collision, and in collision with other things.
## The Work
1. The inverse kinematics are implemented in _collDetect.py_.
2. _collDetect.py_ implements most of the functions from the [previous deliverablse](https://github.com/smike1210/VRepBaxterRoballer/tree/master) to help obtain information such as the skew of a matrix, the different screw axis' of Baxter, etc...
3. When run, the program will go through different configurations, 10 of which result in no collision, 10 of which result in collision with outside objects, and 10 of which result in collision with self. The video attached with this deliverable shows the code recognizing these situations and alerting the user accordingly.
## How to Reproduce What is Shown in the Video
1. First, [download _V-Rep_.](http://www.coppeliarobotics.com/) This guide assumes you will download the x64 _Windows 10_ version of _V-Rep_.
2. After the download is complete, open the _V-REP_PRO_EDU_version number_Setup.exe_ file and go through the instructions to install the program on your machine.
3. Once the program is fully installed, open _V-Rep_. When you first open it, it should look like [this](https://preview.ibb.co/cfZzNn/First_Time_Opening_VRep.png).
4. Next, it is time to focus on the programming of Baxter. Before we start programming, create a folder in a directory of your choice that is a copy of this [GitHub directory](https://github.com/smike1210/VRepBaxterRoballer/tree/master/deliverable4).
5. You directory should look something like [this](https://preview.ibb.co/kmKPNn/Directory.png).
6. The first step is to obtain the scene used that is shown in the video. To do this, open the file called _Baxter_no_scripts.ttt_. To do this, in _V-Rep_, go to _file_ -> _open scene_ and then select the _Baxter_no_scripts.ttt_ file. The scene should look like [this](https://preview.ibb.co/bGNiF7/scene.png).
7. Open _collDetect.py_ in _Spyder_. If you have not downloaded _Spyder_ already, you can do so [here](https://pythonhosted.org/spyder/installation.html). Downloading it is outside the scope of this document, but can be done very easily, and [this link](https://pythonhosted.org/spyder/installation.html) has a great guide on how to do so.
8. Make sure that _V-Rep_ is already open, and then [press the green play button on the top bar of _Spyder_](https://preview.ibb.co/bC4r2n/play_Button.png). After pressing the play button, if you have _Spyder_ and _V-Rep_ filling up two different halves of your screen, you should be able to interact with the program and see Baxter going through the motions seen in the video linked above.
9. Fin! (For now, at least...)


# Deliverable 3 - Demonstrate forward kinematics
## The Job
1. Write code that implements numerical inverse kinematics.

2. Write code that (1) generates a goal pose either at random or in response to some kind of user input, (2) draws a frame in the simulator at the goal pose, (3) either moves the robot in the simulator to a set of joint variables that achieves the goal pose or indicates in some way that the goal pose is not reachable.

3. Create a short video (at most 120 seconds) showing the robot achieving several different goal poses - selected either at random or in response to user input - and highlighting agreement between the goal pose and the actual pose of the tool frame in each case. Your video should also show the result of asking the robot to achieve at least one goal pose that is not reachable.
## The Work
1. The inverse kinematics are implemented in _InverseKin.py_.
2. _InverseKin.py_ implements most of the code from the [previous deliverable](https://github.com/smike1210/VRepBaxterRoballer/tree/master/deliverable2) to obtain the screw axis' for both of Baxter's arms. It then uses that information to arrive at a set of thetas to reach a desired ending position by constantly updating theta using the formula (J<sup>T</sup>J-.1I)<sup>-1</sup>J^<sup>T</sup>V.
3. When run, the program asks the user to enter the coordinates for the left arm to go to. It then goes through the algorithm described above to come up with a set of thetas for all of Baxter's seven left arm revolute joints and the main central rotation joint. The code checks that the thetas are all within the working limits of each joint, or if not, starts the algorithm over again. It also detects for bad inputs (e.g. location the arm cannot reach), and will notify the user when this happens. It will move to the calculated thetas when those test are passed. The code then repeats for the right arm as well.
## How to Reproduce What is Shown in the Video
1. First, [download _V-Rep_.](http://www.coppeliarobotics.com/) This guide assumes you will download the x64 _Windows 10_ version of _V-Rep_.
2. After the download is complete, open the _V-REP_PRO_EDU_version number_Setup.exe_ file and go through the instructions to install the program on your machine.
3. Once the program is fully installed, open _V-Rep_. When you first open it, it should look like [this](https://preview.ibb.co/cfZzNn/First_Time_Opening_VRep.png).
4. Next, it is time to focus on the programming of Baxter. Before we start programming, create a folder in a directory of your choice that is a copy of this [GitHub directory](https://github.com/smike1210/VRepBaxterRoballer/tree/master/deliverable3).
5. You directory should look something like [this](https://preview.ibb.co/kmKPNn/Directory.png).
6. The first step is to obtain the scene used that is shown in the video. To do this, open the file called _Baxter_no_scripts.ttt_. To do this, in _V-Rep_, go to _file_ -> _open scene_ and then select the _Baxter_no_scripts.ttt_ file. The scene should look like [this](https://preview.ibb.co/bGNiF7/scene.png).
7. Open _InverseKin.py_ in _Spyder_. If you have not downloaded _Spyder_ already, you can do so [here](https://pythonhosted.org/spyder/installation.html). Downloading it is outside the scope of this document, but can be done very easily, and [this link](https://pythonhosted.org/spyder/installation.html) has a great guide on how to do so.
8. Make sure that _V-Rep_ is already open, and then [press the green play button on the top bar of _Spyder_](https://preview.ibb.co/bC4r2n/play_Button.png). After pressing the play button, if you have _Spyder_ and _V-Rep_ filling up two different halves of your screen, you should be able to interact with the program and see Baxter going through the motions seen in the video linked above.
9. Fin! (For now, at least...)



# Deliverable 2 - Demonstrate forward kinematics
## The Job
1. Draw a schematic of your robot.

2. Derive the forward kinematics of your robot, from the schematic.

3. Write code that implements the forward kinematics (i.e., a function that takes joint variables as input and returns the pose of the tool frame as output).

4. Write code that (1) moves the robot in the simulator to a given set of joint variables, and (2) draws a frame in the simulator at the pose that is predicted by your implementation of the forward kinematics.

5. Create a short video (at most 120 seconds) showing the robot move to several different configurations, and highlighting agreement between the predicted and actual pose of the tool frame in each case
## The Work
1. The forward kinematics of the Baxter robot are calculated using the products of exponentials method.  We assigned axes in a manner that was consistent to the [robot documentation](https://www.ohio.edu/mechanical-faculty/williams/html/pdf/BaxterKinematics.pdf).  We also got the lengths of the connections from the robot documentation.
2. Using our [schematic](https://github.com/smike1210/VRepBaxterRoballer/blob/master/deliverable2/ForwardKinematics.pdf), the  forward kinematics of the robot can easily be derived.  The calculations for the forward kinematics from the base rotational joint of the robot to the end effectors of both arms are done in _forwardKin.py_.  _forwardKin.py_ gives a pose of the end effectors of both arms.
3. The robot in the simulation can then be moved by using the capabilities of _forwardKin.py_ to interface with the V-Rep simulation.  Start V-Rep and open the _baxter no-script.ttt_ scene.  By choosing the desired joint angles, _forwardKin.py_ will move the joint angles of Baxter in the simulation.  
4. _forwardKin.py_ also has the functionality of placing frames at the calculated end effector pose that the joint angles achieve.  By first initializing a "reference frame" scene object in V-rep, and using the calculated end effector pose as an input, the reference frame will move in the simulation.
5. Open the file scene file _baxter no-script.ttt_ and run the Python script _forwardKin.py_ to see the results of our work this week.



# Deliverable 1 - Demonstrate robot motion with code in simulator
## The Job

1. Install a robot simulator on your laptop.

2. Insert your chosen robot and at least one object into the simulator.

3. Write code that makes all joints of the robot move.

4. Create a short video (at most 120 seconds) showing your robot move.

## The Work
1. First, [download V-Rep.](http://www.coppeliarobotics.com/) This guide assumes you will download the x64 _Windows 10_ version of V-Rep.
2. After the download is complete, open the _V-REP_PRO_EDU_version number_Setup.exe_ file and go through the instructions to install the program on your machine.
3. Once the program is fully installed, open V-Rep. When you first open it, it should look like [this](https://preview.ibb.co/cfZzNn/First_Time_Opening_VRep.png).
4. The first step is to obtain the scene used that is shown in the video. To do this, download the file called _baxter.ttt_ that can be downloaded from [here](https://drive.google.com/drive/u/1/folders/1e5i1j-gdS_KqfPlqJCSnCmjjXyLpepCP), and open it in V-Rep. To do this, in V-Rep, go to _file_ -> _open scene_ and then select the _baxter.ttt_ file that you just downloaded. The scene should look like [this](https://preview.ibb.co/bGNiF7/scene.png).
5. Next, it is time to focus on the programming of Baxter. The checkpoint for this part of the projects is to simply move all of the joints, which can be seen in the motion in the video linked above. Before we start programming, create a folder in a directory of your choice. We created a folder on the Desktop named "VRepBaxterRoballer".
6. Next, go to the location where the V-Rep program was installed. On our machines, this was _C:\Program Files\V-REP3_. Once in the folder, click on the folder _V_REP_PRO_EDU_ (note: this will be a different name if a different version of V-Rep was installed). Then go to the location  _programming_ -> _remoteApiBindings_ -> _python_ -> _python_. From this folder, copy the two files called _vrep.py_ and _vrepConst.py_, and paste them into the directory you created earlier.
7.  Next, go back the location were V-Rep was installed, and this time go to the location  _V_REP_PRO_EDU_ -> _programming_ -> _remoteApiBindings_ -> _lib_ -> _lib_ -> _Windows_ -> _64Bit_ and then copy and paste _remoteApi.dll_ into the directory you created.
8. Next, download and copy the _baxterMovementTest.py_ file that is in the same location as this document, and paste it into the directory you created.
9. You directory should look like [this](https://preview.ibb.co/kmKPNn/Directory.png).
10. Open _baxterMovementTest.py_ in _Spyder_. If you have not downloaded _Spyder_ already, you can do so [here](https://pythonhosted.org/spyder/installation.html). Downloading it is outside the scope of this document, but can be done very easily, and [this link](https://pythonhosted.org/spyder/installation.html) has a great guide on how to do so. After the file has been opened in _Spyder_, you should have a screen that looks like [this](https://preview.ibb.co/d9aaTS/Spyder_File.png).
11. Make sure that V-Rep is already open, and then [press the green play button on the top bar of _Spyder_](https://preview.ibb.co/bC4r2n/play_Button.png). After pressing the play button, if you go to the open V-Rep program, you should see Baxter going through the motions seen in the video linked above.
12. Fin! (For now, at least...)
