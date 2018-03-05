# ECE 470 Motion Demonstration Project Checkpoint
### Group Members: _Michael Shea_ & _Jacob Heglund_
## Purpose
This document is meant to explain how to obtain the results that are displayed in  
[this video.](www.youtube.com) The first steps were to chose a robot and a simulation enviroment. We chose to use [_V-Rep_](http://www.coppeliarobotics.com/) for our simulation enviroment and [Baxter](http://www.rethinkrobotics.com/baxter/) as the robot. The following steps are a guide to reproduce what we did in a _Windows 10_ enviroment. There are many way to control a robot in V-Rep. We chose to use the [Remote Api](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm) with [Python 3](). We used the [Spyder IDE](https://www.python.org/downloads/) for all the programming.

## Steps Taken
1. First, [download V-Rep.](http://www.coppeliarobotics.com/) This guide assumes you will download the x64 _Windows 10_ version of V-Rep.
2. After the download is complete, open the _V-REP_PRO_EDU_version number_Setup.exe_ file and go through the instructions to install the program on your machine.
3. Once the program is fully installed, open V-Rep. When you first open it, it should look like [this](https://preview.ibb.co/cfZzNn/First_Time_Opening_VRep.png).
4. The first step is to obtain the scene used that is shown in the video. To do this, download the file caled _baxter.tt_ that is in the location of this document, and open it in V-Rep. To do this, in V-Rep, go to _file_ -> _open scene_ and then select the _baxter.tt_ file that you just downloaded. The scene should look like [this](https://preview.ibb.co/bGNiF7/scene.png).
5. Next, it is time to focus on the programming of Baxter. The checkpoint for this part of the projects is to simply move all of the joints, which can be seen in the motion in the video linked above. Before we start programming, create a folder in a directory of your choice. We created a folder on the Desktop named "VRepBaxterRoballer".
6. Next, go to the location where the V-Rep program was installed. On our machines, this was _C:\Program Files\V-REP3_. Once in the folder, click on the folder _V_REP_PRO_EDU_ (note: this will be a differnet name if a different version of V-Rep was installed). Then go to the location  _programming_ -> _remoteApiBindings_ -> _python_ -> _python_. From this folder, copy the two files called _vrep.py_ and _vrepConst.py_, and paste them into the directory you created earlier.
7.  Next go back the location were V-Rep was installed, and this time go to the location  _V_REP_PRO_EDU_ -> _programming_ -> _remoteApiBindings_ -> _lib_ -> _lib_ -> _Windows_ -> _64Bit_ and then copy and paste _remoteApi.dll_ into the directory you created.
8. Next, download and copy the _baxterMovementTest.py_ file that is in the same location as this document, and paste it into the directory you created.
9. You directory should look like [this](https://preview.ibb.co/kmKPNn/Directory.png).
10. Open _baxterMovementTest.py_ in _Spyder_. If you have not downloaded _Spyder _ already, you can do so [here](https://pythonhosted.org/spyder/installation.html). Downloading it is outside the scope of this document, but can be done very easily, and [this link](https://pythonhosted.org/spyder/installation.html) has a great guide on how to do so. After the file has been opened in _Spyder_, you should have a screen that looks like [this](https://preview.ibb.co/d9aaTS/Spyder_File.png).
11. Make sure that V-Rep is already open, and then [press the green play button on the top bar of _Spyder_](https://preview.ibb.co/bC4r2n/play_Button.png). After pressing the play button, if you go to the open V-Rep program, you should see Baxter going through the motions seen in the video linked above.
12. Fin! (For now, at least...)
