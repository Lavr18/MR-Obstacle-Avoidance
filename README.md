# MR-Obstacle-Avoidance
A fuzzy-logic based controller for the Mobile Robot to avoid obstacles based on the data from its sonar sensors.

The controller takes 3 inputs from sonar sensors called inputLEFT, inputFRONT, and inputRIGHT. InputFRONT is the minimum between the two front sensors.

<img src="https://github.com/Lavr18/MR-Obstacle-Avoidance/blob/master/inputSensors.png" width="408" height = "325">

The memebrship function for each input is as follows. X axis represents the distance to an obstacle in mm.

![Alt text](https://github.com/Lavr18/MR-Obstacle-Avoidance/blob/master/inputMFforGit.PNG)

The outputs are the robot's motors. The membesrship function for each output is shown below. X axis is the motor speed in mm/s.

![Alt text](https://github.com/Lavr18/MR-Obstacle-Avoidance/blob/master/outputMFforGit.PNG)

The table of fuzzy rules.

![Alt text](https://github.com/Lavr18/MR-Obstacle-Avoidance/blob/master/fuzzyRules.PNG)

The link to the video where the robot avoids obstacles: https://www.youtube.com/watch?v=iqcaPnnepPo

The link to the full report: https://drive.google.com/file/d/1pkQx3OK0kQITQ3sZTtjMU8FuwTggx_yA/view

