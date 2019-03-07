# MR-Obstacle-Avoidance
The fuzzy-logic based controller for the Mobile Robot to avoid obstacles based on the data from its sonar sensors.

The controller takes 3 inputs from sonar sensors called inputLEFT, inputFRONT, and inputRIGHT. InputFRONT is the minimum between the two front sensors.

<img src="https://github.com/Lavr18/MR-Obstacle-Avoidance/blob/master/inputSensors.png" width="408" height = "325">

The memebrship function for each input is as follows.

![Alt text](https://github.com/Lavr18/MR-Obstacle-Avoidance/blob/master/inputMFforGit.PNG)

The membesrship function for each output 

The table of fuzzy rules.

![Alt text](https://github.com/Lavr18/MR-Obstacle-Avoidance/blob/master/fuzzyRules.PNG)

