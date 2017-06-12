Musical Instument and Visualizer  
Akshay Rajagopal, akshayr2  
Charles Hale, cphalepk  

This directory has two subdirectories
VR Final Project contains all the Unity code
vrduino contains all the VRduino code

You can run the code by loading vrduino.ino on an Arduino and opening scene1 in the Assets folder in Unity.

The ReadUSB.cs script attached to the wand in the scene is disabled. When this code runs on a Mac, it will print out your available USB ports, you can change to that port and reenable the script if you with for this to work with the imu.

Positional tracking was implemented but is disabled in vrduino.ino for demo (it instead just streams Quaternions from the gyro and accelerometer).

Within the scene, we have a series of pylons as a visual cue for where to point the wand. The left side corresponds to low pitch while the right side corresponds to high pitch. The bottom corresponds to high volume while the top corresponds to low volume.

The majority of the work we did is in:
* /vrduino
  * vrduino.ino - contains vrduino code for streaming in the right format.
* /VR Final Project/Assets:
  * scene1.unity - main scene  
  * ReadUSB.cs - streams input values for wand position/orientation.  
  * pylonpositioning.cs - sets the position for each of the cylinders around the perimeter that correspond to music visualization.  
  * Cylinder.cs - attached to each cylinder on the perimeter so that they react to the STFT of the instrument output.  
  * Instrument.cs - attached to an audio source attached to the wand so that we set the sound in relation the the position and orientation of the object it is attached to.  
  * SpotLightController.cs - controls the color that appears when we point the wand on the plane spanned by the pylons.  
  * strings.wav - the edited audio file for looping sound for our instrument.  
