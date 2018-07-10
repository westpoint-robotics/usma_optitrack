# usma_optitrack
## Overview
Instructions for configuring a Natural Point Optitrack motion capture system and VRPN client in order to stream pose data to ROS.
The goal is to track rigid bodies using the OptiTrack motion capture system and stream pose data (position and orientation) to a base station computer.
Required:
1. Mocap computer - Windows desktop running Motive and VRPN server
2. Base station - Ubuntu laptop running ROS and VRPN client

## Motion capture computer setup
1. Turn on motion capture cameras through powerstrip and launch Motive. 
   - There is a delay between turning on the cameras and the connection to Motive. Once the lights on the lower right corner of the cameras are blinking, they can be connected to Motive.
2. Open existing project and select calibration file (March 2018 file works well for general purposes). At this point, the blue lights on the cameras should activate.
3. Select Capture Layout found in the upper righthand corner shown below. ![alt tag](Capture.png)
4. Click and drag over the markers of a single body. Right click on the selected body and choose Rigid Body -> Create New Rigid Body. Repeat for all desired bodies. (keep in mind that the orientation they are created in will be used as no rotation).
5. Open View -> Data Streaming
   - Assign each rigid body a unique "Name" field (found in the lower left corner). This is the ID that mocap\_optitrack will use to identify them and the ROS topic they will be published to.
   - If an offset is required, click the "Transformation" tab (again in the lower left corner) and offset the pivot points of rigid bodies to reflect actual positions of the object they represent.
   - As a sanity check, click the "Real-Time Info" tab and move your rigid body to see that it is moving as expected. (The axes on this computer will not be the same as those published to ROS so don't worry if these axes are not aligned as desired).
   - In the "Data Streaming" portion (on the righthand side of the window), ensure that the "VRPN Streaming Engine - Broadcast Frame Data" is enabled. ![alt tag](DataStream.png) 
   - Still in the "Data Streaming" section, scroll through the "Optitrack Streaming Engine" portion. The inputs should match the following  
   ![alt tag](DataStreamcropped.png) ![alt tag](BroadcastVRPN.png)  
   The most important fields (and the ones that require change from their possible default setting) are "Stream Unlabled Markers" should be "False", "Stream Rigid Bodies" - "True", "Up Axis" - "Z Up" (this will cause a rotation about the X Axis to get Z Up), and the "Multicast Interface" should have the address 192.168.200.XX (this is the DS3 network).
6. Switch to Ubuntu

## Base station setup [[ref]](http://wiki.ros.org/vrpn_client_ros)
On a laptop with Ubuntu 16.04 and ROS Kinetic:
1. Clone this repositiory to your catkin workspace
``` 
cd ~/catkin_ws/src
git clone 
cd ..
catkin_make
```
3. Launch the basic.launch file: `roslaunch usma_optitrack test.launch`
4. Data can be visualized using `rosrun rqt_multiplot rqt_multiplot` (Multiplot plugin must be installed for rqt)
   - Go to settings  
   ![alt tag](Multiplot1.png)
   - Add new curve  
   ![alt tag](Multiplot2.png)
   - Choose topic and field to plot and choose graph window size (graph window defaults to automatic)  
   ![alt tag](Multiplot3.png)
   - Hit enter and go back to graph output window. Hit play to display current data output  
   ![alt tag](Multiplot4.png)
