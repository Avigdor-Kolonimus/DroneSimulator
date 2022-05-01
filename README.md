# Drone Simulator
The main goal for this project is to try find good solution of small drone, flying inside indoor building without getting hit and crash.
The project fully autonomous 2d drone simulator, this simulator is trying to be realistic as much as it can, with lidar sensors,gyroscope sensor ,optical flow sensor and speed sensor.
We add a little bit noise to each sample to make it more realistic approach.
Basic API with real-time info and also manual controlling.
We also implemented kind of area mapping when the drone fly.
This project written in Java.

## Getting Started
When all files located inside eclipse or any other explorer we have the Maps folder which contains couple of maps with route and obstacles.
- Inside "SimulationWindow" in main we have map object with the path to any map you want to test.
- Inside "Drone" we have path to our image represents the drone itself.
After setting this up it is ready to launch.

## Sensors
- Lidar - check the distance between his spot forward and return the distance if hit, if not return 300 as max sample enabled.
In our project we set 4 lidars - one in front, second 90 degrees (left), third -90 degrees (right), fourd -180 degrees (back).
- Gyroscpoe - check the rotation of the drone. (0-360)
- Optical flow - check his location on map.
- Speed - max speed is 2m per second.

## Symbols 
- Yellow mark - mapped area.
- Black circle - his purpose to get some idea from where drone came and simply make some route that his passed.(for navigation)
- Red points - represents the wall point.
- Blue line - his whole route.

## API description
Really simple API with few buttons -
- Play/Pause button - plays and pauses the flight simulator.
- Arm/Disarm button - drone landing or lifting.
- Toggle Map - allows you to hide the real map, entering to "real time" vision.
- Open Graph - displays a graph with coordinates.
- Return Home - by clicking it drone will return to starting point.

## Map rules
If you wish to add custom map it has to be black/white pixels- black is wall/obstacle, white is safe pass.

## Required
- JGrapht library.

## Images
![](./Images/API.png)
![](./Images/map11_1.png)
![](./Images/map11_2.png)
![](./Images/map11_3.png)
![](./Images/map11_4.png)

