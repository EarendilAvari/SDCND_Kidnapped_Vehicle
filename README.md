# Self Driving Car Nanodegree

## Project 6: Particle Filter

This project consists of a localization system which can estimate very accurately the position of a car on a map. The system estimates the 2D position of the car and its heading (yaw angle). 

This software is programmed in C++11 using the standard template library.

In order to estimate where the car is in a map, a collection of landmarks is used. These can be found on the file /data/map_data.txt which is readed by the software.

GPS position of the object, its speed and yaw rate and the observations are received from the [ Udacity term 2 simulator ](https://github.com/udacity/self-driving-car-sim). Therefore the simulator is needed in order to run this software. To communicate this software with the simulator the [uWebSockets API](https://github.com/uNetworking/uWebSockets) is used. 

To compile the software, GCC 7.4.0 and CMake 3.15.0-rc1 is used.

### About the algorithms

The purpose of this software is to implement a particle filter which uses processed LIDAR measurements to estimate the position of an autonomous car on a map. In this section it is explained briefly how the algorithm works.

A particle filter is called like that because it does not estimate the position of an object once but it estimates it a lot of times. All these possible locations of the object are called "particles". The formal name of the algorithm is "Monte Carlo localization". Every one of these "particles" has a weight which indicates how probable is that this particle is the real position of the object. In the case of this project, the particle with the highest weight is considered the position of the car.

In order to determine the weights and with it the position of the object, four phases are executed: Initialization phase, prediction phase, update phase and resampling phase.

#### Initialization phase

As its name says, it initializes the position of the particles using GPS, it only occurs when the software starts up. 

#### Prediction phase

Within this phase, the new position of the particles is predicted using a motion model. In the case of this software, the bycicle model is used. This model uses the linear speed of the car and the yaw or heading rate. This model assumes that the acceleration of the object is constant.

#### Update phase

Within this phase, the weights of the particles are updated. The weights are updated calculating the two dimensional Gaussian probability between the LIDAR observations and the actual position of the landmarks in the map. This phase is divided in three subphases which are done separately for every particle:

- Convertion of observations: The observations received from the LIDAR are in car coordinates, while the landmark positions are in map coordinates. In order to use these observations, they need to be converted to map coordinates. This is done applying the homogeneous transformation commonly used in robotics.

- Data association: In this subphase every observation is associated with a landmark. This association is then used to calculate the weight of the particle.

- Weight calculation: The weight of the particle is calculated getting the 2D Gaussian probability between the observations and the landmarks associated before. This is calculated for every group (observation, landmark) and then all calculations are multiplied together to get the weight of the particle.

#### Resampling phase

In this phase it is decided which particles will be used for the next cycle. A particle can be selected multiple times and the times a particle is selected depends on its weight calculated on the last phase. 

This process is done using the resampling wheel method explained by Sebastian Thrun on [this video](https://www.youtube.com/watch?v=wNQVo6uOgYA).


### Results

In order to test the performance of the software, the RMSE (Root mean squared error) between the position of the best particle and the ground truth data is used (the exact position of the car). This is not done within this software, instead it is done by the simulator, which receives the position of the best particle as a json message.

The results of the software can be seen in the following video:

[![Particle filter](https://i.imgur.com/kVXCUbS.png)](https://youtu.be/D3h6b6f7BwA "Particle filter") 


### How to use

The software is already compiled in this repository in the folder "build", but here is explained how it can be compiled again:

- Install the required dependencies: gcc/g++ >= 5.4, make >= 4.1, cmake >= 3.5.
- Install uWebSockets using the file /install-linux.sh
- Delete the directory build and create a new one with: mkdir build.
- Run the command ./build.sh to compile.
- Run the command ./run.sh to start the software.
- Open the term 2 simulator and select Project 3: Kidnapped Vehicle.
- Press start and see how the car moves and the measurements and estimations are shown.

These steps are for a linux environment. For other operative systems see the [Udacity's repository](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).








