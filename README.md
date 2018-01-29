# Term2_Proj2_ParticleFilter
Localization Using Particle Filter

The target of this project is to apply Particle Filter to localize a lost car in a given map using C++ (Eclipse IDE).

## Content of this repo
- `scr` source code directory:
  - `main.cpp` - communicate with simulation tool, call functions to run the Particle Filter and calculate Error
  - `particle_filter.cpp` - initializes the Particle Filter, execute predict, update and resample function of Particle Filter

## Particle Filter Implementation

1. Initialize all the particles using GPS coordinates + position measurement noisy

2. Move all the particles using constant velocity constant yaw rate model, then add position measurement noisy

Note:

2.1 I think control noisy should be applid when using CTRV model, instead of imposing position noisy afterwards

2.2 When yaw rate is zero, CTRV model has to be changed to constant velocity constant yaw angle model.

3. Transfom sensor measurements from car local coordinates to map coordinates using PARTICLE position.

4. Associate EACH measurment in EACH particle with its closest landmark, calculate probability for each measurment & landmark association, the product of all the probability of each particle will be its weight.

5. Resample particles based on their weights, discrete_distribution is a perfect tool which can do the work automatically. The particle with highest weight can represent the location of the car. 

## Result

After the simulation tool run for 100 seconds, the output says "Success! Your particle filter passed!" 

## How to run the code
Clone this repo and perform
```
mkdir build && cd build
cmake ../src/  && make
./particle_filter 
```



