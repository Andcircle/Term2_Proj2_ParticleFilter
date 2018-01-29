# Term2_Proj2_ParticleFilter
Localization Using Particle Filter

The target of this project is to apply Particle Filter to localize a lost car using C++ (Eclipse IDE).

## Content of this repo
- `scr` source code directory:
  - `main.cpp` - communicate with simulation tool, call functions to run the Particle Filter and calculate Error
  - `particle_filter.cpp` - initializes the Particle Filter, execute predict and update function of Particle Filter

## Particle Filter Implementation

1. move robot and take measurement
2. take all the particles in the map do the same motion, then do the measurements too.
3. Compare particle measurement to robot measurement, then calculate the weights. (still use gaussion distribution with sensor noise as std)
4. resample particles based on weights.

## Result

After the simulation tool run for 100 seconds, the output says "Success! Your particle filter passed!" 

## How to run the code
Clone this repo and perform
```
mkdir build && cd build
cmake ../src/  && make
./particle_filter 
```



