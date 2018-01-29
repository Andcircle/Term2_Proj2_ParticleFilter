# Term2_Proj2_ParticleFilter
Localization Using Particle Filter

The target of this project is to apply Particle Filter to localize a lost car using C++ (Eclipse IDE).

## Content of this repo
- `scr` source code directory:
  - `main.cpp` - communicate with simulation tool, call functions to run the Particle Filter and calculate Error
  - `particle_filter.cpp` - initializes the Particle Filter, execute predict and update function of Particle Filter

## Particle Filter Implementation


## Result

After the simulation tool run for 100 seconds, the output says "Success! Your particle filter passed!" 

## How to run the code
Clone this repo and perform
```
mkdir build && cd build
cmake ../src/  && make
./ExtendedKF 
./ExtendedKF 
```



