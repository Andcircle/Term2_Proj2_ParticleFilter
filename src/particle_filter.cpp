/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <ctime>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 1000;

	weights.resize(num_particles);

	static default_random_engine gen;
	gen.seed(time(0));

	// This line creates a normal (Gaussian) distribution for x, y, and theta.
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; ++i) {

		Particle p;

        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1./num_particles;
        p.id = i;

        particles.push_back(p);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	static default_random_engine gen;
	gen.seed(time(0));

	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for (int i = 0; i < num_particles; ++i) {

			double x = particles[i].x;
			double y = particles[i].y;
			double t = particles[i].theta;

			if (fabs(yaw_rate)<=0.0001)
			{
				x += velocity*delta_t*cos(t);
				y += velocity*delta_t*sin(t);
			}
			else
			{
				x = x + velocity/yaw_rate*(sin(t+yaw_rate*delta_t)-sin(t));
				y = y + velocity/yaw_rate*(cos(t)-cos(t+yaw_rate*delta_t));
				t = t + yaw_rate*delta_t;
			}

			particles[i].x = x + dist_x(gen);
			particles[i].y = y + dist_y(gen);
			particles[i].theta = t + dist_theta(gen);
		}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	std::vector<Map::single_landmark_s> map = map_landmarks.landmark_list;

	for (int i=0; i<num_particles; i++)
	{
		int num_sens = observations.size();
		particles[i].sense_x.resize(num_sens);
		particles[i].sense_y.resize(num_sens);
		particles[i].associations.resize(num_sens);
		particles[i].weight = 1.;


		for (int j=0; j<num_sens; j++)
		{
			double x = particles[i].x;
			double y = particles[i].y;
			double t = particles[i].theta;

			// transform to map x coordinate
			particles[i].sense_x[j]= x + (cos(t) * observations[j].x) - (sin(t) * observations[j].y);
			// transform to map y coordinate
			particles[i].sense_y[j]= y + (sin(t) * observations[j].x) + (cos(t) * observations[j].y);

			double sx = particles[i].sense_x[j];
			double sy = particles[i].sense_y[j];

			double dis = 10000;
			particles[i].associations[j]=0;
			int minId = 0;

			for (int k=0; k<map.size(); k++)
			{
				double range = dist(x,y,map[k].x_f,map[k].y_f);
				if (range<=sensor_range)
				{
					double dis_temp = dist(sx,sy,map[k].x_f,map[k].y_f);
					if (dis_temp<dis)
					{
						particles[i].associations[j]=map[k].id_i;
						dis = dis_temp;
						minId = k;
					}
				}
			}

			//Calculate weights

			//calculate normalization term
			double gauss_norm= (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));

			//calculate exponent
			double lx = map[minId].x_f;
			double ly = map[minId].y_f;

			double exponent = pow((sx - lx),2)/(2 * pow(std_landmark[0],2)) + pow((sy - ly),2)/(2 * pow(std_landmark[1],2));
			if (exponent>100) exponent = 100;

			//calculate weight using normalization terms and exponent
			particles[i].weight *= gauss_norm * exp(-exponent);
		}

		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	static default_random_engine gen;
	gen.seed(time(0));
	discrete_distribution<> dist_particles(weights.begin(), weights.end());
	vector<Particle> new_particles;
	new_particles.resize(num_particles);
	for (int i = 0; i < num_particles; i++) {
		new_particles[i] = particles[dist_particles(gen)];
	}
	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
