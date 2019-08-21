/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO (DONE): Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO (DONE): Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // Set the number of particles
  std::default_random_engine random_gen;  // Random number generator used for normal distribuition

  // These three are objects which generate random numbers based on a normal distribution with
  // mean and standard deviation given by the first parameter and second parameter respectively given here. 
  std::normal_distribution<double> x_rand(x, std[0]);
  std::normal_distribution<double> y_rand(y, std[1]);
  std::normal_distribution<double> th_rand(theta, std[2]);

  for (int i = 0; i < num_particles; i++) {
    // Declares a particle which will be pushed to the particle's list
    Particle curr_part;

    // Initializes the particle position and orientation with the normal distribution generators
    curr_part.id = i;
    curr_part.x = x_rand(random_gen);
    curr_part.y = y_rand(random_gen);
    curr_part.theta = th_rand(random_gen);
    // All the weights are initialized with 1
    curr_part.weight = 1;

    // Pushs particle to vector of particles
    particles.push_back(curr_part);

    // Only for debugging, comment after debugged.
    // std::cout << "DEBUG PRINT INIT PHASE. Particle " << i << " initialized with x: " << 
    //  curr_part.x << ", y: " << curr_part.y << ", th: " << curr_part.theta << std::endl;

  }

    // Sets this variable which indicates that the filter is initialized.
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO (DONE): Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine random_gen; // Random number generator used for normal distribuition

  // This time the normal distribution objects have mean 0.0 and they are added to the final result 
  std::normal_distribution<double> x_noise(0.0, std_pos[0]);
  std::normal_distribution<double> y_noise(0.0, std_pos[1]);
  std::normal_distribution<double> th_noise(0.0, std_pos[2]);

  // The motion model is different for yaw_rate != 0 and yaw_rate == 0
  // Since the yaw rate is a parameter given to the function, doing the if before the for makes the program
  // a bit faster
  if (yaw_rate == 0) {
    for (int i = 0; i < num_particles; i++) {
      // Only for debugging, comment after debugged.
      // std::cout << "DEBUG PRINT PREDICTION PHASE. Velocity: " << velocity << " yaw rate: " << yaw_rate << std::endl;
      // std::cout << "Particle " << i << "x0: " << particles[i].x << " y0: " << particles[i].y << " th0: " << particles[i].theta << std::endl;

      particles[i].x += velocity*delta_t*cos(particles[i].theta) + x_noise(random_gen);
      particles[i].y += velocity*delta_t*sin(particles[i].theta) + y_noise(random_gen);

      // Only for debugging, comment after debugged.
      // std::cout << "Particle " << i << "x0: " << particles[i].x << " y0: " << particles[i].y << " th0: " << particles[i].theta << std::endl;
    }
  }
  else {
    for (int i = 0; i < num_particles; i++) {
      // Only for debugging, comment after debugged.
      // std::cout << "DEBUG PRINT PREDICTION PHASE. Velocity: " << velocity << " yaw rate: " << yaw_rate << std::endl;
      // std::cout << "Particle " << i << "x0: " << particles[i].x << " y0: " << particles[i].y << " th0: " << particles[i].theta << std::endl;

      particles[i].x += (velocity/yaw_rate)*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta)) + x_noise(random_gen);
      particles[i].y += (velocity/yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t)) + y_noise(random_gen);
      particles[i].theta += yaw_rate*delta_t;

      // Only for debugging, comment after debugged.
      // std::cout << "Particle " << i << "x0: " << particles[i].x << " y0: " << particles[i].y << " th0: " << particles[i].theta << std::endl;
    }
  }  

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}