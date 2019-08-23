/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016. Modified at Aug 22, 2019.
 * Author: Tiffany Huang. Modified by Jorge Rilling.
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

void ParticleFilter::dataAssociation(const Map &map_landmarks, 
                                     const std::vector<LandmarkObs>& observations, unsigned int part_number) {
  /**
   * TODO (DONE): Find the map landmark that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   */

  // These vectors are filled up with the asociations of the observations from a particle to 
  // map landmarks
  std::vector<int> landmark_ids;
  std::vector<double> landmark_x;
  std::vector<double> landmark_y;

  // These three variables are used to process the asociations corresponding to one observation
  // They are used to fill up the vectors
  int curr_id;
  double curr_landmark_x;
  double curr_landmark_y;

  // It iterates through all observations (in map coordinates)
  for (unsigned int i = 0; i < observations.size(); i++) {
    // This variable corresponds to the minimum distance between the current observation and the 
    // landmarks. 
    double min_distance = 1000.0;
    // It iterates through all map landmarks to find the one which has the minimum
    // distance with the current observation
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      double curr_distance = dist(observations[i].x, observations[i].y, 
                              map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
      if (curr_distance < min_distance) {
        min_distance = curr_distance;
        curr_id = map_landmarks.landmark_list[j].id_i;
        curr_landmark_x = map_landmarks.landmark_list[j].x_f;
        curr_landmark_y = map_landmarks.landmark_list[j].y_f;
      }
    }
    // The vectors are filled up
    landmark_ids.push_back(curr_id);
    landmark_x.push_back(curr_landmark_x);
    landmark_y.push_back(curr_landmark_y);
  }

  // The particle's vectors are updated
  particles[part_number].associations = landmark_ids;
  particles[part_number].sense_x = landmark_x;
  particles[part_number].sense_y = landmark_y;

}

  /**
   * convertObservations Convert the observation vector from the coordinate system  
   * of a particle to the map coordinate system
   * @param observations Vector of landmark observations in car coordinates
   * @param part_number No. of the particle used to do the transformation.
   * @output Vector of landmark observations in map coordinates
   */
std::vector<LandmarkObs> ParticleFilter::convertObservations
                        (const std::vector<LandmarkObs> &observations, unsigned int part_number) {
  // Creates an empty vector of observations with map coordinate system and an observation
  // object to fill the vector up.
  std::vector<LandmarkObs> observations_map;
  LandmarkObs current_observation;

  for (unsigned int i = 0; i < observations.size(); i++) {
    current_observation.id = observations[i].id;
    current_observation.x = particles[part_number].x + 
                            cos(particles[part_number].theta) * observations[i].x -
                            sin(particles[part_number].theta) * observations[i].y;
    current_observation.y = particles[part_number].y +
                            sin(particles[part_number].theta) * observations[i].x +
                            cos(particles[part_number].theta) * observations[i].y;

    // Pushes "current_observation" to observations vector in map coordinates
    observations_map.push_back(current_observation);
  }
  return observations_map;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const std::vector<LandmarkObs> &observations, 
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

  // Takes from "observations" only the observations with modulo smaller than "sensor_range" (Tested)
  std::vector<LandmarkObs> observations_on_range;
  for (unsigned int i = 0; i < observations.size(); i++) {
    if (module(observations[i].x, observations[i].y) <= sensor_range) {
      observations_on_range.push_back(observations[i]);
    }
  }

  // Loops through all particles
  std::vector<LandmarkObs> observations_conv_map;
  // std::cout << "DEBUG PRINT UPDATE PHASE " << std::endl;
  for (int i = 0; i < num_particles; i++) {
    // Converts the observations to map coordinates
    observations_conv_map = convertObservations(observations_on_range, i);
    // Associates the observations to map landmarks and save the asociations on the particles object vector
    dataAssociation(map_landmarks, observations_conv_map, i);
    // Calculates the weights of the particles using the 2D Gaussian probability function 
    // with the observation in map coordinates as test point and the position of the associated 
    // landmark as mean.
    particles[i].weight = 1.0;
    for (unsigned int j = 0; j < particles[i].associations.size(); j++) {
      // The 
      particles[i].weight *= _2dGauss(observations_conv_map[j].x, observations_conv_map[j].y, 
                              particles[i].sense_x[j], particles[i].sense_y[j], std_landmark[0], std_landmark[1]);
    }
    // std::cout << "Particle " << i << " new weight: " << particles[i].weight << std::endl;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * The method uses the resampling wheel explained by Sebastian Trunn on the 
   * video: https://www.youtube.com/watch?v=wNQVo6uOgYA
   */
  std::vector<Particle> new_particles;

  // Determines the maximal weight
  double max_weight = 0.0;
  for (unsigned int i = 0; i < particles.size(); i++) {
    if (particles[i].weight > max_weight) {
      max_weight = particles[i].weight;
    }
  }

  // Random particle index used by the weights wheel
  unsigned int index = rand() % num_particles;

  // Beta is a random number between 0 and 2*max_weight
  double beta = 0.0;
  std::default_random_engine random_gen;
  std::uniform_real_distribution <> beta_gen(0.0, 2*max_weight);

  // All the particles are iterated. Here it is important to note that
  // the index used to access the particles is not the variable i,
  // instead it is the random variable "index" defined before.
  for (unsigned int i = 0; i < num_particles; i++) {
    // For every particle being processed beta is added up with a 
    // random number between 0.0 and twice the weight of the most
    // important particle
    beta += beta_gen(random_gen); 
    while (beta > particles[index].weight) {
      beta -= particles[index].weight;
      index = (index + 1)%num_particles;
    }
    // The particle added to the vector of new particles is the first
    // one found with weight bigger than beta. Beta is made smaller with  
    // the weight of every not accepted particle.
    new_particles.push_back(particles[index]);
  }
  particles = new_particles;
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