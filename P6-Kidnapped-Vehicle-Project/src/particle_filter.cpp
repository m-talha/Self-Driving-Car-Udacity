/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Mohammed Talha & Tiffany Huang
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
using std::normal_distribution;

#define NUM_PARTICLES 10
#define EPSILON 0.001
#define OBSERVABLE_LANDMARKS 5

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   */

  num_particles = NUM_PARTICLES;  // Set the number of particles

  // Sample from Gaussian PDF x, y & theta
  std::default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i=0; i < num_particles; ++i) {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;

    particles.push_back(p);
    
  }

  weights.resize(NUM_PARTICLES);
  is_initialized = true;

  // Testing
  // std::cout << "GPS: " << x << ", " << y << ", " << theta << std::endl;
  // std::cout << "Samples: " << std::endl;
  // for (int i=0; i<5; ++i) {
  //   int j = rand() % num_particles;
  //   std::cout << j << ", " << particles[j].x << ", " << particles[j].y << ", " << particles[j].theta << std::endl;
  // }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  std::default_random_engine gen;
  normal_distribution<double> dist_x(0,std_pos[0]);
  normal_distribution<double> dist_y(0,std_pos[1]);
  normal_distribution<double> dist_theta(0,std_pos[2]);
  
  // Cache constants
  double yaw_delta = yaw_rate * delta_t;
  double vel_yaw_rate = velocity / yaw_rate;

  // Predict the vehicle state using the bicycle motion model
  for (auto& particle : particles) {

    if (fabs(yaw_rate) < EPSILON) {
      particle.x += velocity * delta_t * cos(particle.theta);
      particle.y += velocity * delta_t * sin(particle.theta);
    } else {
      particle.x += vel_yaw_rate * (sin(particle.theta + yaw_delta) - sin(particle.theta));
      particle.y += vel_yaw_rate * (cos(particle.theta) - cos(particle.theta + yaw_delta));
      particle.theta += yaw_delta; 
    }

    // Add random noise
    particle.x += dist_x(gen);
    particle.y += dist_y(gen);
    particle.theta += dist_theta(gen); 
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
   * Update the weights of each particle using a mult-variate Gaussian 
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

  for (auto i=0; i < particles.size(); ++i) {
    double weight = 1.0;
    // Landmarks observable if this particle correctly represented the car state
    vector<Map::single_landmark_s> observable_landmarks;
    // Check if each landmark is within the detectable zone
    for (auto lmark : map_landmarks.landmark_list) {
      if (dist(lmark.x_f, lmark.y_f, particles[i].x, particles[i].y) <= sensor_range) {
        observable_landmarks.push_back(Map::single_landmark_s { lmark.id_i,
                                                                lmark.x_f,
                                                                lmark.y_f });
      }
    }

    // Lists of landmark attributes associated with each particle
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    // Transform each observation from current particle frame to map frame
    for (auto obs : observations) {
      LandmarkObs temp;
      temp.id = obs.id;
      temp.x = particles[i].x + (cos(particles[i].theta) * obs.x) - (sin(particles[i].theta) * obs.y);
      temp.y = particles[i].y + (sin(particles[i].theta) * obs.x) + (cos(particles[i].theta) * obs.y);

      // For each observation, assign the closest observable landmark
      // Equivalent to dataAssociation function above
      double min_dist = std::numeric_limits<double>::max();
      int lmark_id;
      double lmark_x, lmark_y;
      for (auto lmark : observable_landmarks) {
        auto distance = dist(lmark.x_f, lmark.y_f, temp.x, temp.y);
        if (distance < min_dist) {
          min_dist = distance;
          lmark_id = lmark.id_i;
          lmark_x = lmark.x_f;
          lmark_y = lmark.y_f;
        }
      }
      associations.push_back(lmark_id);
      sense_x.push_back(lmark_x);
      sense_y.push_back(lmark_y);

      // Calculate weight of particle as product of observation probabilities
      weight *= multi_gauss(temp.x, temp.y, lmark_x, lmark_y, std_landmark[0], std_landmark[1]);
    }

    // Store associated landmarks for each particle 
    SetAssociations(particles[i], associations, sense_x, sense_y);
    // Update particle weight
    particles[i].weight = weight;
    // Store weights of each particle for resampling
    weights.at(i) = weight;
  }
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  // Use a discrete distribution to simulate sampling in proportion to weight
  std::default_random_engine gen;  
  std::discrete_distribution<> dist_weights(weights.begin(), weights.end());

  // Sample n particles
  vector<Particle> resampled(particles.size());
  for (int i = 0; i < NUM_PARTICLES; ++i) {
    auto j = dist_weights(gen);
    resampled[i] = particles[j];
  }

  // Store resampled particles as new particles
  particles = resampled;
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