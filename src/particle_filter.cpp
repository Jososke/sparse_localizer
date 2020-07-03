#include "particle_filter.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <float.h>
#include "helper_functions.h"
#include "multiv_gauss.h"

using std::string;
using std::vector;
using std::normal_distribution;

  //Sample from these normal distributions like this: 
  //   sample_x = dist_x(gen);
  //   where "gen" is the random engine initialized here
  std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles

  if (is_initialized) return;

  // This line creates a normal (Gaussian) distribution for x
  normal_distribution<double> dist_x(x, std[0]);
  // This line creates a normal (Gaussian) distribution for y
  normal_distribution<double> dist_y(y, std[1]);
  // This line creates a normal (Gaussian) distribution for theta
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; i++)
  {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    particles.push_back(p);
  }
  // The filter is now initialized.
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std[], 
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   */

  // Creating Gaussian distributions
  normal_distribution<double> dist_x(0, std[0]);
  normal_distribution<double> dist_y(0, std[1]);
  normal_distribution<double> dist_theta(0, std[2]);

  for (int i = 0; i < num_particles; i++)
  {
    //Add measurements to each particle
    if (fabs(yaw_rate) < .0001) //when yaw is near 0
    {
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }
    else
    {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }
    
    //adding random Gaussian noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   */

  for (int i = 0; i < observations.size(); i++)
  {
    double minDist = DBL_MAX; //init to really large number
    int mapId = -1; //initializing the id
    for (int j = 0; j < predicted.size(); j++)
    {
      double distance = pow(observations[i].x - predicted[j].x, 2) + 
                        pow(observations[i].y - predicted[j].y, 2);
      if (distance < minDist)
      {
        minDist = distance;
        mapId = predicted[j].id;
      }
    }
    observations[i].id = mapId; //updating observation based on min prediction
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * Update the weights of each particle using a mult-variate Gaussian 
   *   distribution.
   * The observations are given in the VEHICLE'S coordinate system. 
   *   The particles are located according to the MAP'S coordinate system. 
   *   There will need to be a transform between the two systems.
   *   This transformation requires both rotation AND translation (but no scaling).
   */

  // predict measurement to map landmarks for each particle
  for (int i = 0; i < num_particles; i++)
  {
    vector<LandmarkObs> predicted;
    //getting the values of the particle
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;

    //determinging the correct measurments for each particle in the sensor range
    for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
    {
      //only consider landmarks in the sensor range
      double distance = sqrt(pow(x - map_landmarks.landmark_list[j].x_f, 2) + 
                        pow(y - map_landmarks.landmark_list[j].y_f, 2));
      if (distance <= sensor_range)
        predicted.push_back({map_landmarks.landmark_list[j].id_i,
                            map_landmarks.landmark_list[j].x_f,
                            map_landmarks.landmark_list[j].y_f});
    }
  
    // transforming observations from vehicle frame to map frame.
    vector<LandmarkObs> transformed_observations;
    for(unsigned int j = 0; j < observations.size(); j++) 
    {
      //2D homogeneous transformation 
      double map_x = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
      double map_y = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
      transformed_observations.push_back(LandmarkObs{observations[j].id, map_x, map_y});
    }

    //associate the predicted measurment with the observed measurement
    dataAssociation(predicted, transformed_observations);

    //update the weight of the particle
    particles[i].weight = 1.0;

    for (auto curObs : transformed_observations)
    {
      // find coordinates of the prediction associated with the observation
      double mu_x = 0, mu_y = 0;
      for (auto curPred : predicted)
        if (curPred.id == curObs.id) 
        {
          mu_x = curPred.x;
          mu_y = curPred.y;
          break; //exit for loop
        }
      double x_obs = curObs.x, y_obs = curObs.y;
      //set the weight using the multivariate gaussian function (normalized)
      particles[i].weight *= multiv_prob(std_landmark[0], std_landmark[1], 
                                          x_obs, y_obs, mu_x, mu_y);
    }
  }
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional 
   *   to their weight. 
   */

  vector<double> weights;
  double maxWeight = DBL_MIN;
  //find the max particle weight and store all the weights
  for(int i = 0; i < num_particles; i++) 
  {
    weights.push_back(particles[i].weight);
    if (particles[i].weight > maxWeight) 
      maxWeight = particles[i].weight;
  }

  // random distribution for beta 
  std::uniform_real_distribution<double> distReal(0.0, maxWeight);
  // random integer distribution for resampling wheel
  std::uniform_int_distribution<int> distInt(0, num_particles-1);

  // index of resample wheel
  int index = distInt(gen);

  double beta = 0.0;

  // resample wheel
  vector<Particle> newParticles;
  for(int i = 0; i < num_particles; i++) 
  {
    beta += distReal(gen) * 2.0;
    while(beta > weights[index]) 
    {
      beta -= weights[index];
      index = (index+1)%num_particles;
    }
    newParticles.push_back(particles[index]);
  }
  particles = newParticles; //saving the newParticles 
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