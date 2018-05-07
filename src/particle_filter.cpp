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
#include <float.h>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;
static default_random_engine generator(time(0));

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  num_particles = 100;

  // define normal distributions for sensor noise
  normal_distribution<double> ndX(0, std[0]);
  normal_distribution<double> ndY(0, std[1]);
  normal_distribution<double> ndTheta(0, std[2]);

  // auto generate particles with normal distribution
  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;         // particle id.
    p.weight = 1.0;   // uniform weight for initial values (maximum state of confusion)

    // state definitions with noise
    p.x = x + ndX(generator);
    p.y = y + ndY(generator);
    p.theta = theta + ndTheta(generator);

    particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  // define normal distributions for sensor noise
  normal_distribution<double> ndX(0, std_pos[0]);
  normal_distribution<double> ndY(0, std_pos[1]);
  normal_distribution<double> ndTheta(0, std_pos[2]);

  for (int i = 0; i < num_particles; i++) {

    // predict new state
    if (fabs(yaw_rate) < 0.00001) {

      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    } else {

      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    // adding noise : TODO: revisit this.
    particles[i].x += ndX(generator);
    particles[i].y += ndY(generator);
    particles[i].theta += ndTheta(generator);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

  // process association for each map observation
  for(unsigned int i = 0; i < observations.size(); i++) {
    
    LandmarkObs obs = observations[i];
    double minimum_distance = DBL_MAX;      // setting a maximum possible values for float/double

    // placeholder to associate prediction with observations
    int landMarkId = -1;
    
    for(unsigned int j = 0; j < predicted.size(); j++) {
      // for each prediction calculate distance from the car and find the landmark with minimum distance
      LandmarkObs p = predicted[j];
      double cur_dist = dist(obs.x, obs.y, p.x, p.y);
      if (cur_dist < minimum_distance) {
        // placeholder for landmark with minimum distance
        minimum_distance = cur_dist;
        landMarkId = p.id;
      }
    }

    // tie up observation with predicted landmarkId
    observations[i].id = landMarkId;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

  // for each particle...
  for (int i = 0; i < num_particles; i++) {

    // find all the landmarks (from the map coordinate system) that falls within the sensor range from (px, py).
    vector<LandmarkObs> lm_with_in_range;
    double sensorRange = pow(sensor_range, 2);
    for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

      double distX = particles[i].x - map_landmarks.landmark_list[j].x_f;   // distance in x-axis from px to landmark
      double distY = particles[i].y - map_landmarks.landmark_list[j].y_f;   // distance in y-axis from py to landmark
      int lm_id = map_landmarks.landmark_list[j].id_i;

      if((pow(distX, 2) + pow(distY, 2)) <= sensorRange) {
        // measured distance is within the sensor range.
        lm_with_in_range.push_back(LandmarkObs{ lm_id, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f });
      }
    }

    // perform transformation from vehicle coordinate system to map coordinates
    vector<LandmarkObs> transformed_lm_obs;
    for (unsigned int j = 0; j < observations.size(); j++) {
      double t_x = cos(particles[i].theta)*observations[j].x - sin(particles[i].theta)*observations[j].y + particles[i].x;
      double t_y = sin(particles[i].theta)*observations[j].x + cos(particles[i].theta)*observations[j].y + particles[i].y;
      transformed_lm_obs.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
    }

    // /////////////////////////
    // tie up predictions and observations on the current particle
    dataAssociation(lm_with_in_range, transformed_lm_obs);
    // /////////////////////////


    // reset particle weight
    particles[i].weight = 1.0;

    for(unsigned int j = 0; j < transformed_lm_obs.size(); j++) {
      
      // placeholders for observation and associated prediction coordinates
      double obs_x, obs_y, pr_x, pr_y;
      obs_x = transformed_lm_obs[j].x;
      obs_y = transformed_lm_obs[j].y;

      int associated_prediction = transformed_lm_obs[j].id;

      // get the x,y coordinates of the prediction associated with the current observation
      for(unsigned int k = 0; k < lm_with_in_range.size(); k++) {
        if (lm_with_in_range[k].id == associated_prediction) {
          pr_x = lm_with_in_range[k].x;
          pr_y = lm_with_in_range[k].y;
        }
      }

      // //////////
      // calculate weight for this observation with multivariate Gaussian
      double s_x = std_landmark[0];
      double s_y = std_landmark[1];
      double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-obs_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-obs_y,2)/(2*pow(s_y, 2))) ) );

      particles[i].weight *= obs_w;
    }
  }
}

void ParticleFilter::resample() {


  vector<Particle> new_particles;

  // get all of the current weights to form sampling wheel
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  // generate random starting point
  uniform_int_distribution<int> uniintdist(0, num_particles-1);
  auto index = uniintdist(generator);

  // maximum weights
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;

  // re-sampling
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(generator) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
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

    // added to avoid warnings during compilation.
    return particle;
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
