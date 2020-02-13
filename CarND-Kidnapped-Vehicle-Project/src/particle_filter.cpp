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

#define THESHOLD_MIN_VALUE 0.00001

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  
  //gen is the random engine initializer
  std::default_random_engine gen1;

  num_particles = 80;  // TODO: Set the number of particles
  particles.resize(num_particles) ;

  //Create normal distribution for x,y and theta using the gps estimate
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  
  //Take GPS data as the rough initial estimate for all the particles
  //Initialize all the particles
  vector<Particle> ran_particles ;
  for (int i = 0; i < num_particles; ++i) 
  {
    double init_x, init_y, init_theta; 
    init_x = dist_x(gen1);
    init_y = dist_y(gen1);
    init_theta = dist_theta(gen1);
    Particle particle;  // Uses the struct defined
    particle.id = i;
    particle.x = init_x;
    particle.y = init_y;
    particle.theta = init_theta;
    particle.weight = 1.0/num_particles;  //Initialize all the particles with weigth 1.0/number of particles
    ran_particles.push_back(particle);  // Append the particle which has been initialized
  }
  particles = ran_particles ;
  //Now that we have initialized all the particles, update the initailzed flag.
  is_initialized = true ;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  //gen is the random engine initializer
  std::default_random_engine gen2;

  //Create normal distribution for x,y and theta to incorporrate uncertainity/noise
  std::normal_distribution<double> dist_x(0, std_pos[0]);
  std::normal_distribution<double> dist_y(0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0, std_pos[2]);

  double noise_x , noise_y, noise_theta;  //variable to capture noise in measurments
  noise_x = dist_x(gen2);
  noise_y = dist_y(gen2);
  noise_theta = dist_theta(gen2);

  // Calculate new state.
  for (int i = 0; i < num_particles; ++i) {

      //double theta = particles[i].theta;

      // 2 set of equations, one when yaw rate is 0 and other one when yaw rate is not equal to 0
      if ( fabs(yaw_rate) < 0.00001 ) {  //Here no need to update yaw rate , when its 0
        particles[i].x += (velocity * delta_t) * (cos( particles[i].theta ))  ;
        particles[i].y += (velocity * delta_t) * (sin( particles[i].theta ))  ;
        
      } else {
        particles[i].x += (velocity / yaw_rate) * ( sin( particles[i].theta + yaw_rate * delta_t ) - sin( particles[i].theta ) )  ;
        particles[i].y += (velocity / yaw_rate) * ( cos( particles[i].theta ) - cos( particles[i].theta + yaw_rate * delta_t ) ) ;
        particles[i].theta += (yaw_rate * delta_t) ;
      }
      // adding noise
      particles[i].x += dist_x(gen2);
      particles[i].y += dist_y(gen2);
      particles[i].theta += dist_theta(gen2);

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

  //Iterate through all the observations
  for(unsigned i = 0 ; i < observations.size() ; ++i){  
    double closest_dist ; //placeholder for tracking the closest distance during interation
    int closest_id ; // placeholder for tracking the particle id with the closest distance
    int first_step ;  // This is to help in initialization of above values for each observations
    first_step = 0 ;
    for (unsigned j = 0 ; j < predicted.size() ; ++j ){
      //Calculate the distance
      double obs_pred_dist ;
      obs_pred_dist = std::pow((observations[i].x - predicted[j].x),2) + std::pow((observations[i].y - predicted[j].y),2);
      if (first_step == 0 ){
        closest_dist = obs_pred_dist;
        closest_id = predicted[j].id;
        first_step = 99 ;
      }
      if(obs_pred_dist < closest_dist){
        closest_dist = obs_pred_dist;
        closest_id = predicted[j].id;
      }

    }
    //Now associate the observation id with the id associated with closest distance
    observations[i].id = closest_id;

  }

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

  //The steps need to be repeated for all the particles
  for ( int i=0 ; i<num_particles ; ++i ) {   
    
    //Vector holding the landmarks, which fall withing the sensor range
    vector<LandmarkObs> landmarks_in_range ;
    double sensor_range_squared ;
    sensor_range_squared = std::pow(sensor_range,2);
    double lm_dist;  // variable to hold the distance between landmark and sensor

    //For each particle get the pose
    double part_x = particles[i].x;
    double part_y = particles[i].y;
    double part_theta = particles[i].theta;
    
    
    //Go through each landmark in the map
    for(unsigned int j = 0 ; j < map_landmarks.landmark_list.size() ; ++j) {
      float landmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;
      int landmark_id = map_landmarks.landmark_list[j].id_i;

      lm_dist = std::pow((part_x - landmark_x),2) + std::pow((part_y - landmark_y),2);
      if(lm_dist <= sensor_range_squared){
        landmarks_in_range.push_back(LandmarkObs{landmark_id,landmark_x,landmark_y});
      }
    }
    
    //Transform the observations from car coordinate system to map coordinate system in map frame
    //Homogenous transformation matrix is used for this purpose
    //The following formulaes are result of matrix multiplication of Homogeneous transformation
    vector<LandmarkObs> car_to_map_obs;
    for(unsigned int k = 0; k < observations.size() ; ++k){
      double x_map = std::cos(part_theta) * observations[k].x - std::sin(part_theta) * observations[k].y +part_x;
      double y_map = std::sin(part_theta) * observations[k].x + std::cos(part_theta) * observations[k].y +part_y;
      car_to_map_obs.push_back(LandmarkObs{observations[k].id,x_map,y_map});
      
    }
    // Associate appropriate observation to a landmark
    dataAssociation(landmarks_in_range, car_to_map_obs);   

    particles[i].weight = 1.0 ; //Initialize the weight
    for (unsigned k=0 ; k < car_to_map_obs.size() ; ++k ) {
      double sig_x = std_landmark[0];
      double sig_y = std_landmark[1];
      double x_obs = car_to_map_obs[k].x ;
      double y_obs = car_to_map_obs[k].y ;
      double mu_x ;
      double mu_y;
      
      for (unsigned m=0 ; m < landmarks_in_range.size() ; ++m){
        if(landmarks_in_range[m].id == car_to_map_obs[k].id) {
          mu_x = landmarks_in_range[m].x;
          mu_y = landmarks_in_range[m].y;
        }
      }
      // calculate normalization term
      double gauss_norm;
      gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

      // calculate exponent
      double exponent;
      exponent = (std::pow(x_obs - mu_x, 2) / (2 * std::pow(sig_x, 2))) + (std::pow(y_obs - mu_y, 2) / (2 * std::pow(sig_y, 2)));

      // calculate weight using normalization terms and exponent
      double weight;
      weight = gauss_norm * std::exp(-exponent);

      //Weight assignment based on the check of 0 weight
      if (weight == 0) {
        particles[i].weight *= 0.00001;
      } else {
        particles[i].weight *= weight;
      }

    }

  }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */


  //Wheel method would be used for resampling
  //Make a vector is weigths and also identify the max weigth
  double mw = std::numeric_limits<double>::min();
  vector<double> wts;
  for(int n=0 ; n<num_particles ; ++n){
    wts.push_back(particles[n].weight);
    if( particles[n].weight > mw){
      mw = particles[n].weight;
    }
  }

  //gen is the random engine initializer
  std::default_random_engine gen3;
  //Create uniform distribution of indexes and pick up an initial index value
  std::uniform_int_distribution<int> index_dist(0,num_particles-1);
  int index = index_dist(gen3); //Get initial index value

  //Create unifrom distribution of weigths between 0 to max
  std::uniform_real_distribution<double> wt_dist(0,mw);

  double beta = 0.0 ; //Initialize to 0

  std::vector<Particle> resampled_list ;

  for ( int j = 0 ; j < num_particles; ++j){
    beta += wt_dist(gen3) * 2.0 * mw ; 
    while(beta > wts[index]){
      beta -= wts[index];
      index = (index+1) % num_particles;
    }
    resampled_list.push_back(particles[index]);
  }
  particles =  resampled_list ; // Assign the resampled particles to particles

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