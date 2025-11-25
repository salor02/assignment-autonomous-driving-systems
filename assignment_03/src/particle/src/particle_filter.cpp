#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "particle/particle_filter.h"
using namespace std;

static  default_random_engine gen;

#define RESAMPLING_WHEEL

/*
* TODO
* This function initialize randomly the particles
* Input:
*  std - noise that might be added to the position
*  nParticles - number of particles
*/
void ParticleFilter::init_random(int nParticles, std::pair<float, float> min_pt, std::pair<float, float> max_pt) {
    num_particles = nParticles;
    uniform_real_distribution<double> dist_x(min_pt.first, max_pt.first);
    uniform_real_distribution<double> dist_y(min_pt.second, max_pt.second);
    normal_distribution<double> dist_theta(0, 2 * M_PI);

    for(int i=0; i < num_particles; i++){
        particles.push_back(Particle(dist_x(gen), dist_y(gen), dist_theta(gen))); 
    }
    is_initialized=true;
}

/*
* This function initialize the particles using an initial guess
* Input:
*  x,y,theta - position and orientation
*  std - noise that might be added to the position
*  nParticles - number of particles
*/ 
void ParticleFilter::init(double x, double y, double theta, double std[],int nParticles) {
    num_particles = nParticles;

    //random value between [-noise.x,+noise.x]
    /*
        BUGFIX: The original code contained a misuse of the normal distributions, which were initialized
        with a mean of -std[N] and a std of std[N], but this didn't make sense
    */
    normal_distribution<double> dist_x(0, std[0]); 
    normal_distribution<double> dist_y(0, std[1]);
    normal_distribution<double> dist_theta(0, std[2]);

	//creates all the particles, each one with <x,y,theta> randomly generated in the defined distribution
    for(int i = 0; i < num_particles; i++){
        particles.push_back(Particle(x + dist_x(gen), y + dist_y(gen), theta + dist_theta(gen)));
    }
    
    is_initialized=true;
}

/*
* The predict phase uses the state estimate from the previous timestep to produce an estimate of the state at the current timestep
* Input:
*  delta_t  - time elapsed beetween measurements
*  std_pos  - noise that might be added to the position
*  velocity - velocity of the vehicle
*  yaw_rate - current orientation
* Output:
*  Updated x,y,theta position
*/
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

    /*
        The following lines perform different computations based on the particle's yaw rate:
        - a null yaw rate means that the particle is moving in a straight line
        - a yaw rate different from zero means that the particle is changing its theta as well
    */
    for(Particle& particle : particles){
        double x,y,theta;
        if (fabs(yaw_rate) < 0.00001) {
            //straight movement
            x = particle.x + velocity * delta_t * cos(particle.theta);
            y = particle.y + velocity * delta_t * sin(particle.theta);
            theta = particle.theta;
        }else{ 
            //non-straight movement
            x = particle.x + (velocity/yaw_rate) * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta));
            y = particle.y + (velocity/yaw_rate) * (cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t));
            theta = particle.theta + yaw_rate * delta_t;
        }
           
        normal_distribution<double> dist_x(0, std_pos[0]);
        normal_distribution<double> dist_y(0, std_pos[1]);
        normal_distribution<double> dist_theta(0, std_pos[2]);

        //adds the computed noise to the current particles position (x,y,theta), this noise represents the prediction phase's noise
        particle.x = x + dist_x(gen);
        particle.y = y + dist_y(gen);
        particle.theta = theta + dist_theta(gen);
	}
}

/*
* This function associates the landmarks from the MAP to the landmarks from the OBSERVATIONS
* Input:
*  mapLandmark   - landmarks of the map
*  observations  - observations of the car
* Output:
*  Associated observations to mapLandmarks (perform the association using the ids)
*/
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> mapLandmark, std::vector<LandmarkObs>& observations) {
    
    //assigns to each particle's observation the id of the nearest landmark, based on euclidean distance
    for(LandmarkObs& obs : observations){
        double min_dist = std::numeric_limits<double>::max();
        int min_id = -1;
        for(LandmarkObs landmark : mapLandmark){
            double distance = dist(landmark.x, landmark.y, obs.x, obs.y);
            if(distance < min_dist){
                min_dist = distance;
                min_id = landmark.id;
            }
        }
        obs.id = min_id;
    }
}

/*
* This function transform a local (vehicle) observation into a global (map) coordinates
* Input:
*  observation   - A single landmark observation
*  p             - A single particle
* Output:
*  local         - transformation of the observation from local coordinates to global
*/
LandmarkObs transformation(LandmarkObs observation, Particle p){
    LandmarkObs global;
    
    /*
        The following lines take an observation produced by the vehicle (local observation) and transform
        it into an observation produced by the particle (global observation). Keep in mind that an observation is just a tuple composed
        of <x,y,id> data, representing the position of a reflector wrt the observer (i.e. the vehicle that shall be localized). 
        This function transforms the coordinates coming from the vehicle (which has its own reference system), in coordinates
        valid for the global reference system (i.e. the one whose particles "belong" to) 
    */
    global.id = -1;
    global.x = observation.x * cos(p.theta) - observation.y * sin(p.theta) + p.x;
    global.y = observation.x * sin(p.theta) + observation.y * cos(p.theta) + p.y;

    return global;
}

/*
* This function updates the weights of each particle
* Input:
*  std_landmark   - Sensor noise
*  observations   - Sensor measurements
*  map_landmarks  - Map with the landmarks
* Output:
*  Updated particle's weight (particles[i].weight *= w)
*/
void ParticleFilter::updateWeights(double std_landmark[], 
		std::vector<LandmarkObs> observations, std::vector<LandmarkObs> mapLandmark) {

    for(int i=0;i<particles.size();i++){

        /*
            The following lines transorm local observation in global observation (vehicle's POV -> particle's POV)
            and associate each observation to a landmark
        */
        std::vector<LandmarkObs> transformed_observations;
        for(LandmarkObs local_obs : observations){
            transformed_observations.push_back(transformation(local_obs, particles[i]));
        }
        dataAssociation(mapLandmark, transformed_observations);

        /*
            The following lines compute the weigth of the considered particle based on the distance between
            the transformed observation and the real location of landamrk (which is known): the more
            the distance the less the probability of that particle representing the correct position of the vehicle 
        */
        particles[i].weight = 1.0;
        for(int k=0;k<transformed_observations.size();k++){
            double obs_x,obs_y,l_x,l_y;
            obs_x = transformed_observations[k].x;
            obs_y = transformed_observations[k].y;

            //get the associated landmark 
            for (int p = 0; p < mapLandmark.size(); p++) {
                if (transformed_observations[k].id == mapLandmark[p].id) {
                    l_x = mapLandmark[p].x;
                    l_y = mapLandmark[p].y;
                }
            }	
			//computing weight based on this observation. An high distance represents a low weight
            double w = exp( -( pow(l_x-obs_x,2)/(2*pow(std_landmark[0],2)) + pow(l_y-obs_y,2)/(2*pow(std_landmark[1],2)) ) ) / ( 2*M_PI*std_landmark[0]*std_landmark[1] );
            particles[i].weight *= w;
        }
    }    
}

/*
* This function resamples the set of particles by repopulating the particles using the weight as metric
*/
void ParticleFilter::resample() {
    
    uniform_int_distribution<int> dist_distribution(0,num_particles-1);
    double beta  = 0.0;
    vector<double> weights;
    int index = dist_distribution(gen);
    vector<Particle> new_particles;

    for(int i=0;i<num_particles;i++)
        weights.push_back(particles[i].weight);
																
    float max_w = *max_element(weights.begin(), weights.end());
    uniform_real_distribution<double> uni_dist(0.0, max_w);

    #ifdef RESAMPLING_WHEEL
    for(int i = 0; i < num_particles; i++){
        beta = beta + uni_dist(gen) * 2;
        while(weights[index] < beta){
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }
    #endif

    particles.swap(new_particles);
}


