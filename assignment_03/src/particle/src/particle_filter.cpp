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
// #define SYSTEMATIC_RESAMPLING
// #define STRATIFIED_RESAMPLING
// #define ADAPTIVE_RESAMPLING

// Thresholds for adapative resampling (discovered by trial and error, working with sensor noise of 0.5)
const double U_MIN = 0.95;
const double U_MAX = 1;

/*
* This function initialize randomly the particles
* Input:
*  nParticles Number of particles used by the algorithm
*  min_pt Point <x,y> whose coords are the lowest among cloudReflector points
*  max_pt Point <x,y> whose coords are the greatest among cloudReflector points	 
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

void ParticleFilter::adaptiveResampling(normal_distribution<double> dist_x, normal_distribution<double> dist_y, normal_distribution<double> dist_theta, Particle& new_particle){
    new_particle.x = new_particle.x + dist_x(gen);
    new_particle.y = new_particle.y + dist_y(gen);
    new_particle.theta = new_particle.theta + dist_theta(gen);
}

void ParticleFilter::stratifiedResampling(normal_distribution<double> dist_x, normal_distribution<double> dist_y, normal_distribution<double> dist_theta){

    //build the CDF (Cumulative Distribution Function), the normalization is applied through the process
    vector<double> cumulative_weights = {weights[0]};
    for(int i = 1; i < num_particles; i++){
        cumulative_weights.push_back(weights[i] + cumulative_weights.back());
    }

    //in this algorithm the wheel is splitted in num_particles different sections, each one has the same length
    double section = 1.0 / num_particles;

    //pick a random value u belonging to the first section of the wheel
    uniform_real_distribution<double> uni_dist(0.0, section);
    double u;

    //this index keeps track of the current "step" of the CDF
    int index = 0;

    vector<Particle> new_particles;

    /*
        Stratified resampling: the wheel is divided in num_particles different sections and a random value u
        is picked for each section. This value represents the mark indicating the particle to keep.
    */
    for(int i = 0; i < num_particles; i++){
        u = i * section + uni_dist(gen);

        while(u > cumulative_weights[index]){
            index++;
        }
        
        Particle new_particle = particles[index];
        #ifdef ADAPTIVE_RESAMPLING
        adaptiveResampling(dist_x, dist_y, dist_theta, new_particle);
        #endif
        new_particles.push_back(new_particle);
    }

    particles.swap(new_particles);
}

void ParticleFilter::systematicResampling(normal_distribution<double> dist_x, normal_distribution<double> dist_y, normal_distribution<double> dist_theta){

    //build the CDF (Cumulative Distribution Function), the normalization is applied through the process
    vector<double> cumulative_weights = {weights[0]};
    for(int i = 1; i < num_particles; i++){
        cumulative_weights.push_back(weights[i] + cumulative_weights.back());
    }

    //in this algorithm the wheel is splitted in num_particles different sections, each one has the same length
    double section = 1.0 / num_particles;

    //pick a random value u belonging to the first section of the wheel
    uniform_real_distribution<double> uni_dist(0.0, section);
    double u = uni_dist(gen);

    //this index keeps track of the current "step" of the CDF
    int index = 0;

    vector<Particle> new_particles;

    /*
        Systematic resampling: the wheel is divided in num_particles different sections and a random initial value u
        is picked at the start. This value represents the mark indicating the particle to keep. At each iteration
        the value u is incremented by the length of one section, resulting in a evenly marked wheel.
    */
    for(int i = 0; i < num_particles; i++){
        while(u > cumulative_weights[index]){
            index++;
        }
        
        Particle new_particle = particles[index];
        #ifdef ADAPTIVE_RESAMPLING
        adaptiveResampling(dist_x, dist_y, dist_theta, new_particle);
        #endif
        new_particles.push_back(new_particle);

        u += section;
    }

    particles.swap(new_particles);
}

void ParticleFilter::resamplingWheel(normal_distribution<double> dist_x, normal_distribution<double> dist_y, normal_distribution<double> dist_theta){
    //get a random starting index to initalize the wheel
    uniform_int_distribution<int> dist_distribution(0,num_particles-1);
    int index = dist_distribution(gen);

    //get max weight
    double max_w = *max_element(weights.begin(), weights.end());
    // std::cout<<"max "<<max_w<<endl;

    //other useful variables' initialization
    double beta  = 0.0;
    vector<Particle> new_particles;
    uniform_real_distribution<double> uni_dist(0.0, max_w);

    /*
        Resampling wheel algorithm: the wheel spins once for each particle through 
        incrementing beta. The first slice of the wheel that has a weight greater than the
        value of beta is taken. The selected particle survives for this round.
    */
    for(int i = 0; i < num_particles; i++){
        beta = beta + uni_dist(gen) * 2;
        while(weights[index] < beta){
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        Particle new_particle = particles[index];
        #ifdef ADAPTIVE_RESAMPLING
        adaptiveResampling(dist_x, dist_y, dist_theta, new_particle);
        #endif
        new_particles.push_back(new_particle);
    }

    particles.swap(new_particles);
}

void ParticleFilter::resample(double resampling_std[]) {

    //calculate the total weight for the normalization purpose
    double weigth_sum = 0;
    for(int i = 0; i < num_particles; i++){
        weigth_sum += particles[i].weight;
    }

    //build the weights vector
    weights.clear();
    for(int i=0;i<num_particles;i++)
        weights.push_back(particles[i].weight / weigth_sum);

    #ifdef ADAPTIVE_RESAMPLING

    double square_weight_sum = 0;
    for(int i = 0; i < weights.size(); i++){
        square_weight_sum += pow(weights[i], 2);
    }
    //the closer to num_particles the better the filter is performing (effective sample size)
    double ess = 1.0 / square_weight_sum;

    //the closer 0 the better the filter is performing, values close to 1 indicate a filter degeneration
    double u = 1.0 - ess / num_particles;

    /*
        The adaptive resmapling takes effect for values of u between U_MIN and U_MAX.
        Based on this range, alpha can get value from 0 to 1. Alpha is just a multiplicator
        of the std_resample: higher values of u require an high correction and therefore a larger variance of the sampled
        particles. 
    */
    double alpha = clamp((u - U_MIN) / (U_MAX - U_MIN), 0.0, 1.0);
    
    cout<<alpha<<endl;
    
    normal_distribution<double> dist_x(0.0, resampling_std[0] * alpha);
    normal_distribution<double> dist_y(0.0, resampling_std[1] * alpha);
    normal_distribution<double> dist_theta(0.0, resampling_std[2] * alpha);
    #else
    //dummy values if adaptive resampling is not active
    normal_distribution<double> dist_x(0.0, resampling_std[0]);
    normal_distribution<double> dist_y(0.0, resampling_std[1]);
    normal_distribution<double> dist_theta(0.0, resampling_std[2]);
    #endif

    #ifdef RESAMPLING_WHEEL
        resamplingWheel(dist_x, dist_y, dist_theta);
    #elif defined SYSTEMATIC_RESAMPLING
        systematicResampling(dist_x, dist_y, dist_theta);
    #elif defined STRATIFIED_RESAMPLING
        stratifiedResampling(dist_x, dist_y, dist_theta);
    #endif
}


