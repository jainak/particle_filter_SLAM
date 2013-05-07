#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include<iostream>

//#include <random>
#include<time.h>
#include <math.h>

//include ROS header
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/image_encodings.h"
#include <stdexcept>

#include <cmath>

using namespace cv;
using namespace std;
//namespace enc = sensor_msgs::image_encodings;


using namespace sensor_msgs::image_encodings;

//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>


#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#define _USE_MATH_DEFINES

using namespace std;
using namespace cv;

double start = 0.0;

double x = start;          // inital state
double x_state_noise = 0.5;  //state estimation noise
double x_meas_noise = 0.5;   //measurement noise
double T = 100.0;    //time
double N = 10;    //no o f particles
double delta_T = 0.01;
double timeSteps = T/delta_T;

double V = 2;      //variance of initial estimate
vector<double> x_P;   //vector of particles
vector<double> x_out;   //vector of particles
vector<double> z_out;   //vector of particles
double x_est;   //vector of particles
vector<double> x_est_out;   //vector of particles

typedef boost::mt19937                     	Engine;    // Mersenne Twister
typedef boost::normal_distribution<float> 	Distribution;   // Normal Distribution
typedef boost::variate_generator<Engine,Distribution> 	Generator;    // Variate generator
Engine  eng;

class Feature{
	
	public:
	float r[3];
	float q[4];
	float u0, v0, ut, vt;
	KeyPoint data;
	geometry_msgs::Point position;
		
	void initiate(float trans[], float quat[], float U0, float V0, float Ut, float Vt, KeyPoint feat_data){
		memcpy (r, trans, sizeof (trans));
		memcpy (q, quat, sizeof (quat));
		u0 = U0;
		v0 = V0;
		ut = Ut;
		vt = Vt;
		data = feat_data;
	}
};

class Particle{
	public:
	geometry_msg::PoseStamped x;
	vector<Feature> features;
	vector<Feature> last_features;
	visualization_msgs::Marker map;
	geometry_msg::Point z_update;
	float weight;
	
	void intialize_pose(){
	}
};

geometry_msg::PoseStamped robot_pose;
vector<Particle> x_P;
// Windows names
static const char graph[] = "Graph";
/// Create black empty images
Mat plot = Mat::zeros( 720, 720, CV_8UC3 );



void initialize_measurements(){
	//distribution DIST(mean,variance)
    Distribution dist(0,x_meas_noise);
    Generator  gen(eng,dist);
	cout << "Initial particle Measurements:" << endl;
	for(int i =0; i<N; i++){
		//double pos_with_uncertainty = NormalDistribution(x, V);
		double meas_with_uncertainty = pow(x_P.at(i),2)/20 + gen();
		z_out.push_back(meas_with_uncertainty);
		cout << meas_with_uncertainty << endl;
	}
}

void initialize_particles(){
	//distribution DIST(mean,variance)
    Distribution dist(x,V);
    Generator  gen(eng,dist);
	cout << "Initial particle positions:" << endl;
	for(int i =0; i<N; i++){
		//double pos_with_uncertainty = NormalDistribution(x, V);
		double pos_with_uncertainty = gen();
		x_P.push_back(pos_with_uncertainty);
		cout << pos_with_uncertainty << endl;
	}
}

double motion_model_output(double x_t, double t){
	//distribution DIST(mean,variance)
    Distribution dist(0,x_state_noise);
    Generator  gen(eng,dist);

	double x_t_1 = 0.5*x_t + 25*x_t/(1+pow(x_t,2)) + 8*cos(1.2*(t-1)*delta_T) + gen();
	return x_t_1;
}

double measurement_model_output(double x_t_1){
	Distribution dist(0,x_state_noise);
    Generator  gen(eng,dist);
	
	double meas = pow(x_t_1,2)/20 + gen();
	return meas;
}

double predict_position_particles(double x_t, double t){
	//distribution DIST(mean,variance)
    Distribution dist(0,x_state_noise);
    Generator  gen(eng,dist);

	double x_t_1 = 0.5*x_t + 25.0*x_t/(1+pow(x_t,2)) + 8.0*cos(1.2*(t-1)*delta_T) + gen();
	return x_t_1;
}

double predict_measurement_particles(double x_t_1){

	double meas = pow(x_t_1,2)/20.0;
	return meas;
}

double assign_weight(double z, double z_update){
	double weight = (1/sqrt(2.0*M_PI*x_meas_noise)) * exp(pow(-(z-z_update),2)/(x_meas_noise));
	return weight;
}

int main(int argc, char** argv){

	namedWindow("GRAPH", CV_WINDOW_AUTOSIZE);
		
	
	initialize_particles();
	initialize_measurements();
	
	x_out.push_back(x);
	x_est = x;
	x_est_out.push_back(x);
	double actual_meas;
	
	for(int t =1; t<timeSteps; t++){
		
		//Get new position from motion model using given actions
		x = motion_model_output(x,t);
	
		//Get measurements from measurement model (actual measurements)
		actual_meas = measurement_model_output(x);
		
		cout << "State: \t" << x << "\nMeasurement: \t" << actual_meas << endl;
		
		vector<double> x_P_update;
		vector<double> z_update;
		vector<double> weight_p;
		
		//Predict position and measurements for all particles
		for(int i=0; i<N; i++){
			
			//Update position of particle based on given actions: Motion prediction model
			x_P_update.push_back(predict_position_particles(x_P.at(i), t));
			
			//Update measurements of particle using giving actions: Measurement prediction model
			z_update.push_back(predict_measurement_particles(x_P_update.at(i)));
			
			//Assign weight to every particle based on closeness of predicted measurement
			weight_p.push_back(assign_weight(actual_meas,z_update.at(i)));
		}
		
		//Normalize weights for resampling
		double w_sum = 0;
		for(int i =0; i<N; i++)
			w_sum += weight_p.at(i);
		
		vector<double> cum_weights;
		for(int i =0; i<N; i++){
			weight_p.at(i) = weight_p.at(i)/w_sum;
			if (i == 0)
				cum_weights.push_back(weight_p.at(i));
			else
				cum_weights.push_back(cum_weights.at(i-1) + weight_p.at(i));
		}
		
		//Get estimation x_est of quadrotor's local position
		
		//x_est = mean(uniform_distributed sample set)
		x_est = 0.0;
		//Uni_Dist dist(0.0,1.0);
		//Uni_Gen gen(eng, dist);
		
		//std::default_random_engine generator;
		//std::uniform_real_distribution<double> distribution(0.0,1.0);

		for(int i =0; i<N; i++){
			//double rd = Uni_Dist();
			//double rd = distribution(generator);
			double rd=((double)rand()/(double)RAND_MAX);
			for(int j =0; j<N; j++){
				if(rd > cum_weights.at(j)){
					x_P.at(i) = x_P_update.at(j);
					x_est += x_P.at(i);
					break;
				}
				else
					continue;			
			}
		}
		
		x_est /= N;
		
		//estimated position - green
		circle(plot, Point(t*delta_T*10, 360 + x_est*20 ), 1, Scalar(0,255,0), -1,8);
		//actual measurement - Blue
		circle(plot, Point(t*delta_T*10, 200 + actual_meas*20 ), 1, Scalar(255,0,0), -1,8);
		//motion model prediction - pink
		circle(plot, Point(t*delta_T*10, 360 + x*20 ), 1, Scalar(255,0,255), -1,8);
		//circle(plot, Point(t, 100.0), 5, Scalar(255,255,0), -1,8);
		
		imshow("GRAPH",plot);
		waitKey(3);
		x_out.push_back(x);
		z_out.push_back(actual_meas);
		x_est_out.push_back(x_est);
		
		cout << "X_est:\t" << x_est << endl;
		//cout << "X_actual:\t" << x << endl;
		//cout << "X_est:\t" << x_est << endl;
		
		//Plot estimation, actual and measured
		
	}
	
	
	
	return 1;	
}
