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
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/image_encodings.h"
#include <pf_drone_state_estimation/Measurement_data.h>
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


ros::Publisher map_pub;
ros::Publisher pose_pub;

geometry_msgs::PoseStamped x;	//initial pose
double x_state_noise = 0.5;  	//state estimation noise
double x_meas_noise = 0.5;   	//measurement noise
double T = 100.0;    			//time
double N = 10;    				//no o f particles
double delta_T = 0.01;			//timestep
double timeSteps = T/delta_T;	//no. of time steps

double V = 2;      				//variance of initial estimate
vector<geometry_msgs::PoseStamped> x_out;   //vector of particles positions from motion model
vector<geometry_msgs::Point> z_out;   		//vector of measurements made
geometry_msgs::PoseStamped x_est;   		//particles' estimated position 
vector<geometry_msgs::PoseStamped> x_est_out;   //vector of particles' estimated position 
visualization_msgs::Marker map_est;			//Estimated map to be plotted

typedef boost::mt19937                     	Engine;    // Mersenne Twister
typedef boost::normal_distribution<float> 	Distribution;   // Normal Distribution
typedef boost::variate_generator<Engine,Distribution> 	Generator;    // Variate generator
Engine  eng;

/**************************************
Define Feature and Particle class
**************************************/
class Feature{
	
public:
	//geometry_msgs::PoseStamped r_pose;
	//Point p0;
	
	//2D pixel point of the feture
	Point pt;
	//Sift/Surf keypoint data			
	KeyPoint data;
	//3D position of the feature		
	geometry_msgs::Point position;	
		
	void initiate(geometry_msgs::PoseStamped p, Point k0, Point k1, KeyPoint feat_data, geometry_msgs::Point pos){
		//r_pose = p;
		//p0 = k0;
		pt = k1;
		data = feat_data;
		position = pos;
	}
};

class Particle{
	
public:
	//robot pose
	geometry_msgs::PoseStamped r_pose;
	
	//list fo features last seen 	
	vector<Feature> features;			
	//vector<Feature> last_features;
	
	//every particle maintains its own map (only the best one survives)
	visualization_msgs::Marker map;
	
	//list of 2D pixels where the the last seen features are predicted to be visible		
	vector<Point> meas_update;
	
	//weight for particle assigned after every measurement prediction	
	float weight;		
	
	void initialize_pose(geometry_msgs::PoseStamped x_in){
		r_pose = x_in;
	}
	
	void set_meas(vector<Feature> f){
		features = f;
	}
};

/************************************
Define global vars for PF
************************************/
geometry_msgs::PoseStamped robot_pose;
vector<Particle> x_P;
vector<Feature> last_reading;

// Windows names
static const char graph[] = "Graph";
Mat plot = Mat::zeros( 720, 720, CV_8UC3 );


/*************************************
Initialize robot pose from extrinsic calibration
*************************************/
void initialize_robot_pose(const geometry_msgs::PoseStamped::ConstPtr& p){
	x.pose.position.x = p->pose.position.x;
	x.pose.position.y = p->pose.position.y;
	x.pose.position.z = p->pose.position.z;
	
	x.pose.orientation.x = p->pose.orientation.x;
	x.pose.orientation.y = p->pose.orientation.y;
	x.pose.orientation.z = p->pose.orientation.z;
	x.pose.orientation.w = p->pose.orientation.w;
	
}
/*
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
}*/

/*************************************
Initialize robot pose for all particles with uncertainty
***************************************/
void initialize_particles(){
	//distribution DIST(mean,variance)
    Distribution dist(0.0,V);
    Generator  gen(eng,dist);
	cout << "Initial particle positions:" << endl;
	for(int i =0; i<N; i++){
		//double pos_with_uncertainty = NormalDistribution(x, V);
		geometry_msgs::PoseStamped init_pose;
		init_pose.pose.position.x = x.pose.position.x + gen();
		init_pose.pose.position.y = x.pose.position.y + gen();
		init_pose.pose.position.z = x.pose.position.z + gen();
		
		init_pose.pose.orientation.x = x.pose.orientation.x + gen();
		init_pose.pose.orientation.y = x.pose.orientation.y + gen();
		init_pose.pose.orientation.z = x.pose.orientation.z + gen();
		init_pose.pose.orientation.w = x.pose.orientation.w + gen();
		Particle p;
		p.initialize_pose(init_pose);
		x_P.push_back(p);
	}
}

/********************************************
Callback function for motion model
********************************************/
geometry_msgs::PoseStamped motion_model_output(geometry_msgs::PoseStamped x_t, double t){
	//distribution DIST(mean,variance)
    Distribution dist(0,x_state_noise);
    Generator  gen(eng,dist);
	geometry_msgs::PoseStamped x_t_1 = x_t;
	//double x_t_1 = 0.5*x_t + 25*x_t/(1+pow(x_t,2)) + 8*cos(1.2*(t-1)*delta_T) + gen();
	x_t_1.pose.position.x = x_t_1.pose.position.x + 0.4*delta_T;
	return x_t_1;
}

/************************************
Callback function for subscribing to measurements from measurement model
*************************************/
void get_measurements(const pf_drone_state_estimation::Measurement_data::ConstPtr& data){
	vector<pf_drone_state_estimation::Feature_msg> feat = data->features;
	last_reading.clear();
	for(int i = 0; i< feat.size(); i++){
		Feature reading;
		reading.pt.x = feat.at(i).px;
		reading.pt.y = feat.at(i).py;
		
		reading.data = KeyPoint(Point(feat.at(i).f.x,feat.at(i).f.y),feat.at(i).f.size, feat.at(i).f.angle, feat.at(i).f.response, feat.at(i).f.octave, feat.at(i).f.class_id);
		
		reading.position.x = feat.at(i).posX;
		reading.position.y = feat.at(i).posY;
		reading.position.z = feat.at(i).posZ;
		
		last_reading.push_back(reading);
	}
}

double measurement_model_output(double x_t_1){
	Distribution dist(0,x_state_noise);
    Generator  gen(eng,dist);
	
	double meas = pow(x_t_1,2)/20 + gen();
	return meas;
}

/************************************
Motion prediction of all particles given the set of action at time t
*************************************/
geometry_msgs::PoseStamped predict_position_particles(Particle p, geometry_msgs::PoseStamped x, int particle_number){
	//distribution DIST(mean,variance)
    Distribution dist(0,x_state_noise);
    Generator  gen(eng,dist);

	//double x_t_1 = 0.5*x_t + 25.0*x_t/(1+pow(x_t,2)) + 8.0*cos(1.2*(t-1)*delta_T) + gen();
	geometry_msgs::PoseStamped x_t_1 = p.r_pose;
	x_t_1.pose.position.x = x_t_1.pose.position.x + 0.4*delta_T;
	
	return x_t_1;
}

/************************************
Measurement prediction of all particles given the set of action at time t
*************************************/
vector<Point> predict_measurement_particles(geometry_msgs::PoseStamped curr_pose, Particle p_update){
	vector<Point> meas;
	
	
	return meas;
}

/************************************
Assign weights to particles based on actual and predicted measurements
************************************/
double assign_weight(Particle p){
	//double weight = (1/sqrt(2.0*M_PI*x_meas_noise)) * exp(pow(-(z-z_update),2)/(x_meas_noise));
	//return weight;
	return 0.0;
}

/************************************
Estimate robot pose from resampled particles
*************************************/
geometry_msgs::PoseStamped estimate_robot_pose(vector<Particle> p){
	geometry_msgs::PoseStamped est;
	est.pose.position.x = 0;
	est.pose.position.y = 0;
	est.pose.position.z = 0;
		
	est.pose.orientation.x = 0;
	est.pose.orientation.y = 0;
	est.pose.orientation.z = 0;
	est.pose.orientation.w = 0;
	for(int i = 0; i < N; i++ ){
		est.pose.position.x += p.at(i).r_pose.pose.position.x;
		est.pose.position.y += p.at(i).r_pose.pose.position.y;
		est.pose.position.z += p.at(i).r_pose.pose.position.z;
		
		est.pose.orientation.x += p.at(i).r_pose.pose.orientation.x;
		est.pose.orientation.y += p.at(i).r_pose.pose.orientation.y;
		est.pose.orientation.z += p.at(i).r_pose.pose.orientation.z;
		est.pose.orientation.w += p.at(i).r_pose.pose.orientation.w;
	}
	
	est.pose.position.x = est.pose.position.x/N;
	est.pose.position.y = est.pose.position.y/N;
	est.pose.position.z = est.pose.position.z/N;
	
	est.pose.orientation.x = est.pose.orientation.x/N;
	est.pose.orientation.y = est.pose.orientation.y/N;
	est.pose.orientation.z = est.pose.orientation.z/N;
	est.pose.orientation.w = est.pose.orientation.w/N;
	
	return est;
}


/************************************
Particle filter
************************************/
void start_particle_filter(const std_msgs::String::ConstPtr& cmd){
  if (cmd->data.c_str() == "true"){
	
	//Initialize particles and their measurements
	initialize_particles();
	//initialize_measurements();
	for(int i=0; i<N; i++){
		x_P.at(i).features = last_reading;
		//x_P.at(i).last_features = last_reading;
	}
	
	//Put current pose into database
	x_out.push_back(x);
	x_est = x;
	x_est_out.push_back(x);
	vector<Feature> actual_meas;
	
	for(int t =1; t<timeSteps; t++){
		
		//Get new position from motion model using given actions
		// I think motion model and motion prediction model are both same!! we don't need motion motion... only motion prediction model should be enough
		x = motion_model_output(x,t);
	
		//Get measurements from measurement model (actual measurements)
		actual_meas = last_reading;
		
		vector<Particle> particle_update;	//predicted particles
		//vector<Point> meas_update;
		vector<double> weight_p;
		
		//Predict position and measurements for all particles
		for(int i=0; i<N; i++){
			particle_update.push_back(x_P.at(i));
			
			//Update position of particle based on given actions: Motion prediction model
			particle_update.at(i).r_pose = predict_position_particles(x_P.at(i), x, i);
			particle_update.at(i).features = actual_meas;
			
			//Update measurements of particle using giving actions: Measurement prediction model
			particle_update.at(i).meas_update = predict_measurement_particles(x, particle_update.at(i));
			
			//Assign weight to every particle based on closeness of predicted measurement
			particle_update.at(i).weight = assign_weight(particle_update.at(i));
		}
		
		//Normalize weights for resampling
		double w_sum = 0;
		for(int i =0; i<N; i++)
			w_sum += particle_update.at(i).weight;
		
		//Calculate cumulative weights
		vector<double> cum_weights;
		for(int i =0; i<N; i++){
			particle_update.at(i).weight = particle_update.at(i).weight/w_sum;
			if (i == 0)
				cum_weights.push_back(particle_update.at(i).weight);
			else
				cum_weights.push_back(cum_weights.at(i-1) + particle_update.at(i).weight);
		}
		
		//Get estimation x_est of quadrotor's local position
		
		//x_est = mean(uniform_distributed sample set)
		//x_est = x;
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
					x_P.at(i) = particle_update.at(j);
					break;
				}
				else
					continue;			
			}
		}
		
		//Get robot estimated position
		x_est = estimate_robot_pose(x_P);
		pose_pub.publish(x_est);
		//Update world map for all particles
		for(int i=0; i<N; i++){
			for(int j=0; j<x_P.at(i).features.size(); j++){
				x_P.at(i).map.points.push_back(x_P.at(i).features.at(j).position); 
			}
		}
		map_pub.publish(x_P.at(0).map);
		
		//x_est += x_P.at(i);
		//x_est /= N;
		
		//estimated position - green
		//circle(plot, Point(t*delta_T*10, 360 + x_est*20 ), 1, Scalar(0,255,0), -1,8);
		//actual measurement - Blue
		//circle(plot, Point(t*delta_T*10, 200 + actual_meas*20 ), 1, Scalar(255,0,0), -1,8);
		//motion model prediction - pink
		//circle(plot, Point(t*delta_T*10, 360 + x*20 ), 1, Scalar(255,0,255), -1,8);
		//circle(plot, Point(t, 100.0), 5, Scalar(255,255,0), -1,8);
		
		//imshow("GRAPH",plot);
		//waitKey(3);
		x_out.push_back(x);
		for(int a = 0; a < actual_meas.size(); a++)
			z_out.push_back(actual_meas.at(a).position);
		x_est_out.push_back(x_est);
		
		//cout << "X_est:\t" << x_est << endl;
		//cout << "X_actual:\t" << x << endl;
		//cout << "X_est:\t" << x_est << endl;
		
		//Plot estimation, actual and measured
		
	}
  }else{
  	ROS_INFO("System not initialized yet. Publish correct command on topic /sys_init.");
  }
}

int main(int argc, char** argv){

	ros::init(argc, argv, "particle_filter");
	ros::NodeHandle nh;
	
	ROS_INFO("Start Particle filter...");
	//namedWindow("GRAPH", CV_WINDOW_AUTOSIZE);
		
	map_pub = nh.advertise<visualization_msgs::Marker>("/points_map",1);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose_est",1);
	
	ros::Subscriber init_command = nh.subscribe("/sys_init", 1000, start_particle_filter);
	ros::Subscriber init_pose = nh.subscribe("/init_pose", 1000, initialize_robot_pose);
	ros::Subscriber get_meas = nh.subscribe("/init_pose", 1000, get_measurements);
	//start_particle_filter();
	
	//destroyWindow("GRAPH");
	ros::spin();
	//ROS_INFO is the replacement for printf/cout.
	ROS_INFO("particle_filter::main.cpp::No error.");
	
	return 1;	
}
