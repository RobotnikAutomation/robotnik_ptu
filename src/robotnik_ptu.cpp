/** \file robotnik_ptu.cpp
 * \author Robotnik Automation S.L.L.
 * \version 1.0
 * \date    2015
 *
 * \brief robotnik_ptu ros node
 * Component to manage a dynamixel servo pan-tilt unit
 * (C) 2015 Robotnik Automation, SLL
 * uses dynamixel ros package 
 * With some changes this node could be generalized to any kinematic chain, 
 * as was done in robotnik_dpro_controller
 */
 
#include <string.h>
#include <vector>
#include <stdint.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <cstdlib>
#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/MotorState.h>
#include <sensor_msgs/JointState.h>

#include <std_msgs/Float64.h>

#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#define	 ROBOTNIK_PTU_MIN_COMMAND_REC_FREQ		1.0
#define	 ROBOTNIK_PTU_MAX_COMMAND_REC_FREQ		20.0

// Servo Dynamixel MX-64
// 0º: 2048 -90º: 1024 +90º: 3072     180º / 2048
// Servo Dynamixel MX-28 
// TODO TBC
#define  TOLERANCE_RAD      0.03           // Accuracy for state machine transitions
#define  PAN_ANGLE_MAX_RAD  3.13           // Pan angle +
#define  PAN_ANGLE_MIN_RAD  -3.13          // Pan angle - 
#define  TILT_ANGLE_MAX_RAD  1.5708        // Pan angle +
#define  TILT_ANGLE_MIN_RAD  -1.5708       // Pan angle - 


using namespace std;

class robotnik_ptu_node
{

public:
	self_test::TestRunner self_test_;
	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	int error_count_;
	int slow_count_;
	double desired_freq_;
	diagnostic_updater::Updater diagnostic_;						// General status diagnostic updater
	diagnostic_updater::FrequencyStatus freq_diag_;		         	// Component frequency diagnostics
        diagnostic_updater::HeaderlessTopicDiagnostic *subs_command_freq; // Topic reception frequency diagnostics
	ros::Time last_command_time;									// Last moment when the component received a command
        diagnostic_updater::FunctionDiagnosticTask command_freq_;

	// Current axis position in rad
	double position_pan_rad_;
	double position_tilt_rad_;	
	dynamixel_msgs::MotorState motor_state_pan_;
	dynamixel_msgs::MotorState motor_state_tilt_;
	
	// Subscriber
	ros::Subscriber read_pos_sub_;      // motor states
	ros::Subscriber joint_command_subscriber_; // commands received 
	
	// Publisher
	ros::Publisher cmd_pan_pos_pub_;	// command to pan controller
	ros::Publisher cmd_tilt_pos_pub_;   // command to tilt controller
	ros::Publisher joint_states_publisher_; // joint states 
    std::vector< std::string > joint_names_; // joint names for publishing the joint_states
	
	//! Node running
	bool running;	

	// Error counters and flags
	std::string was_slow_;
	std::string error_status_;
	
	//! Mutex for controlling the changes and access to 
	// pthread_mutex_t mutex_position;

/*!	\fn robotnik_ptu_node::robotnik_ptu_node()
 * 	\brief Public constructor
*/
robotnik_ptu_node(ros::NodeHandle h) : self_test_(), diagnostic_(),
  node_handle_(h), private_node_handle_("~"), 
  error_count_(0),
  slow_count_(0),
  desired_freq_(50),   // not higher than the laser scanner frecuency
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05)   ),
  command_freq_("Command frequency check", boost::bind(&robotnik_ptu_node::check_command_subscriber, this, _1))
{
    running = false;
    ros::NodeHandle robotnik_ptu_node_handle(node_handle_, "robotnik_ptu_node");

    // Subcscribing
	read_pos_sub_ = robotnik_ptu_node_handle.subscribe<dynamixel_msgs::MotorStateList>("/motor_states/pan_tilt_port", 1, 
													  &robotnik_ptu_node::MotorStateCallback, this ); 
    joint_command_subscriber_ = robotnik_ptu_node_handle.subscribe("in/joint_commands", 1,
                                                      &robotnik_ptu_node::jointCommandCallback, this);

    // Publishing
    cmd_pan_pos_pub_ = private_node_handle_.advertise<std_msgs::Float64>("/pan_controller/command", 10 ); 
    cmd_tilt_pos_pub_ = private_node_handle_.advertise<std_msgs::Float64>("/tilt_controller/command", 10 );     
    joint_states_publisher_ = robotnik_ptu_node_handle.advertise<sensor_msgs::JointState>("/joint_states", 2);

    // Self test
    self_test_.add("Connect Test", this, &robotnik_ptu_node::ConnectTest);

    // Component frequency diagnostics
    diagnostic_.setHardwareID("robotnik_ptu_node");
    diagnostic_.add("Motor Controller", this, &robotnik_ptu_node::controller_diagnostic);
    diagnostic_.add( freq_diag_ );
    diagnostic_.add( command_freq_ );
    
    position_pan_rad_ = 0.0;
	position_tilt_rad_= 0.0;

    joint_names_.resize(2);
    joint_names_[0] = "ptu1_joint_1"; //node_name_ + "_joint_1";
    joint_names_[1] = "ptu1_joint_2"; // node_name_ + "_joint_2";

 
    ROS_INFO("Desired freq %5.2f", desired_freq_);

}

/*!	\fn robotnik_ptu_node::~robotnik_ptu_node()
 * 	\brief Public destructor
*/
~robotnik_ptu_node(){
}

/*!	\fn int robotnik_ptu_node::set_pos_rad()
 * 	\brief Publish new setpoint for a dynamixel servo configured in position mode
*/
int set_pos_rad( double radpos_pan, double radpos_tilt )
{
	// Current time
	ros::Time current_time = ros::Time::now();
	
	// Create msg
	std_msgs::Float64 float64_msg;
	
	// Publish the pan message
	double refrad = radpos_pan;
	if (refrad > PAN_ANGLE_MAX_RAD) refrad = PAN_ANGLE_MAX_RAD;
	if (refrad < PAN_ANGLE_MIN_RAD) refrad = PAN_ANGLE_MIN_RAD;
	float64_msg.data = refrad;		
	cmd_pan_pos_pub_.publish(float64_msg);	
	
	// Publish the tilt message
	refrad = radpos_tilt;
	if (refrad > TILT_ANGLE_MAX_RAD) refrad = TILT_ANGLE_MAX_RAD;
	if (refrad < TILT_ANGLE_MIN_RAD) refrad = TILT_ANGLE_MIN_RAD;
	float64_msg.data = refrad;
	cmd_tilt_pos_pub_.publish(float64_msg);
	
	// ROS_INFO("SET_POS_RAD : %5.2f ", refrad);
}

/*!	\fn int robotnik_ptu_node::start()
 * 	\brief Start Controller
*/
int start(){

	set_pos_rad( 0.0, 0.0 );
	
    ROS_INFO("robotnik_ptu: homing");
	usleep(15000);
	
 	// TODO - refer node name
 	ROS_INFO("Robotnik PTU controller started");
	freq_diag_.clear();
	running = true;
	return 0;
}

/*!	\fn robotnik_ptu_node::stop()
 * 	\brief Stop Controller
*/
int stop(){

    ROS_INFO("Stopping driver");
	running = false;
    return 0;
}

// Callbacks
/*!     \fn void robotnik_ptu_node::MotorStateCallback(const dynamixel_msgs::MotorStateList::ConstPtr& motor_state_list)
        * Callback - motor state list
*/
void MotorStateCallback(const dynamixel_msgs::MotorStateList::ConstPtr& motor_state_list)
{
    last_command_time = ros::Time::now();
	dynamixel_msgs::MotorState motor_state;
	
	int32_t id, goal, position;
	
	// TODO - Check conversions according to servo
	motor_state_pan_ = motor_state_list->motor_states[0];	
	position = motor_state_pan_.position;
	position_pan_rad_ = (((double) position - 2048.0) / 1024.0) * 1.570796327;   
	
	motor_state_tilt_ = motor_state_list->motor_states[1];	
	position = motor_state_tilt_.position;
	position_tilt_rad_ = (((double) position - 2048.0) / 1024.0) * 1.570796327;   	
  	
}

/*!     \fn void robotnik_ptu::jointCommandCallback(const dynamixel_msgs::MotorStateList::ConstPtr& motor_state_list)
        * Callback - motor state list
*/
void jointCommandCallback(const sensor_msgs::JointStateConstPtr& joint_cmd)
{
    // sensor_msgs::JointState joint_commands = *joint_cmd;     
    bool has_pos;
       
    // Check that position references have arrived
    if (joint_cmd->position.size() > 0) 
		has_pos = true;
    if (!has_pos && (joint_cmd->velocity.size() > 0))
        ROS_ERROR("PTU Cannot be controlled in speed mode");
    if (!has_pos && (joint_cmd->effort.size() > 0))
        ROS_ERROR("PTU Dynamixel servos do not accept torque references");

	// Set commanded positions 
	if (has_pos) {
	   set_pos_rad( joint_cmd->position[0], joint_cmd->position[1] );
       }	
}

/*!	\fn robotnik_ptu_node::ConnectTest()
 * 	\brief Test to connect to Motors
*/
void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
{
   // connection test
   // TBC
   status.summary(0, "Connected successfully.");
}

/*
 *\brief Checks the status of the driver Diagnostics
 *
 */
void controller_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	// add and addf are used to append key-value pairs.
	//stat.addf("Controller StatusWord (HEX)", "%x", sw ); // Internal controller status
	//stat.add("Current velocity", driver->readVelocity() );

	if (motor_state_pan_.error != 0) stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Pan motor on error");
	else stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Pan Motor: OK");
	stat.add("Pan id         :", motor_state_pan_.id );
	stat.add("Pan goal       :", motor_state_pan_.goal );
	stat.add("Pan position   :", motor_state_pan_.position);
	stat.add("Pan error      :", motor_state_pan_.error);
	stat.add("Pan speed      :", motor_state_pan_.speed);
	stat.add("Pan load       :", motor_state_pan_.load);
	stat.add("Pan voltage    :", motor_state_pan_.voltage);
	stat.add("Pan temperature:", motor_state_pan_.temperature);
	stat.add("Pan moving     :", motor_state_pan_.moving);
	
	if (motor_state_tilt_.error != 0) stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Tilt motor on error");
	else stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Tilt Motor: OK");
	stat.add("Tilt id         :", motor_state_tilt_.id );
	stat.add("Tilt goal       :", motor_state_tilt_.goal );
	stat.add("Tilt position   :", motor_state_tilt_.position);
	stat.add("Tilt error      :", motor_state_tilt_.error);
	stat.add("Tilt speed      :", motor_state_tilt_.speed);
	stat.add("Tilt load       :", motor_state_tilt_.load);
	stat.add("Tilt voltage    :", motor_state_tilt_.voltage);
	stat.add("Tilt temperature:", motor_state_tilt_.temperature);
	stat.add("Tilt moving     :", motor_state_tilt_.moving);

}


/*
 *	\brief Checks that the robot is receiving at a correct frequency the command messages. Diagnostics
 *
 */
void check_command_subscriber(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	ros::Time current_time = ros::Time::now();
/*
	double diff = (current_time - last_command_time).toSec();
	if(diff > 1.0){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Topic is not receiving commands");
		//ROS_INFO("check_command_subscriber: %lf seconds without commands", diff);
	 	// if (driver) driver->SetDesiredSpeed(0.0, 0.0);
	}else{
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");
	}
*/
}

////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS
int read_and_publish()
  {
	  
	static double prevtime = 0;

	double starttime = ros::Time::now().toSec();

	if (prevtime && prevtime - starttime > 0.05)
	{
		ROS_WARN("Full robotnik_ptu_node loop took %f ms. Nominal is 10ms.", 1000 * (prevtime - starttime));
		was_slow_ = "Full robotnik_ptu_node loop was slow.";
		slow_count_++;
	}

	
	
	// Publish joint states
    sensor_msgs::JointState joint_state;
    joint_state.name = joint_names_;
    joint_state.header.stamp = ros::Time::now();

    // Joint positions
    joint_state.position.resize(2);
    joint_state.position[0] = position_pan_rad_;
    joint_state.position[1] = position_tilt_rad_;

    // Joint velocities
    joint_state.velocity.resize(2);
    joint_state.velocity[0] = 0.0;   // could be converted from motor_state speed (supposed grad/s) 
    joint_state.velocity[1] = 0.0;   

    // Joint torques
    joint_state.effort.resize(2);
    joint_state.effort[0] = 0.0;	// 
    joint_state.effort[1] = 0.0;

	joint_states_publisher_.publish( joint_state );


    //ROS_INFO("position_rad_=%5.2f", position_rad_);
	double endtime = ros::Time::now().toSec();

	if (endtime - starttime > 0.05)
	{
		ROS_WARN("Gathering data took %f ms. Nominal is 10ms.", 1000 * (endtime - starttime));
		was_slow_ = "Full robotnik_ptu_node loop was slow.";
		slow_count_++;
	}

	prevtime = starttime;
	starttime = ros::Time::now().toSec();
	ros::Time current_time = ros::Time::now();		

	freq_diag_.tick();
		
	return(0);
}

bool spin()
{
    ros::Rate r(desired_freq_);  // 50.0 

    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
      if (start() == 0)
      {
		ROS_INFO("start==0");
        //while(ros::ok() && node_handle_.ok()) {
		while(ros::ok()) {
          if(read_and_publish() < 0)
            break;
          self_test_.checkTest();
          diagnostic_.update();
          ros::spinOnce();
	      r.sleep();
          }
	      ROS_INFO("robotnik_ptu_node::spin - END OF ros::ok() !!!");
        } 
        else {
	    // No need for diagnostic here since a broadcast occurs in start
        // when there is an error.
        usleep(1000000);
        self_test_.checkTest();
        ros::spinOnce();
        }
      
   }

   return true;
}

}; // class robotnik_ptu_node

// MAIN
int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotnik_ptu_node");
	
	ros::NodeHandle n;		
  	robotnik_ptu_node rptu(n);

  	rptu.spin();

	return (0);
}
// EOF
