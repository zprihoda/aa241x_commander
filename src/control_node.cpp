/**
 * this node will contain a skeleton for students for the control node.
 *
 * this is still in development!
 */


#include <ros/ros.h>


#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <aa241x_mission/SensorMeasurement.h>



class ControlNode {

public:

	ControlNode(float flight_alt);

	int run();


private:


	// node handler
	ros::NodeHandle _nh;

	// TODO: add any settings, etc, here
	float _flight_alt = 20.0f;		// desired flight altitude [m] AGL (above takeoff)

	// data
	mavros_msgs::State _current_state;
	geometry_msgs::PoseStamped _current_local_pos;

	// subscribers
	ros::Subscriber _state_sub;
	ros::Subscriber _local_pos_sub;
	ros::Subscriber _beacon_meas_sub;	// measurement to the beacons (or whatever the meas will be)
	ros::Subscriber _ap_range_sub;		// measurement from the imaging node
	// TODO: add subscribers here

	// publishers
	ros::Publisher _cmd_pub;
	// TODO: recommend adding publishers for data you might want to log

	// callbacks
	void stateCallback(const mavros_msgs::State::ConstPtr& msg);
	void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void beaconMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg);  // NOTE: this might not even be wanted here by students
	//void apRangeCallback(const aa241x_student::APRange::ConstPtr& msg);
	// TODO: add callbacks here

	// helper functions
	void waitForFCUConnection();


};


ControlNode::ControlNode(float flight_alt) :
_flight_alt(flight_alt)
{


	// subscribe to the desired topics
	_state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &ControlNode::stateCallback, this);
	_local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &ControlNode::localPosCallback, this);
	_beacon_meas_sub = _nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &ControlNode::beaconMeasCallback, this);
	//_ap_range_sub = _nh.subscribe<aa241x_student::APRange>("ap_range", 10, &ControlNode::apRangeCallback, this);

	// publisher
	_cmd_pub = _nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

}

void ControlNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
	_current_state = *msg;
	ROS_INFO("got state message: %s", _current_state.mode.c_str());
}

void ControlNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	_current_local_pos = *msg;
}

// TODO: I think I am going to remove all beacon measurement related content from this node
// TODO: create another node for the path planning that takes in this information
void ControlNode::beaconMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
	// TODO: decide what to do with the beacon measurement here
}

/*
void ControlNode::apRangeCallback(const aa241x_student::APRange::ConstPtr& msg) {
	// TODO: decide what to do with the range measurement here
	// TODO: basically writing some of the controller in this callback
}
*/

void ControlNode::waitForFCUConnection() {
	// wait for FCU connection
	ros::Rate rate(5.0);
	while (ros::ok() && _current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
}


int ControlNode::run() {

	// wait for the controller connection
	waitForFCUConnection();
	ROS_INFO("connected to the FCU");

	// set up the general command parameters
	// NOTE: these will be true for all commands send
	mavros_msgs::PositionTarget cmd;
	cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

	// command positions in local coordinates
	cmd.type_mask = (mavros_msgs::PositionTarget::IGNORE_VX |
		mavros_msgs::PositionTarget::IGNORE_VY |
		mavros_msgs::PositionTarget::IGNORE_VZ |
		mavros_msgs::PositionTarget::IGNORE_AFX |
		mavros_msgs::PositionTarget::IGNORE_AFY |
		mavros_msgs::PositionTarget::IGNORE_AFZ |
		mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

	// cmd.type_mask = 2499;  // mask for Vx Vy and Pz control
	// TODO: need to add a link to the mask to explain the value
	cmd.yaw = 0;			// always want to keep heading north

	// the position information
	// NOTE: this is defined in ENU
	geometry_msgs::Point pos;
	pos.x = 0;	// E
	pos.y = 0;	// N
	pos.z = 0;	// U

	// the velocity information
	// NOTE: this is defined in ENU
	geometry_msgs::Vector3 vel;
	vel.x = 0;	// E
	vel.y = 0;	// N
	vel.z = 0;	// U

	// set the loop rate in [Hz]
	// NOTE: must be faster than 2Hz
	ros::Rate rate(10.0);
	while (ros::ok()) {

		// if not in offboard mode, just keep waiting until we are
		// and if not enabled, then keep waiting
		// NOTE: need to be streaming setpoints in order for offboard to be allowed
		if (_current_state.mode != "OFFBOARD") {
			// send command to stay in the same position
			pos.x = 0;
			pos.y = 0;
			pos.z = 0;

			cmd.header.stamp = ros::Time::now();
			cmd.position = pos;
			cmd.velocity = vel;
			_cmd_pub.publish(cmd);

			// run the ros components
			ros::spinOnce();
			rate.sleep();
			continue;
		}

		// at this point the pixhawk is in offboard control, so now need to make
		// sure sending commands


		// TODO: populate the control elements desired
		pos.x = 0;
		pos.y = 0;
		pos.z = _flight_alt;

		// publish the command
		cmd.header.stamp = ros::Time::now();
		cmd.position = pos;
		cmd.velocity = vel;
		_cmd_pub.publish(cmd);

		// remember need to always call spin once for the callbacks to trigger
		ros::spinOnce();
		rate.sleep();
	}

	// return  exit code
	return EXIT_SUCCESS;


}

int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "control_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	// TODO: determine settings

	// create the node
	ControlNode node(20.0f);

	// run the node
	return node.run();
}