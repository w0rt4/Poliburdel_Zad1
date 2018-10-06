#ifndef MAVROS_COMMAND_HPP
#define MAVROS_COMMAND_HPP

#include "ros/ros.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/SetMode.h"
#include <iostream>
#include "mavros_msgs/ADSBVehicle.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/TimeReference.h"
#include "mavros_msgs/State.h"
#include <string>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"

using namespace std;

class mavrosCommand {
public:
	
	mavrosCommand();
	
	virtual ~mavrosCommand();
	
	//services
	void land();
	void guided();
	void arm();
	void takeOff(double altitude);
	void servo(double width);
	void picture();
	
	//publishers
	void flyTo(double latitude, double longitude, double altitude);
	void slowDown(double counterForce);
	
	//subscribers
	double getCompassHeading();
	int getTime();
	double getRelativeAltitude();
	
	int getAdsbIcao();
	double getAdsbHeading();
	double getAdsbVelocity();
	double getAdsbLatitude();
	double getAdsbLongitude();
	
	double getGlobalPositionLatitude();
	double getGlobalPositionLongitude();
	double getGlobalPositionAltitude();
	
	bool getConnected();
	bool getArmed();
	bool getGuided();
	string getState();
	string getQrValue();
	double getQrPositionX();
	double getQrPositionY();
	
	//others
	double toRad(double degree);
	bool isInPosition(double lat_current, double long_current, double lat_destination, double long_destination, double cordinatesPrecision);
	double distanceBetweenCordinates(double lat1, double long1, double lat2, double long2);
	void initSubscribers();
	
private:
	void init();
	
	ros::NodeHandle _nh;
	
	ros::ServiceClient _client;
	ros::ServiceClient _clientTakeOff;
	ros::ServiceClient _clientGuided;
	ros::ServiceClient _clientLand;
	ros::ServiceClient _clientServo;
	ros::ServiceClient _clientPicture;
	
	ros::Publisher _pub_mav;
	ros::Publisher _pub_mavPositionTarget;
	
	mavros_msgs::GlobalPositionTarget cmd_pos_glo;
	mavros_msgs::PositionTarget cmd_pos_target;
	
	void adsbVehicleCb(mavros_msgs::ADSBVehicle::ConstPtr msg);
	void globalPositionGlobalCb(sensor_msgs::NavSatFix::ConstPtr msg);
	void compassHeadingCb(std_msgs::Float64::ConstPtr msg);
	void stateCb(mavros_msgs::State::ConstPtr msg);
	void globalPostionRelAltitudeCb(std_msgs::Float64::ConstPtr msg);
	void timeReferenceCb(sensor_msgs::TimeReference::ConstPtr msg);
	void qrMessageCb(std_msgs::String::ConstPtr msg);
	void qrPositionCb(geometry_msgs::PoseStamped::ConstPtr msg);
		
	//Subscribers
	ros::Subscriber _adsbVehicleSub;
	ros::Subscriber _globalPositionGlobalSub;
	ros::Subscriber _compassHeadingSub;
	ros::Subscriber _stateSub;
	ros::Subscriber _globalPositionRelAltitudeSub;
	ros::Subscriber _timeReferenceSub;
	ros::Subscriber _qrMessageSub;
	ros::Subscriber _qrPositionSub;
	
	//adsb/vehicle
	int _adsbICAO;
	double _adsbHeading, _adsbVelocity, _adsbLatitude, _adsbLongitude;
	

	//global_position/global
	double _globalPositionLatitude, _globalPositionLongitude, _globalPositionAltitude;
	
	//global_position/compass_hdg
	double _compassHeading;
	
	//state
	bool _connected, _armed, _guided;
	string _state;
	
	//global_position/rel_alt
	double _relativeAltitude;
	
	//timeReference
	time_t _time;
	
	//qrMessage
	string _qrMessage;
	
	//qrPosition
	double _qrPositionX, _qrPositionY;
};


#endif
