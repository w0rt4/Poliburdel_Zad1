#include <iostream>
#include "mavrosCommand.hpp"

using namespace std;
#define PI 3.14159265

mavrosCommand::mavrosCommand(){
	
	_nh = ros::NodeHandle();
	init();
}

mavrosCommand::~mavrosCommand(){
}

void mavrosCommand::init(){
	_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	_clientTakeOff = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
	_clientGuided = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	_clientLand = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
	_clientServo = _nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
	_clientPicture = _nh.serviceClient<std_srvs::Empty>("/image_saver/save");
	
	_pub_mav = _nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global",100);
	_pub_mavPositionTarget = _nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",100);
	
	/////
	_compassHeadingSub = _nh.subscribe("/mavros/global_position/compass_hdg", 100, &mavrosCommand::compassHeadingCb, this);
	_adsbVehicleSub = _nh.subscribe("/mavros/adsb/vehicle", 100, &mavrosCommand::adsbVehicleCb, this);
	_globalPositionGlobalSub = _nh.subscribe("/mavros/global_position/global", 100, &mavrosCommand::globalPositionGlobalCb, this);
	_stateSub = _nh.subscribe("/mavros/state", 100, &mavrosCommand::stateCb, this);
	_globalPositionRelAltitudeSub = _nh.subscribe("/mavros/global_position/rel_alt", 100, &mavrosCommand::globalPostionRelAltitudeCb, this);
	_timeReferenceSub = _nh.subscribe("/mavros/time_reference", 100, &mavrosCommand::timeReferenceCb, this);
	_qrMessageSub = _nh.subscribe("/visp_auto_tracker/code_message", 100, &mavrosCommand::qrMessageCb, this);
	_qrPositionSub = _nh.subscribe("/visp_auto_tracker/object_position", 100, &mavrosCommand::qrPositionCb, this);
}

void mavrosCommand::qrPositionCb(geometry_msgs::PoseStamped::ConstPtr msg) {
	_qrPositionX = msg->pose.position.x;
	_qrPositionY = msg->pose.position.y;
}

void mavrosCommand::qrMessageCb(std_msgs::String::ConstPtr msg) {
	_qrMessage = msg->data;
}

void mavrosCommand::adsbVehicleCb(mavros_msgs::ADSBVehicle::ConstPtr msg) {
	
	_adsbICAO = msg->ICAO_address;
	_adsbHeading = msg->heading;
	_adsbVelocity = msg->hor_velocity;
	_adsbLatitude = msg->latitude;
	_adsbLongitude = msg->longitude;

}

void mavrosCommand::globalPositionGlobalCb(sensor_msgs::NavSatFix::ConstPtr msg){
	_globalPositionLatitude = msg->latitude;
	_globalPositionLongitude = msg->longitude;
	_globalPositionAltitude = msg->altitude;
}

void mavrosCommand::compassHeadingCb(std_msgs::Float64::ConstPtr msg) {
	_compassHeading = msg->data;
}
	
void mavrosCommand::stateCb(mavros_msgs::State::ConstPtr msg){
	_connected = msg->connected;
	_armed = msg->armed;
	_guided = msg->guided;
	_state = msg->mode;
}

void mavrosCommand::globalPostionRelAltitudeCb(std_msgs::Float64::ConstPtr msg){
	_relativeAltitude = msg->data;
}
 
void mavrosCommand::timeReferenceCb(sensor_msgs::TimeReference::ConstPtr msg) {
	_time = msg->time_ref.toSec();
}
 
void mavrosCommand::takeOff(double altitude){
	
	mavros_msgs::CommandTOL srv_takeOff;
	srv_takeOff.request.min_pitch = 0.0;
	srv_takeOff.request.yaw = 0.0;
	srv_takeOff.request.latitude = 0.0;//-35.363265;
	srv_takeOff.request.longitude = 0.0;//149.165241;
	srv_takeOff.request.altitude = altitude;
	_clientTakeOff.call(srv_takeOff);
	if (srv_takeOff.response.success) {
	cout << "TAKE OFF SUCCESFUL" << endl;
	}
	else cout << "TAKE OFF FAIL" <<endl;
}

void mavrosCommand::land(){
	
	mavros_msgs::CommandTOL srv_land;
	srv_land.request.min_pitch = 0.0;
	srv_land.request.yaw = 0.0;
	srv_land.request.latitude = 0.0;//-35.363265;
	srv_land.request.longitude = 0.0;//149.165241;
	srv_land.request.altitude = 0.0;
	_clientLand.call(srv_land);
	if (srv_land.response.success) {
	cout << "LAND SUCCEFUL" << endl;
	}
	else cout << "LAND FAIL" <<endl;
	
}

void mavrosCommand::picture(){
	std_srvs::Empty srv_picture;
	_clientPicture.call(srv_picture);
	cout<<"PICTURE CAPTURED"<<endl;
	//if (srv_picture.response.success)cout<<"PICTURE CAPTURED"<<endl;
	//else cout<<"PICTURE FAIL"<<endl;
}

void mavrosCommand::servo(double width){//width 1000-2000
	
	mavros_msgs::CommandLong srv_servo;
	srv_servo.request.command = 183;
	srv_servo.request.param1 = 9.0;
	srv_servo.request.param2 = width;
	_clientServo.call(srv_servo);
	if (srv_servo.response.success) {
	cout << "SERVO SUCCESFUL" << endl;
	}
	else cout << "SERVO FAIL" <<endl;
	
}

void mavrosCommand::guided(){
	
	mavros_msgs::SetMode srvSetMode;
	srvSetMode.request.custom_mode = "GUIDED";
	_clientGuided.call(srvSetMode);
	if (srvSetMode.response.mode_sent) {
	cout << "GUIDED MODE SUCCESFUL" << endl;
	}else cout << "GUIDED MODE FAIL" <<endl;
}

void mavrosCommand::arm(){
	
	mavros_msgs::CommandBool srv;
	
	while(1){
	srv.request.value = true;
	_client.call(srv);
	
	if (srv.response.success) {
	cout << "ARM SUCCESFUL" << endl;
	break;
	}else{
		cout << "ARM FAIL" <<endl;
		sleep(5);
	}
	}
	
}



void mavrosCommand::flyTo(double latitude, double longitude, double altitude){
	
	cmd_pos_glo.header.frame_id ="SET_POSITION_TARGET_GLOBAL_INT";
	cmd_pos_glo.coordinate_frame = 6;
	cmd_pos_glo.type_mask = 4088;
	cmd_pos_glo.latitude = latitude;
	cmd_pos_glo.longitude = longitude;
	cmd_pos_glo.altitude = altitude;
	cmd_pos_glo.velocity.x = 0.0;
	cmd_pos_glo.velocity.y = 0.0;
	cmd_pos_glo.velocity.z = 0.0;
	cmd_pos_glo.acceleration_or_force.x = 0.0;
	cmd_pos_glo.acceleration_or_force.y = 0.0;
	cmd_pos_glo.acceleration_or_force.z = 0.0;
	cmd_pos_glo.yaw = 0.0;
	cmd_pos_glo.yaw_rate = 0.0;
	
	_pub_mav.publish(cmd_pos_glo);
}

void mavrosCommand::slowDown(double counterForce){
	
	cout<<"SLOWING DOWN"<<endl;
	cmd_pos_target.header.frame_id ="SET_POSITION_TARGET_LOCAL_NED";
	cmd_pos_target.coordinate_frame = 9;
	cmd_pos_target.type_mask = 4039;
	cmd_pos_target.velocity.x = 0.0;
	cmd_pos_target.velocity.y = -counterForce;
	cmd_pos_target.velocity.z = 0.0;
	cmd_pos_target.acceleration_or_force.x = 0.0;
	cmd_pos_target.acceleration_or_force.y = 0.0;
	cmd_pos_target.acceleration_or_force.z = 0.0;
	cmd_pos_target.yaw = 0.0;
	cmd_pos_target.yaw_rate = 0.0;
	
	_pub_mavPositionTarget.publish(cmd_pos_target);
}

double mavrosCommand::getCompassHeading(){	
	return _compassHeading;
}
int mavrosCommand::getTime(){	
	return _time;
}
double mavrosCommand::getRelativeAltitude(){	
	return _relativeAltitude;
}

int mavrosCommand::getAdsbIcao(){
	return _adsbICAO;
}
double mavrosCommand::getAdsbHeading(){
	return _adsbHeading;
}
double mavrosCommand::getAdsbVelocity(){
	return _adsbVelocity;
}
double mavrosCommand::getAdsbLatitude(){
	return _adsbLatitude;
}
double mavrosCommand::getAdsbLongitude(){
	return _adsbLongitude;
}

double mavrosCommand::getGlobalPositionLatitude(){
	return _globalPositionLatitude;
}
double mavrosCommand::getGlobalPositionLongitude(){
	return _globalPositionLongitude;
}
double mavrosCommand::getGlobalPositionAltitude(){
	return _globalPositionAltitude;
}

bool mavrosCommand::getConnected(){
	return _connected;
}
bool mavrosCommand::getArmed(){
	return _armed;
}
bool mavrosCommand::getGuided(){
	return _guided;
}
string mavrosCommand::getState(){
	return _state;
}
string mavrosCommand::getQrValue(){
	return _qrMessage;
}
double mavrosCommand::getQrPositionX(){
	return _qrPositionX;
}
double mavrosCommand::getQrPositionY(){
	return _qrPositionY;
}

//others
double mavrosCommand::toRad(double degree){
    return degree/180 * PI;
}

bool mavrosCommand::isInPosition(double lat_current, double long_current, double lat_destination, double long_destination, double cordinatesPrecision){
	
	if(lat_current >= lat_destination - cordinatesPrecision && 
	   lat_current <= lat_destination + cordinatesPrecision && 
	   long_current >= long_destination - cordinatesPrecision && 
	   long_current <= long_destination + cordinatesPrecision){
		   cout<<"IN THE RIGHT POSITION"<<endl;
		   return true;
	   }
	else {
		cout<<"STILL FLYING"<<endl;
		return false;
	}
}

double mavrosCommand::distanceBetweenCordinates(double lat1, double long1, double lat2, double long2) {
    
    double dist;
    dist = sin(toRad(lat1)) * sin(toRad(lat2)) + cos(toRad(lat1)) * cos(toRad(lat2)) * cos(toRad(long1 - long2));
    dist = acos(dist);
    dist = 6371 * dist * 1000;
    return dist;
}

void mavrosCommand::initSubscribers(){
	getTime();
	getGlobalPositionLatitude();
	getGlobalPositionLongitude();
	getGlobalPositionAltitude();
	getArmed();
	getCompassHeading();
	getState();
}
