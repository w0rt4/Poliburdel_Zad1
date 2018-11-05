#include <iostream>
#include "mavrosCommand.hpp"

using namespace std;
using namespace cv;
#define PI 3.14159265

string get_username() {
    struct passwd *pwd = getpwuid(getuid());
    if (pwd)
        return pwd->pw_name;
    else
        return "odroid";
}

void savePicture(Mat in_frame, int cntr)
{
	string name = get_username();
	string savingName = "/home/" + name + "/zdj/" + to_string(cntr) + ".jpg";
	imwrite(savingName, in_frame);
	cout << "PICTURE: " << cntr << " SAVED" << endl;	
}

void bwPicture(Mat in_frame, int cntr)
{
	Mat lower_red_hue_range, upper_red_hue_range, red_hue_image, hsv_image, image_open;

	string name = get_username();
	cvtColor(in_frame, hsv_image, COLOR_BGR2HSV);

	inRange(hsv_image, Scalar(0,150,80), Scalar(10,255,170) , lower_red_hue_range);
	inRange(hsv_image, Scalar(160,150,80), Scalar(179,255,170) , upper_red_hue_range);

	addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
	GaussianBlur(red_hue_image, red_hue_image, Size(9,9), 2, 2);
	
	int morph_size = 2;
    Mat element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    morphologyEx(red_hue_image, image_open, MORPH_OPEN, element, Point(-1,-1), 3 ); 
    
    vector<Vec3f> circles;
    
    HoughCircles( image_open, circles, CV_HOUGH_GRADIENT,3.2,image_open.rows/4,90, 50,12, 22 );
		
	cvtColor(image_open, image_open, COLOR_GRAY2BGR);
    
    for( size_t i = 0; i < circles.size(); i++ )
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		circle( image_open, center,3, Scalar(0,0,255), -1, 8, 0 );
		circle( image_open, center, radius, Scalar(0,0,255),5, 8, 0 );
		cout << "Circle " << i << " center: " << center << " radius: " << radius << endl;
	}
    
	string savingName = "/home/" + name + "/zdj2/" + to_string(cntr) + ".jpg";
	imwrite(savingName, image_open);
}


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
	
	_pub_mav = _nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global",100);
	_pub_mavPositionTarget = _nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",100);
	

	_compassHeadingSub = _nh.subscribe("/mavros/global_position/compass_hdg", 100, &mavrosCommand::compassHeadingCb, this);
	_adsbVehicleSub = _nh.subscribe("/mavros/adsb/vehicle", 100, &mavrosCommand::adsbVehicleCb, this);
	_globalPositionGlobalSub = _nh.subscribe("/mavros/global_position/global", 100, &mavrosCommand::globalPositionGlobalCb, this);
	_stateSub = _nh.subscribe("/mavros/state", 100, &mavrosCommand::stateCb, this);
	_globalPositionRelAltitudeSub = _nh.subscribe("/mavros/global_position/rel_alt", 100, &mavrosCommand::globalPostionRelAltitudeCb, this);
	_timeReferenceSub = _nh.subscribe("/mavros/time_reference", 100, &mavrosCommand::timeReferenceCb, this);
	_rcInSub = _nh.subscribe("mavros/rc/in", 100, &mavrosCommand::rcInputCb, this);
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

void mavrosCommand::rcInputCb(mavros_msgs::RCIn::ConstPtr msg) {
	_rcIn[5] = msg->channels[5];
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
int mavrosCommand::getRCInput(){
	return _rcIn[5];
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

double mavrosCommand::getBearingBetweenCoordinates(double lat1, double long1, double lat2, double long2)
{
    double x,y;
    x = cos(toRad(lat2)) * sin(toRad((long2 - long1)));
    cout<< "X "<<x<<endl;
    y = cos(toRad(lat1)) * sin(toRad(lat2)) - sin(toRad(lat1)) * cos(toRad(lat2)) * cos(toRad((long2 - long1)));
    cout<< "Y "<<y<<endl;
    
    return fmod(atan2(x, y) / PI * 180 + 360, 360);
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


