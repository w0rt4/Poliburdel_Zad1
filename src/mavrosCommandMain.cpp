#include <iostream>
#include "ros/ros.h"
#include <fstream>
#include <math.h>
#include "mavrosCommand.hpp"
#include <nlohmann/json.hpp>
#include <GeographicLib/UTMUPS.hpp>

using namespace std;
using namespace GeographicLib;
using namespace cv;
using namespace boost;

double latitude[200];
double longitude[200];
int pointsCount = 0;
float missionAltitude = 5;
int yawMapping;
#define PI 3.14159265

enum directions
{
	FromStartLine = 1,
	ToStartLine = 2
};

using json = nlohmann::json;

const uint8_t WAIT_FOR_START = 0;
const uint8_t TAKEOFF_HOME = 1;
const uint8_t NEXT_POINT = 2;
const uint8_t FLY_HOME = 3;
const uint8_t LAND_HOME = 4;
const uint8_t END = 5;
int i=0;
int currentState = 0;
bool isInit = false;
double homeLatitude;
double homeLongitude;
double homeAltitude;
bool isMapping = false; //is drone mapping at the moment
bool precisionLanding = false;
double dronAltitude;
double pictureFrequency = 40; //40 = 1 photo per  two seconds


//Czestotliwosc do ustawienia w Hz
int frequency = 20;
//////////////////////////
int loopCounter;
int loopCounter1=0;

double cordinatesPrecision = 0.00002;//0.000005;
//////////////////

void mission(mavrosCommand command);
void waitForStart(mavrosCommand command);
void takeOffHome(mavrosCommand command);
void nextPoint(mavrosCommand command);
void flyHome(mavrosCommand command);
void landHome(mavrosCommand command);
void getLatLongShift(mavrosCommand command, double length, double angle, double pointLatitude, double pointLongitude);
bool getCordinates(mavrosCommand command);



int main(int argc, char* argv[]){

	ros::init(argc, argv, "mapping");
	mavrosCommand command;
	
	ros::Rate loop_rate(frequency);
	sleep(1);
	
	if(getCordinates(command) == false)
	{
		cout<<"FILE mission.json IS DAMAGED!"<<endl;
		return 0;
	}

	i=0;
	
	VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
	
	
	if (cap.isOpened() == false)  
	{
		cout << "Cannot open the video camera" << endl;
		return -1;
	} 
	
	int cntr = 0;
	Mat frame;
	
	while (ros::ok()) {
		
		if(loopCounter >= 10){
			mission(command);
			loopCounter = 0;
		}
		
		if(loopCounter1 >= pictureFrequency && isMapping == true){
			
			cap.read(frame);
			
			thread save = thread(savePicture, frame, cntr, i);
			save.detach();
			
			Mat copyFrame = frame;
			
			thread bw_save = thread(bwPicture, copyFrame, cntr);
			bw_save.detach();
			
			cntr++;
			loopCounter1 = 0;
		}
		
		
		loopCounter++;
		loopCounter1++;
		ros::spinOnce();
		loop_rate.sleep();
	}	
	cap.release();
	return 0;
}

void mission(mavrosCommand command){
	switch(currentState){
		case WAIT_FOR_START:
			if(isInit == true)waitForStart(command);
			else{
				command.initSubscribers();
				isInit = true;
			}
		break;
		case TAKEOFF_HOME:
			takeOffHome(command);
		break;
		case NEXT_POINT:
			nextPoint(command);
		break;
		case LAND_HOME:
			landHome(command);
		break;
		case END:
			cout<<"END OF MISSION"<<endl;
			exit(0);
		break;
		default:

		break;
	}
}

void waitForStart(mavrosCommand command){
	
	homeLatitude = command.getGlobalPositionLatitude();
	homeLongitude = command.getGlobalPositionLongitude();
	homeAltitude = command.getGlobalPositionAltitude();
	
	latitude[pointsCount] = homeLatitude;
	longitude[pointsCount] = homeLongitude;
	
	dronAltitude = missionAltitude;
	command.guided();
	sleep(1);

	command.arm();
	sleep(1);

	command.takeOff(missionAltitude);
	currentState = TAKEOFF_HOME;
	sleep(3);
	command.flyTo(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), missionAltitude);
	
}

void takeOffHome(mavrosCommand command){
	
	cout<<"CURRENT ALTITUDE: "<< command.getGlobalPositionAltitude() - homeAltitude <<endl;
	if(command.getGlobalPositionAltitude() - homeAltitude >= missionAltitude){
		currentState = NEXT_POINT;
		command.flyTo(latitude[i], longitude[i], missionAltitude);
		dronAltitude = missionAltitude;
		cout<<"RIGHT ALTITUDE"<<endl;
		cout<<"FLY DESTINATION: ";
		cout<<fixed << setprecision(7) << latitude[i] <<", ";
		cout<<fixed << setprecision(7) << longitude[i] <<endl;
	
	}
	else{
		 dronAltitude = dronAltitude + 0.01;
		 command.flyTo(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), dronAltitude);
	 }
}

void nextPoint(mavrosCommand command){
	
	cout<<"CURRENT POSITION: ";
	cout<<fixed << setprecision(7) << command.getGlobalPositionLatitude()<<", ";
	cout<<fixed << setprecision(7) << command.getGlobalPositionLongitude()<<" ";
	
	if(command.isInPosition(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), latitude[i], longitude[i], cordinatesPrecision)){
		
		i++;
		
		if(i >= pointsCount + 1){ //IS IN HOME POSITION?
			currentState = LAND_HOME;
			dronAltitude = 5;
			command.flyTo(homeLatitude, homeLongitude, dronAltitude);
			return;
		}
		
		if(i % 5 == 1)
		{
			isMapping = true;
			cordinatesPrecision = 0.00002;
		}
		else
		{
			 isMapping = false;
			 cordinatesPrecision = 0.00008;
		}
		
		command.flyTo(latitude[i], longitude[i], missionAltitude);
	}
	else if(command.isInPosition(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), latitude[i - 1], longitude[i - 1], 0.00002))
	{
		cout<<"RESEND COMMAND FLY TO"<<endl;
		command.flyTo(latitude[i], longitude[i], missionAltitude);
	}

}

void landHome(mavrosCommand command)
{
	if(!precisionLanding){
		
		cout<<"CURRENT ALTITUDE: "<< command.getGlobalPositionAltitude() - homeAltitude<<endl;	
		if(command.getGlobalPositionAltitude() - homeAltitude <= 5)
		{
			precisionLanding = true;
			sleep(3);
			command.land();
		}
		else
		{
			 dronAltitude = dronAltitude - 0.1;
			 command.flyTo(homeLatitude, homeLongitude, dronAltitude);
		}
	}
	else
	{
		cout<<command.getArmed()<<endl;
		if(!command.getArmed())currentState = END;
	}
}

void getLatLongShift(mavrosCommand command, double length, double angle, double* pointLatitude, double* pointLongitude)
{
	double easting, northing;
	int zone;
 	bool northp;
	UTMUPS::Forward(*pointLatitude, *pointLongitude, zone, northp, easting, northing);
	
	double longitudeShift = length * sin(command.toRad(angle));
	double latitudeShift = length * cos(command.toRad(angle));
	
	northing += longitudeShift;
	easting += latitudeShift;
	
	UTMUPS::Reverse(zone, northp, easting, northing, *pointLatitude, *pointLongitude);
}

bool getCordinates(mavrosCommand command){

	string name = get_username();
	
	ifstream theFile("/home/" + name + "/catkin_ws/src/Poliburdel_Zad1/mission.json");
	json missionSettings = json::parse(theFile);
	theFile.close();
	
	int ik,jk;
 	double x, x_wsp_14, x_wsp_12, x_pom;
 	double y, y_wsp_14, y_wsp_12, y_pom;
 	double easting, northing, longitudeShift, latitudeShift;
 	int zone;
 	bool northp;
 	
 	missionAltitude = missionSettings["mission"]["altitude"];
 	pictureFrequency = missionSettings["mission"]["photosFrequency"];
 	
 	double leftDownLongitude = missionSettings["mission"]["leftDown"]["longitude"];
 	double leftDownLatitude = missionSettings["mission"]["leftDown"]["latitude"];
 	double leftUpLongitude = missionSettings["mission"]["leftUp"]["longitude"];
 	double leftUpLatitude = missionSettings["mission"]["leftUp"]["latitude"];
 	double rightDownLongitude = missionSettings["mission"]["rightDown"]["longitude"];
 	double rightDownLatitude = missionSettings["mission"]["rightDown"]["latitude"];
 	
 	int direction = directions(FromStartLine);
 	
 	int pointsOnSingleScan = 2;
 	int scanCount = ceil(command.distanceBetweenCordinates(leftDownLatitude, leftDownLongitude, rightDownLatitude, rightDownLongitude) / 19);
 	double distanceBetweenSingleScan = command.distanceBetweenCordinates(leftDownLatitude, leftDownLongitude, rightDownLatitude, rightDownLongitude) / scanCount;
	
	if(missionAltitude == 0 || leftDownLatitude == 0 || leftUpLatitude == 0 || rightDownLatitude == 0 || leftDownLongitude == 0 || leftUpLongitude == 0 || rightDownLongitude == 0)
	{
		return false;
	}	 
	
	pictureFrequency = pictureFrequency * 20;
	yawMapping = atan((leftUpLongitude - leftDownLongitude) * 0.67 / (leftUpLatitude - leftDownLatitude) * 1.11) * 180 / PI;

	if(leftUpLongitude - leftDownLongitude >= 0 && leftUpLatitude - leftDownLatitude == 0)
	{
		yawMapping = 90;
	}
	else if(leftUpLatitude - leftDownLatitude < 0)
	{
		yawMapping = 180 + yawMapping;
	}
	else if(leftUpLongitude - leftDownLongitude < 0  && leftUpLatitude - leftDownLatitude == 0)
	{
		yawMapping = 270;
	}
	else if(leftUpLongitude - leftDownLongitude < 0  && leftUpLatitude - leftDownLatitude > 0)
	{
		yawMapping = 360 + yawMapping;
	}
	
	yawMapping = yawMapping % 360;
				
 	x_wsp_12 = leftUpLatitude - leftDownLatitude;
	y_wsp_12 = leftUpLongitude - leftDownLongitude;
	x_wsp_14 = rightDownLatitude - leftDownLatitude;
	y_wsp_14 = rightDownLongitude - leftDownLongitude;
	x_wsp_12 = x_wsp_12 / (pointsOnSingleScan - 1);
	y_wsp_12 = y_wsp_12 / (pointsOnSingleScan - 1);
	x_wsp_14 = x_wsp_14 / (scanCount - 1);
	y_wsp_14 = y_wsp_14 / (scanCount - 1);
	x_pom = leftDownLatitude;
	y_pom = leftDownLongitude;
 	
 	for(jk = 0; jk < scanCount; jk++)
 	{
		if(direction == directions(FromStartLine))
		{
			x = x_pom;
			y = y_pom;
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			
			for(ik = 1; ik < pointsOnSingleScan; ik++)
			{
				x = x_pom + ik * x_wsp_12;
				y = y_pom + ik * y_wsp_12;
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
			}
			
			x_pom = x + x_wsp_14;
			y_pom = y + y_wsp_14;
			
			cout << command.getBearingBetweenCoordinates(x, y, x_pom, y_pom)<<endl;
			
			// Zakrety test
			getLatLongShift(command, 9.2, 70, &x, &y);
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			
			getLatLongShift(command, 5.4, 54, &x, &y);
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			
			getLatLongShift(command, 5, 4, &x, &y);
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			//
			
			direction = directions(ToStartLine);
	   }
		else if(direction == directions(ToStartLine))
		{
			x = x_pom;
			y = y_pom;
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			
			for(ik = 1; ik < pointsOnSingleScan; ik++)
			{
				x = x_pom - ik * x_wsp_12;
				y = y_pom - ik * y_wsp_12;
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
			}
			
			x_pom = x + x_wsp_14;
			y_pom = y + y_wsp_14;
			
			if(jk < scanCount - 1) 
			{
				// Zakrety test
				getLatLongShift(command, 9.2, 340, &x, &y);
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
			
				getLatLongShift(command, 5, 360, &x, &y);
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
			
				getLatLongShift(command, 5, 30, &x, &y);
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
				//
			}
			
			direction = directions(FromStartLine);
	   }
	}
	
	return true;
}


