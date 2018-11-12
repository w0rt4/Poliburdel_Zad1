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
mutex mtx;
string recipientName;
string recipientIP;

//Czestotliwosc do ustawienia w Hz
int frequency = 20;
//////////////////////////
int loopCounter;
int loopCounter1=0;

double cordinatesPrecision = 0.00002;//0.000005;
//////////////////

bool mission(mavrosCommand command);
bool waitForStart(mavrosCommand command);
void takeOffHome(mavrosCommand command);
void nextPoint(mavrosCommand command);
void flyHome(mavrosCommand command);
void landHome(mavrosCommand command);
void getLatLongShift(mavrosCommand command, double length, double angle, double &pointLatitude, double &pointLongitude);
bool getCordinates(mavrosCommand command);
void sendPicture(string recipientName, string recipientIP);
bool fileExists(const string& name);

int main(int argc, char* argv[]){

	ros::init(argc, argv, "mapping");
	ros::NodeHandle nh;
	
	mavrosCommand command(&nh);
	
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
	
	cout << recipientName << recipientIP << endl;
	
	thread send = thread(sendPicture, recipientName, recipientIP);
	send.detach();
	
	while (ros::ok()) {
		
		if(loopCounter >= 10){
			if(!mission(command))
			{
				cap.release();
				return 1;
			}
			
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

void sendPicture(string recipientName, string recipientIP)
{	
	int pictureSendCount = 0;
	
	mtx.lock();
	int stateCopy = currentState;
	mtx.unlock();
	while(stateCopy != END)
	{
		string name = get_username();
		if (fileExists("/home/" + name + "/zdj/" + to_string(pictureSendCount) + ".jpg"))
		{
			char buffer[128];
			string result = "";
			string scp = "scp /home/" + name + string("/zdj/") + to_string(pictureSendCount) + string(".jpg ") + recipientName + string("@") + recipientIP + string(":/home/") + recipientName + string("/zdj");
			FILE* pipe = popen(scp.c_str(), "r");
			if (pipe)
			{
				try
				{
					while (!feof(pipe))
					{
						if (fgets(buffer, 128, pipe) != NULL)
						{
							result += buffer;
						}
					}
				}
				catch (...)
				{
					cout<<"Error during sending file " << pictureSendCount << endl;
					pclose(pipe);
					
					mtx.lock();
					stateCopy = currentState;
					mtx.unlock();
					
					continue;
				}
				
				pclose(pipe);	
			}
		
			if (result.empty())
			{
				cout << "PICTURE " << pictureSendCount << " SEND" << endl;
				pictureSendCount++;
			}			
		}
		else
		{
			sleep(5);
		}
		
		mtx.lock();
		stateCopy = currentState;
		mtx.unlock();
	}
}

bool fileExists(const string& name)
{
	if (FILE *file = fopen(name.c_str(), "r"))
	{
		fclose(file);
		return true;
	}
	
	return false;
}

bool mission(mavrosCommand command){
	mtx.lock();
	int stateCopy = currentState;
	mtx.unlock();
	
	switch(stateCopy){
		case WAIT_FOR_START:
			if(isInit == true)
			{
				if(!waitForStart(command))
				{
					return false;
				}
			}
			else
			{
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
	
	return true;
}

bool waitForStart(mavrosCommand command){
	
	homeLatitude = command.getGlobalPositionLatitude();
	homeLongitude = command.getGlobalPositionLongitude();
	homeAltitude = command.getGlobalPositionAltitude();
	
	latitude[pointsCount] = homeLatitude;
	longitude[pointsCount] = homeLongitude;
	
	dronAltitude = missionAltitude;
	if(!command.guided())
	{
		return false;
	}
	sleep(1);

	if(!command.arm())
	{
		return false;
	}
	sleep(1);
	
	command.takeOff(missionAltitude);
	mtx.lock();
	currentState = TAKEOFF_HOME;
	mtx.unlock();
	sleep(3);
	command.flyTo(command.getGlobalPositionLatitude(), command.getGlobalPositionLongitude(), missionAltitude);

	return true;	
}

void takeOffHome(mavrosCommand command){
	
	cout<<"CURRENT ALTITUDE: "<< command.getGlobalPositionAltitude() - homeAltitude <<endl;
	if(command.getGlobalPositionAltitude() - homeAltitude >= missionAltitude){
		mtx.lock();
		currentState = NEXT_POINT;
		mtx.unlock();
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
			mtx.lock();
			currentState = LAND_HOME;
			mtx.unlock();
			dronAltitude = 5;
			command.flyTo(homeLatitude, homeLongitude, dronAltitude);
			return;
		}
		
		if(i % 6 == 1)
		{
			cout << "Mapuje" << endl;
			isMapping = true;
			cordinatesPrecision = 0.00002;
		}
		else if (i % 6 == 0)
		{
			isMapping = false;
			cordinatesPrecision = 0.00002;
		}
		else
		{
			cout << "Nie Mapuje " <<i % 5<< endl;
			isMapping = false;
			cordinatesPrecision = 0.00006;
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
		if(!command.getArmed())
		{
			mtx.lock();
			currentState = END;
			mtx.unlock();
		}
	}
}

void getLatLongShift(mavrosCommand command, double length, double angle, double &pointLatitude, double &pointLongitude)
{
	/*double easting, northing;
	int zone;
 	bool northp;
	UTMUPS::Forward(*pointLatitude, *pointLongitude, zone, northp, easting, northing);
	
	double longitudeShift = length * sin(command.toRad(angle));
	double latitudeShift = length * cos(command.toRad(angle));
	
	northing += longitudeShift;
	easting += latitudeShift;
	
	UTMUPS::Reverse(zone, northp, easting, northing, *pointLatitude, *pointLongitude);*/
	
	double lat = command.toRad(pointLatitude);
	double lng = command.toRad(pointLongitude);
	
	double lat2 = asin(sin(lat) * cos((length / 1000) / 6378.1) + cos(lat) * sin((length / 1000) / 6378.1) * cos(command.toRad(angle)));
	double lng2 = lng + atan2(sin(command.toRad(angle)) * sin((length / 1000) / 6378.1) * cos(lat), cos((length / 1000) / 6378.1) - sin(lat) * sin(lat2));
	
	pointLatitude = lat2 / 3.14159265 * 180;
	pointLongitude = lng2 / 3.14159265 * 180;
}

bool getCordinates(mavrosCommand command){

	string name = get_username();
	
	ifstream theFile("/home/" + name + "/catkin_ws/src/Poliburdel_Zad1/mission.json");
	json missionSettings = json::parse(theFile);
	theFile.close();
	
	int ik,jk;
 	double x, x_wsp_14, x_wsp_12, x_pom;
 	double y, y_wsp_14, y_wsp_12, y_pom;
 	double easting, northing, longitudeShift, latitudeShift, bearing;
 	int zone;
 	bool northp;
 	
 	missionAltitude = missionSettings["mission"]["altitude"];
 	pictureFrequency = missionSettings["mission"]["photosFrequency"];
 	recipientName = missionSettings["mission"]["recipientName"];
 	recipientIP = missionSettings["mission"]["recipientIP"];
 	
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
			
			bearing = command.getBearingBetweenCoordinates(latitude[pointsCount - 2], longitude[pointsCount - 2], x, y);
			cout << bearing << endl;
			
			// Zakrety test
			getLatLongShift(command, 9.2, fmod((bearing + 30  + 360), 360), x, y);
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			
			getLatLongShift(command, 5.4, fmod((bearing + 60 + 360), 360), x, y);
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			
			getLatLongShift(command, 6, fmod((bearing + 90 + 360), 360), x, y);
			latitude[pointsCount] = x;
			longitude[pointsCount] = y;
			pointsCount++;
			
			getLatLongShift(command, 5, fmod((bearing + 120 + 360), 360), x, y);
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
			
			bearing = command.getBearingBetweenCoordinates(latitude[pointsCount - 2], longitude[pointsCount - 2], x, y);
			cout << bearing << endl;
			
			if(jk < scanCount - 1) 
			{
				// Zakrety test
				getLatLongShift(command, 9.2, fmod((bearing - 30 + 360), 360), x, y);
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
			
				getLatLongShift(command, 5.4, fmod((bearing - 60 + 360), 360), x, y);
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
			
				getLatLongShift(command, 6, fmod((bearing - 90 + 360), 360), x, y);
				latitude[pointsCount] = x;
				longitude[pointsCount] = y;
				pointsCount++;
			
				getLatLongShift(command, 5, fmod((bearing - 120 + 360), 360), x, y);
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


