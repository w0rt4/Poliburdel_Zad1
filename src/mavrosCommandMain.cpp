#include <iostream>
#include "ros/ros.h"
#include <fstream>
#include <math.h>
#include "mavrosCommand.hpp"
using namespace std;

double latitude[15];
double longitude[15];
int ile=0;
float missionAltitude = 5;
int yawMapping;
#define PI 3.14159265



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
int loopCounter1;

int checkpointsQuantity = 11;
double cordinatesPrecision = 0.00002;//0.000005;
//////////////////

void mission(mavrosCommand command);
void waitForStart(mavrosCommand command);
void takeOffHome(mavrosCommand command);
void nextPoint(mavrosCommand command);
void flyHome(mavrosCommand command);
void landHome(mavrosCommand command);
bool getCordinates();

int main(int argc, char* argv[]){

	ros::init(argc, argv, "beginner_tutorials");
	mavrosCommand command;
	
	ros::Rate loop_rate(frequency);
	sleep(1);
	
	
	

	if(getCordinates() == false){
		cout<<"FILE mission.txt IS DAMAGED!"<<endl;
		return 0;
	}
	/*
	for(i=0; i<ile; i++){
		cout<<fixed << setprecision(7) << latitude[i] <<", ";
		cout<<fixed << setprecision(7) << longitude[i] <<endl;
	}
	*/
	i=0;
	
	while (ros::ok()) {
		
		if(loopCounter >= 10){
			mission(command);
			loopCounter = 0;
		}
		
		if(loopCounter1 >= pictureFrequency && isMapping == true){
			//if((command.getCompassHeading() >= yawMapping - 15 && command.getCompassHeading() >= yawMapping + 15) ||
			 //  (command.getCompassHeading() >= yawMapping - 15 + 180 && command.getCompassHeading() >= yawMapping + 15 + 180)){
					command.picture();
					loopCounter1 = 0;
			//}
			//cout<<"Compass="<<command.getCompassHeading()<<" yaw="<<yawMapping<<endl;
		}
		
		loopCounter++;
		loopCounter1++;
		ros::spinOnce();
		loop_rate.sleep();
	}	
	
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
	
	latitude[ile] = homeLatitude;
	longitude[ile] = homeLongitude;
	
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
		
		if(i >= ile + 1){ //IS IN HOME POSITION?
			currentState = LAND_HOME;
			dronAltitude = 5;
			command.flyTo(homeLatitude, homeLongitude, dronAltitude);
			return;
		}
		
		if(i % 2 == 1)isMapping = true;
		else isMapping = false; 
		
		command.flyTo(latitude[i], longitude[i], missionAltitude);
	}
}

void landHome(mavrosCommand command){
	
	if(!precisionLanding){
		
		cout<<"CURRENT ALTITUDE: "<< command.getGlobalPositionAltitude() - homeAltitude<<endl;	
		if(command.getGlobalPositionAltitude() - homeAltitude <= 5){
			precisionLanding = true;
			sleep(3);
			command.land();
		}
		else{
			 dronAltitude = dronAltitude - 0.1;
			 command.flyTo(homeLatitude, homeLongitude, dronAltitude);
		 }
	}
	else{
		cout<<command.getArmed()<<endl;
		if(!command.getArmed())currentState = END;
	}
}

bool getCordinates(){
	ifstream theFile("mission.txt");
	
	double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0, x4 = 0.0, y4 = 0.0;
	
	char comma;
	
	int ile_x = 2; //longer side
	int ile_y;
	
	int ik,jk, kierunek = 1;
 	double x, x_wsp_14, x_wsp_12, x_pom;
 	double y, y_wsp_14, y_wsp_12, y_pom;
 	
	theFile >> missionAltitude >> ile_y >> pictureFrequency >> x1 >> comma >> y1 >> x2 >> comma >> y2 >> x4 >> comma >> y4;
	theFile.close();
	
	pictureFrequency = pictureFrequency * 20;
	
	if(missionAltitude == 0 || x1 == 0 || x2 == 0 || x4 == 0 || y1 == 0 || y2 == 0 || y4 == 0)return false;  		 
	
	yawMapping = atan( (y2-y1)*0.67 / (x2-x1)*1.11 ) * 180/PI;

	if(y2-y1>=0 && x2-x1==0)yawMapping = 90;
	else if(x2-x1<0)yawMapping = 180 + yawMapping;
	else if(y2-y1<0  && x2-x1==0)yawMapping = 270;
	else if(y2-y1<0  && x2-x1>0)yawMapping = 360 + yawMapping;
	
	yawMapping = yawMapping % 360;
				
 	x_wsp_12 = x2-x1;
	y_wsp_12 = y2-y1;
	x_wsp_14 = x4-x1;
	y_wsp_14 = y4-y1;
	x_wsp_12=x_wsp_12/(ile_x - 1);
	y_wsp_12=y_wsp_12/(ile_x - 1);
	x_wsp_14=x_wsp_14/(ile_y - 1);
	y_wsp_14=y_wsp_14/(ile_y - 1);
	x_pom = x1;
	y_pom = y1;
 	
 	for(jk=0;jk<ile_y;jk++){
		if(kierunek == 1){
		x = x_pom;
		y = y_pom;
		latitude[ile] = x;
		longitude[ile] = y;
		ile++;
			for(ik=1;ik<ile_x;ik++){
				x = x_pom + ik*x_wsp_12;
				y = y_pom + ik*y_wsp_12;
				latitude[ile] = x;
				longitude[ile] = y;
				ile++;
					}
					x_pom = x + x_wsp_14;
					y_pom = y + y_wsp_14;
					kierunek = 2;
	   }
		else if(kierunek == 2){
			x = x_pom;
			y = y_pom;
			latitude[ile] = x;
			longitude[ile] = y;
			ile++;
			for(ik=1;ik<ile_x;ik++){
				x = x_pom - ik*x_wsp_12;
				y = y_pom - ik*y_wsp_12;
				latitude[ile] = x;
				longitude[ile] = y;
				ile++;
					}
					x_pom = x + x_wsp_14;
					y_pom = y + y_wsp_14;
					kierunek = 1;
	   }
	}
	
	return true;
}


