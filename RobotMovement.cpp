// Robot Movement in a 2D grid space and execution of tasks.

#include <iostream>
#include <string>
#include <cmath>

using namespace std;


class Point {
public:
	
	Point();    // Make a point at origin

	// Make a point at (x,y). If a coordinate is out of the range [0, 99], that coord is set to 0.
	Point(int x, int y);
	
	int getX();  // Get the x coordinate
	
	int getY();  // Get the y coordinate

private:
	
	int xPos;   // x coordinate
	
	int yPos;   // y coordinate
};

class Robot {
public:
	// Make a robot with the given name located at origin (0,0) with 100 energy units
	Robot(string name);

	// Make a robot with the given name located at defined coordinates (x,y) with 100 energy units
	Robot(string name, int x, int y);

	string getName();    // Get the robot name
	
	int getPosX();       // Get the x position
	
	int getPosY();       // Get the y position
	
	int getEnergyLevel();                 // Get the current energy level

	// Increment the energy level by the given energy amount
	void charge(int energyUnits);

	// Set the destination where the task needs to be executed
	void setDestination(int x, int y);

	int getDestX();                       // Get the x coord of the destination
	
	int getDestY();                       // Get the y coord of the destination
	
	void setTask(string task);            // Set the task description
	
	string getTask();                     // Get the task description
	/*
	 1. Execute the task only if the robot has enough energy to implement the task. Return true if successful.
	 2. Energy is used up at 1 unit per grid square movement. The robot can only move horizontally or vertically 
	    in the grid (not diagonally) and 1 square move costs 1 energy unit.
	 3. If the robot does not have enough energy, it does not move and no energy is used up. It prints out
	    how many units it is short of energy to carry out the task and returns false.
	 4. This function must use the distanceToDestination function to calculate how much energy is needed/used up.
	 */

	bool executeTask();
	
	void status();          // Print the current status of the robot

	// Transfer the task and all the energy to the other robot
	void transferTaskToFriend(Robot& otherRobot);

private:
	
	string name;    // robot name
	
	Point currentPosition;   // current position in the grid

	// Destination in the grid where the task needs to be executed
	Point destinationPosition;

	int energyLevel;           // Energy level

	string task;              // Task description

	// Function to calculate the distance from the current position to the destination.
	int distanceToDestination();
};

int main() {

	Robot  optimusprime("OptimusPrime");             
	optimusprime.status();

	Robot bumblebee("Bumblebee", 10, 10);
	bumblebee.status();
	bumblebee.setDestination(88, 50);
	bumblebee.setTask("Drive Around");
	bumblebee.executeTask();
	bumblebee.status();

	Robot jazz("Jazz", 50, 10);
	jazz.status();
	bumblebee.transferTaskToFriend(jazz);
	jazz.executeTask();
	jazz.status();
	bumblebee.status();

	Robot megatron("Megatron", 75, 15);
	megatron.status();
	megatron.setDestination(80,10);
	megatron.setTask("Transformer to Jet");
	megatron.executeTask();
	megatron.status();
	
	Robot bonecrusher("Bonecrusher", 24, 52);
	bonecrusher.status();
	bonecrusher.setDestination(61,39);
	bonecrusher.setTask("Mine Sweeper");
	bonecrusher.executeTask();
	bonecrusher.status();

	Robot frenzy("Frenzy", 16, 04);
	frenzy.status();
	frenzy.setDestination(0,88);
	frenzy.setTask("Hack systems");
	frenzy.executeTask();
	frenzy.status();
	
	system("pause");    // Assigning tasks and exercise functionality
	return 0;
}

// Point class implementation
Point::Point() {
	xPos = yPos = 0;
}

Point::Point(int x, int y) {
	if (x < 0 || x > 99) {
		x = 0;
	}
	if (y < 0 || y > 99) {
			y = 0;
	}
	xPos = x;
	yPos = y;
}

int Point::getX() {
	return xPos;
}

int Point::getY() {
	return yPos;
}

// Robot class implementation
Robot::Robot(string name) {
	this->name = name;
	currentPosition = Point(0, 0);
	destinationPosition = Point(0, 0);
	energyLevel = 100;
	task = "none";
}


  // Implement additional Robot functions

	Robot::Robot(string name, int x, int y){
		this->name = name;
		currentPosition = Point(x, y);
		destinationPosition = Point(x, y);
		energyLevel = 100;
		task = "none";
	}

	// Get the robot name
	string Robot::getName(){
		return name;
	}

	// Get the x position
	int Robot::getPosX(){
		return currentPosition.getX();
	}
	
	// Get the y position
	int Robot::getPosY(){
		return currentPosition.getY();
	}
	
	// Get the current energy level
	int Robot::getEnergyLevel(){
		return energyLevel;
	}

	// Increment the energy level by the given energy amount
	void Robot::charge(int energyUnits){
		energyLevel += energyUnits;
	}

	// Set the destination where the task needs to be executed
	void Robot::setDestination(int x, int y){
		Point des (x,y);
		destinationPosition = des;
	}

	// Get the x coord of the destination
	int Robot::getDestX(){
		return destinationPosition.getX();
	}

	// Get the y coord of the destination
	int Robot::getDestY(){
		return destinationPosition.getY();
	}

	// Set the task description
	void Robot::setTask(string task){
		this->task=task;
	}

	// Get the task description
	string Robot::getTask(){
		return task;
	}

	/*
	 1. Execute the task only if the robot has enough energy to carry it out. Return true if successful.
	 2. Energy is used up at 1 unit per grid square movement. The robot can only move horizontally or vertically 
	    in the grid (not diagonally) and 1 square move costs 1 energy unit.
	 3. If the robot does not have enough energy, it does not move and no energy is used up. It prints out
	    how many units it is short of energy to carry out the task and returns false.
	 4. This function must use the distanceToDestination function to calculate how much energy is needed/used up.
	 */

	bool Robot::executeTask(){

		if (distanceToDestination() > energyLevel){

			int totalEnergy = distanceToDestination() - energyLevel;
			cout<<"Destination is unreachable with current energy level."<<endl
				<<"This robot is "<<totalEnergy<<" energy units short and cannot complete task."<<endl;
			return false;
		}
		else{ 
			task=getTask();
			return true;
		}
	}

	// Print the current status of the robot
	void Robot::status(){

		cout<<"**************************************************************"<<endl
			<<name<<" located at "<<"("<<currentPosition.getX() <<","<<currentPosition.getY()<<")"<<endl
			<<"Task: "<<task<<endl
			<<"Destination: " <<"("<<destinationPosition.getX()<<","<<destinationPosition.getY()<<")"<<endl
			<<"Energy Level: "<<energyLevel<<endl
			<<"**************************************************************"<<endl;
	}

	// Transfer the task and all the energy to the other robot
	void Robot::transferTaskToFriend(Robot& otherRobot)
		{
			otherRobot.setDestination(getDestX(),getDestY());
			otherRobot.setTask(task);
			otherRobot.charge(energyLevel);
			energyLevel=0;
		}

	int Robot::distanceToDestination(){
		return destinationPosition.getX() - currentPosition.getX() + destinationPosition.getY() - currentPosition.getY();
	}