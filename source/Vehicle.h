#pragma once

#include "Сarriageway.h";
#include "MainLane.h";


enum VehicleActivity
{
	StraightMove,
	Wait,
	ChangeLine
};


class Vehicle
{
public:
	Vehicle(Сarriageway* carriageway,
		int trafficLaneNum, Unigine::NodeDummyPtr node,
		float speedLimit, LinearPosition startLinPos);
	~Vehicle();

	void update();

	double getClearDist(std::list<Vehicle*>::iterator nextIt, 
		LinearPosition linPos, ObstacleType &obstacleType, LinearPosition &obstacleLP);

	void setIterator(std::list<Vehicle*>::iterator it) {
		vehicleIterator = it;
	}

	bool isReachedEndOfRoad() {
		return _reachedEndOfRoad;
	}

	LinearPosition getCurrPosOnLane() {
		return currLinearPosOnLane;
	}

	float getVelocityToFitIntoSpan(double span);

	void setVelocity(float v) {
		velocity = v;
	}

	void setEnabled(int e) {
		node->setEnabled(e);
	}

	float getSpeedLimit() {
		return speedLimit;
	}

private:
	const float reserveDistBetweenCars = 2.0f;

	//measurement unit - meter/(sec^2)
	const float standartAcceleration = 7.0f;
	const float standartDamping = 7.0f;//TODO?: 5.5 for tracks by Babkov's textbook...

	const float timeToWaitOnPaymentCollectionPoint = 0.5;

	Сarriageway* carriageway;
	int trafficLane;
	MainLane* getCurrTrafficLane() {
		//TODO: переработать способ получения текущей полосы с учетом хранения дополнительных полос уширения 
		return carriageway->getTrafficLane(trafficLane);
	}

	Unigine::NodeDummyPtr node;

	void moveOnPos(Position3D pos) {
		node->setWorldPosition(pos.absPos);
		node->setWorldDirection(pos.tangent, pos.up, Unigine::Math::AXIS_Y);
	}

	//iterator pointing position in traffic lane list
	/*Adding, removing and moving the elements within the list or across several lists does not invalidate the iterators or references.
	An iterator is invalidated only when the corresponding element is deleted.*/
	//https://en.cppreference.com/w/cpp/container/list
	//use splice to move across several lists -- https://stackoverflow.com/a/48330737/8020304
	std::list<Vehicle*>::iterator vehicleIterator;


	//car length
	float length = 0;

	//dynamic envelop of vehicle depending of current velocity
	float getDynamicEnvelop() {
		return length * 2 + Unigine::Math::pow(velocity, 2) / (2 * standartDamping);
	}

	LinearPosition currLinearPosOnLane = LinearPosition::Null();
	

	//set true if last point of spline graph reached
	bool _reachedEndOfRoad = false;

	//current position for all lanes
	Unigine::HashMap<int, int> currentSegments;
	//Unigine::HashMap<int, double> currentLinearPositions;


	float velocity;
	float speedLimit;

	
	VehicleActivity currentActivity = VehicleActivity::StraightMove;
	
	float timeToWait = 0;
	LinearPosition movingThroughObstacle = LinearPosition::Null();
	

};

