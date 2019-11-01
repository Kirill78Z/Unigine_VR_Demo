#pragma once
#include "TrafficLane.h"


class AdditionalLane;





class MainLane : public TrafficLane
{
public:
	MainLane(TrafficSimulation* trafficSim, Ñarriageway* carriageway, Unigine::WorldSplineGraphPtr node);
	~MainLane();

	void update() override;

	

	Position3D pointByAbsPosOnLane(LinearPosition linearPos);//????
	LinearPosition linearPosByPoint(Unigine::Math::dvec3 pt, int startSearchSegment);//TODO: use to get pos for parallel lines

	
	

private:
	
	Unigine::HashMap<int, float> vehProbability;


	//measurement unit - sec
	float timeToAddNewVehicle = 0;
	float timeSpanBetweenAddingVehicles = 100000.0f;

	Vehicle* waitingVehicle = nullptr;


	void getNewVehicleVelocity(Vehicle * vehicle, float &velocity, float speedLimit);

	void startNewVehicle(Vehicle * &vehicle, float velocity);

};

