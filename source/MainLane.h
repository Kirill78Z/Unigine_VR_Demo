#pragma once
#include "TrafficLane.h"


class AdditionalLane;



enum ObstacleType
{
	None,
	MovingVehicle,
	EndOfLane,
	PaymentCollectionPoint
};

class MainLane
{
public:
	MainLane(TrafficSimulation* trafficSim, Ñarriageway* carriageway, Unigine::WorldSplineGraphPtr node);
	~MainLane();

	void update();

	void getNewVehicleVelocity(Vehicle * vehicle, float &velocity, float speedLimit);



	/*Unigine::WorldSplineGraphPtr getWorldSplineGraph() {
		return worldSplineGraph;
	}*/

	void setNum(int num) {
		_num = num;
	}


	Position3D pointByAbsPosOnLane(LinearPosition linearPos);
	Position3D startOfLane()
	{
		Position3D vp;
		vp.splSegment = segments[0];
		vp.absPos = segments[0]->getStartPoint()->getPosition();
		vp.tangent = segments[0]->getStartTangent();
		vp.up = segments[0]->getStartUp();
		return vp;
	}

	LinearPosition startOfLaneLinear() {
		//generate new struct
		LinearPosition lp = LinearPosition(segments[0], 0, 0);
		return lp;
	}


	LinearPosition linearPosByPoint(Unigine::Math::dvec3 pt, int startSearchSegment);//TODO: use to get pos for parallel lines

	//end of vehicle list
	std::list<Vehicle*>::iterator getQueueEnd() {
		return vehicles.end();
	}

	LinearPosition getNextObstacle(LinearPosition pos, bool ignoreFirst);

	double getOveralLength() {
		return overalLength;
	}

	bool getLeadToEndOfRoad() {
		return leadToEndOfRoad;
	}

private:
	Unigine::WorldSplineGraphPtr worldSplineGraph;

	Unigine::Vector<AdditionalLane*> additionalLanes;


	Unigine::Vector<Unigine::SplineSegmentPtr> segments;
	Unigine::Vector<LinearPosition> segmentPositions;

	int searchNearestLinearPosAhead(
		Unigine::Vector<LinearPosition> arr, int first, int last, LinearPosition searchingPos)
	{
		if (last >= first) {
			int mid = first + (last - first) / 2;
			LinearPosition lp = arr[mid];

			if (searchingPos.absLinearPos <= lp.absLinearPos) {
				if (mid == 0 || searchingPos.absLinearPos > arr[mid - 1].absLinearPos) {
					return mid; //we find nearest position ahead
				}
				//search in left subarray
				return searchNearestLinearPosAhead(arr, first, mid - 1, searchingPos);
			}
			else
			{
				//search in right subarray
				return searchNearestLinearPosAhead(arr, mid + 1, last, searchingPos);
			}
		}
		else
		{
			return -1;
		}
	}

	int searchNearestLinearPosÂehind(
		Unigine::Vector<LinearPosition> sortedArr, int first, int last, LinearPosition searchingPos)
	{
		if (last >= first) {
			int mid = first + (last - first) / 2;
			LinearPosition lp = sortedArr[mid];

			if (searchingPos.absLinearPos < lp.absLinearPos) {
				//search in left subarray
				return searchNearestLinearPosÂehind(sortedArr, first, mid - 1, searchingPos);
			}
			else
			{
				if (mid == sortedArr.size() - 1 || searchingPos.absLinearPos < sortedArr[mid + 1].absLinearPos) {
					return mid; //we find nearest position behind
				}

				//search in right subarray
				return searchNearestLinearPosÂehind(sortedArr, mid + 1, last, searchingPos);
			}
		}
		else
		{
			return -1;
		}
	}



	//Immovable obstacles. Currently it is PaymentCollectionPoint only
	Unigine::Vector<LinearPosition> obstacles;


	Unigine::HashMap<int, float> vehProbability;


	double overalLength;
	bool leadToEndOfRoad = true;

	TrafficSimulation* trafficSim;
	Ñarriageway* carriageway;
	int _num;

	//vehicles on this lane
	std::list<Vehicle*> vehicles;

	//measurement unit - sec
	float timeToAddNewVehicle = 0;
	float timeSpanBetweenAddingVehicles = 100000.0f;

	Vehicle* waitingVehicle = nullptr;

	void startNewVehicle(Vehicle * &vehicle, float velocity);

};

