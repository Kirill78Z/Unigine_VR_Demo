#pragma once
#include "TrafficSimulation.h"
#include "Ñarriageway.h"
#include <list>
#include <UnigineWorlds.h>
#include "LinearPosition.h"
#include "Position3D.h"

class Vehicle;
class AdditionalLane;
class MainLane;


enum LaneType
{
	MainLane_,
	AdditionalLane_
};

struct LinearSpan {
	LinearPosition start;
	LinearPosition end;


	LaneType dataType;
	void* data;

	bool isNull() {
		return start.isEmpty();
	}
};

enum ObstacleType
{
	None,
	MovingVehicle,
	EndOfLane,
	PaymentCollectionPoint
};


class TrafficLane
{
public:
	TrafficLane(TrafficSimulation* trafficSim, Ñarriageway* carriageway, Unigine::WorldSplineGraphPtr node);
	void scanNeighboringLanes(std::list<TrafficLane *> &additionalLanes);
	~TrafficLane();

	void virtual update();

	Position3D startOfLane()
	{
		Position3D vp;
		vp.splSegment = segments[0];
		vp.absPos = segments[0]->getStartPoint()->getPosition();
		vp.tangent = segments[0]->getStartTangent();
		vp.up = segments[0]->getStartUp();
		return vp;
	}

	Position3D endOfLane() {
		Position3D vp;
		vp.splSegment = segments[segments.size() - 1];
		vp.absPos = vp.splSegment->getEndPoint()->getPosition();
		vp.tangent = vp.splSegment->getEndTangent();
		vp.up = vp.splSegment->getEndUp();
		return vp;
	}

	LinearPosition startOfLaneLinear() {
		//generate new struct
		LinearPosition lp = LinearPosition(segments[0], 0, 0);
		return lp;
	}

	LinearPosition endOfLaneLinear() {
		//generate new struct
		LinearPosition lp = LinearPosition(segmentPositions[segmentPositions.size() - 1]);
		lp.increaseLinearPos(lp.splSegment->getLength());
		return lp;
	}


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

protected:

	LaneType laneType = LaneType::MainLane_;


	Unigine::WorldSplineGraphPtr worldSplineGraph;

	//vehicles on this lane
	std::list<Vehicle*> vehicles;

	Unigine::Vector<Unigine::SplineSegmentPtr> segments;
	Unigine::Vector<LinearPosition> segmentPositions;
	Unigine::Vector<LinearPosition> obstacles;//Immovable obstacles. Currently it is PaymentCollectionPoint only

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

	//neighboring lanes
	Unigine::Vector<LinearSpan> lanesToTheLeft;
	Unigine::Vector<LinearSpan> lanesToTheRight;

	double overalLength;

	TrafficSimulation* trafficSim;
	Ñarriageway* carriageway;

	bool leadToEndOfRoad = false;
};

