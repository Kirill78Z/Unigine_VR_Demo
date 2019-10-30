#pragma once
#include <UnigineWorlds.h>
#include "TrafficSimulation.h"
#include "Ñarriageway.h"
#include <list>

class Vehicle;

struct Position3D
{
	Unigine::SplineSegmentPtr splSegment;

	Unigine::Math::dvec3 absPos = Unigine::Math::dvec3::ZERO;
	Unigine::Math::vec3 tangent = Unigine::Math::vec3::ZERO;
	Unigine::Math::vec3 up = Unigine::Math::vec3::ZERO;
};

struct LinearPosition
{
	double segStartLinearPos = -1;
	Unigine::SplineSegmentPtr splSegment;
	double absLinearPos = -1;


	static LinearPosition Null() {
		LinearPosition lp = LinearPosition(Unigine::SplineSegmentPtr::Ptr(), -1, -1);
		return lp;
	}

	LinearPosition(Unigine::SplineSegmentPtr splSegment,
		double segStartLinearPos, double absLinearPos) {
		this->segStartLinearPos = segStartLinearPos;
		this->splSegment = splSegment;
		this->absLinearPos = absLinearPos;
	}


	bool isEmpty() {
		if (splSegment) {
			return segStartLinearPos == -1;
		}
		else
		{
			return true;
		}
	}


	float distOnSplineSeg() {
		float distOnSplineSeg = (float)(absLinearPos - segStartLinearPos);

		assert(distOnSplineSeg > -UNIGINE_EPSILON);

		if (Unigine::Math::abs(distOnSplineSeg) < UNIGINE_EPSILON)
			distOnSplineSeg = 0;

		return distOnSplineSeg;
	}


	//returns true if end of spline graph reached
	bool increaseLinearPos(double s) {
		absLinearPos += s;


		//splSegment
		float distOnSplineSeg = this->distOnSplineSeg();

		while (distOnSplineSeg > splSegment->getLength())
		{
			//move to next segment
			Unigine::SplinePointPtr pt = splSegment->getEndPoint();

			assert(pt->getNumSegments() <= 2);

			if (pt->getNumSegments() >= 2) {
				Unigine::Vector<Unigine::SplineSegmentPtr> segments;
				pt->getSplineSegments(segments);
				Unigine::SplineSegmentPtr nextSeg;
				for (int s = 0; s < segments.size(); s++) {
					Unigine::SplineSegmentPtr seg = segments[s];
					if (seg != splSegment) {
						nextSeg = seg;
						break;
					}

				}

				assert(nextSeg);

				segStartLinearPos += splSegment->getLength();
				splSegment = nextSeg;
				distOnSplineSeg = this->distOnSplineSeg();

			}
			else
			{
				return true;
			}
		}

		return false;

	}





	Position3D getPos3D() {
		//TODO
		float distOnSplineSeg = this->distOnSplineSeg();

		float segLen = splSegment->getLength();

		assert(distOnSplineSeg < segLen);

		float linearParam = distOnSplineSeg / segLen;

		float nonLinearParam = splSegment->linearToParametric(linearParam);
		Unigine::Math::dvec3 pt = splSegment->calcPoint(nonLinearParam);
		Unigine::Math::vec3 tan = splSegment->calcTangent(nonLinearParam);
		Unigine::Math::vec3 up = splSegment->calcUpVector(nonLinearParam);

		Position3D vp;
		vp.splSegment = splSegment;
		vp.absPos = pt;
		vp.tangent = tan;
		vp.up = up;
		return vp;
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
	~TrafficLane();

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
	double overalLength;
	bool leadToEndOfRoad = true;

	TrafficSimulation* trafficSim;
	Ñarriageway* carriageway;
	int _num;

	//vehicles on this lane
	std::list<Vehicle*> vehicles;


	float timeToAddNewVehicle = 0;
	float timeSpanBetweenAddingVehicles = 1.0f;

	Vehicle* waitingVehicle = nullptr;

	void startNewVehicle(Vehicle * &vehicle, float velocity);

};

