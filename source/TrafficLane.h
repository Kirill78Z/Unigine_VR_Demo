#pragma once
#include "TrafficSimulation.h"
#include "Сarriageway.h"
#include <UnigineWorlds.h>
#include "LinearPosition.h"
#include "Position3D.h"

class AdditionalLane;
class MainLane;



struct LinearSpan {
	LinearPosition start;
	LinearPosition end;

	TrafficLane* data;

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
	TrafficLane(TrafficSimulation* trafficSim, Сarriageway* carriageway,
		Unigine::WorldSplineGraphPtr node);
	//void scanNeighboringLanes(std::list<TrafficLane *> &additionalLanes);
	~TrafficLane();

	void virtual update();

	Position3D startOfLane()
	{
		Position3D vp = Position3D(segments[0], segments[0]->getStartPoint()->getPosition(),
			segments[0]->getStartTangent(), segments[0]->getStartUp());
		return vp;
	}

	Position3D endOfLane() {
		int last = segmentPositions.size() - 1;

		Unigine::SplineSegmentPtr lastSeg = segmentPositions[last].splSegment;

		Unigine::Math::dvec3 lastPt = lastSeg->getEndPoint()->getPosition();

		Unigine::Math::vec3 tan = lastSeg->getEndTangent();
		Unigine::Math::vec3 up = lastSeg->getEndUp();

		Position3D vp = Position3D(lastSeg, lastPt, tan, up);
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
		assert(lp.increaseLinearPos(lp.splSegment->getLength()));
		return lp;
	}


	//end of vehicle list
	std::list<Vehicle*>::iterator getQueueEnd() {
		return vehicles.end();
	}

	std::list<Vehicle*>::iterator getQueueStart() {
		return vehicles.begin();
	}

	LinearPosition getNextObstacle(LinearPosition pos, bool ignoreFirst);

	double getOveralLength() {
		return overalLength;
	}

	bool getLeadToEndOfRoad() {
		return leadToEndOfRoad;
	}

	int getNumFromLeftToRight() {
		return numFromLeftToRight;
	}

	void calcNeighborLanesLinearSpans();


	void addNeighborLaneLinearSpan(LinearSpan* ls, bool right) {
		if (right) {
			lanesToTheRight.push_back(ls);
		}
		else
		{
			lanesToTheLeft.push_back(ls);
		}
	}

	std::list<LinearSpan*>::iterator lanesToTheLeftBegin() {
		return lanesToTheLeft.begin();
	}

	std::list<LinearSpan*>::iterator lanesToTheRightBegin() {
		return lanesToTheRight.begin();
	}

	std::list<LinearSpan*>::iterator lanesToTheLeftEnd() {
		return lanesToTheLeft.end();
	}

	std::list<LinearSpan*>::iterator lanesToTheRightEnd() {
		return lanesToTheRight.end();
	}


	double getTransitionLengthStart() {
		return transitionLengthStart;
	}

	double getTransitionLengthEnd() {
		return transitionLengthEnd;
	}

	//returns array of two iterators
	void TrafficLane::getNextAndPrevVehicles(
		LinearPosition lp, std::list<Vehicle*>::iterator* result);


	std::list<Vehicle*>::iterator createTempChangeLaneIterator
	(Vehicle* vehChangingLane, std::list<Vehicle*>::iterator itAhead)
	{
		return vehicles.insert(itAhead, vehChangingLane);
	}


	void replaceVehicleItTo(std::list<Vehicle*>::iterator to, TrafficLane* from, std::list<Vehicle*>::iterator it) {
		vehicles.splice(to, from->vehicles, it);
	}

	void deleteTempChangeLaneIt(std::list<Vehicle*>::iterator tempIt) {
		carriageway->deletedTempChangeLaneIts.splice
		(carriageway->deletedTempChangeLaneIts.end(), vehicles, tempIt);
	}


	void openBarrier() {
		barrierIsOpened = true;
		/*if (barrier) {
			Unigine::Math::dmat4 rotM = Unigine::Math::rotate(Unigine::Math::dvec3(rotationAxis), 90);
			barrier->setTransform(rotM*initialTransf);
		}*/

	}

	void closeBarrier() {
		barrierIsOpened = false;
		/*if (barrier) {
			barrier->setTransform(initialTransf);
		}*/
	}

protected:

	//vehicles
	Unigine::HashMap<int, float> vehProbability;
	//measurement unit - sec
	float timeToAddNewVehicle = 0;
	float timeSpanBetweenAddingVehicles = 100000.0f;

	Vehicle* waitingVehicle = nullptr;

	void getNewVehicleVelocity(Vehicle * vehicle, float &velocity, float speedLimit);
	void startNewVehicle(Vehicle * &vehicle, float velocity);



	//DataType laneType = DataType::MainLane_;


	Unigine::WorldSplineGraphPtr worldSplineGraph;

	//vehicles on this lane
	std::list<Vehicle*> vehicles;

	Unigine::Vector<Unigine::SplineSegmentPtr> segments;
	Unigine::Vector<LinearPosition> segmentPositions;
	Unigine::Vector<LinearPosition> obstacles;//Immovable obstacles. Currently it is PaymentCollectionPoint only

	//barrier
	//TODO:Перенести в отдельный класс. На одной полосе может быть несколько ПВП
	Unigine::NodePtr barrier;
	Unigine::NodePtr rotationPt;
	Unigine::Math::dmat4 initialTransf;
	float barrierCurrRotation = 0;
	Unigine::Math::vec3 rotationAxis = Unigine::Math::vec3::ZERO;
	bool barrierIsOpened = false;
	const float rotationAngleVelocity = 90;
	//barrier

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

	int searchNearestLinearPosВehind(
		Unigine::Vector<LinearPosition> sortedArr, int first, int last, LinearPosition searchingPos)
	{
		if (last >= first) {
			int mid = first + (last - first) / 2;
			LinearPosition lp = sortedArr[mid];

			if (searchingPos.absLinearPos < lp.absLinearPos) {
				//search in left subarray
				return searchNearestLinearPosВehind(sortedArr, first, mid - 1, searchingPos);
			}
			else
			{
				if (mid == sortedArr.size() - 1 || searchingPos.absLinearPos < sortedArr[mid + 1].absLinearPos) {
					return mid; //we find nearest position behind
				}

				//search in right subarray
				return searchNearestLinearPosВehind(sortedArr, mid + 1, last, searchingPos);
			}
		}
		else
		{
			return -1;
		}
	}


	int numFromLeftToRight = 0;
	//neighboring lanes
	void calcNeighborLanesLinearSpansOneSide(Unigine::Vector<TrafficLane*> lanes, bool right);

	std::list<LinearSpan*> lanesToTheLeft;
	std::list<LinearSpan*> lanesToTheRight;




	double overalLength;

	TrafficSimulation* trafficSim;
	Сarriageway* carriageway;

	bool leadToEndOfRoad = false;


	//дополнительные расстояния, которые можно использовать 
	//для перестроения на эту полосу в начале и в конце полосы
	double transitionLengthStart = 0;
	double transitionLengthEnd = 0;
};

