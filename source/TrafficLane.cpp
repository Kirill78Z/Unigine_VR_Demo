#include "TrafficLane.h"
#include "Vehicle.h"
#include "AdditionalLane.h"

TrafficLane::TrafficLane(TrafficSimulation* trafficSim, Ñarriageway* carriageway, Unigine::WorldSplineGraphPtr node)
{
	worldSplineGraph = node;
	this->trafficSim = trafficSim;
	this->carriageway = carriageway;

	//validate WorldSplineGraph: all points must be connected one by one from first to last with gap where there obstacle
	//prepare list of spline segments from start to end
	//create splines where there are obstacles and save obstacle position
	Unigine::Vector<Unigine::SplinePointPtr> points;
	worldSplineGraph->getSplinePoints(points);
	worldSplineGraph->getSplineSegments(segments);

	int prevSegEndIndex = 1;
	Unigine::SplineSegmentPtr prevSeg = segments[0];
	if (prevSeg->getEndPoint() != points[1]) {
		throw std::exception("Invalid spline graph");//something wrong
	}
	LinearPosition startLp = LinearPosition(segments[0], 0, 0);
	segmentPositions.append(startLp);
	double currLinearPos = prevSeg->getLength();
	int startSegCount = segments.size();
	for (int s = 1; s < startSegCount; s++) {
		Unigine::SplineSegmentPtr seg = segments[s];

		float len = seg->getLength();
		double lenPts = (seg->getStartPoint()->getPosition() - seg->getEndPoint()->getPosition()).length();

		if (seg->getStartPoint() != points[prevSegEndIndex])
		{
			//there is gap
			if (seg->getStartPoint() == points[++prevSegEndIndex]) {
				//gap in one segment - obstacle
				LinearPosition obstacleLp = LinearPosition(
					segments[s - 1], currLinearPos, currLinearPos);
				obstacles.append(obstacleLp);

				//add segment to close this gap
				Unigine::SplineSegmentPtr closingGapSeg
					= worldSplineGraph->createSplineSegment(
						points[prevSegEndIndex - 1], -prevSeg->getEndTangent(), prevSeg->getEndUp(),
						seg->getStartPoint(), -seg->getStartTangent(), seg->getStartUp());
				segments.append(closingGapSeg);

				//segmentPositions
				LinearPosition additionalLp =
					LinearPosition(segments[segments.size() - 1],
						currLinearPos, currLinearPos);
				segmentPositions.append(additionalLp);

				currLinearPos += closingGapSeg->getLength();
			}
			else
			{
				throw std::exception("Invalid spline graph");//something wrong
			}
		}

		//segmentPositions
		LinearPosition lp = LinearPosition(segments[s], currLinearPos, currLinearPos);
		segmentPositions.append(lp);

		currLinearPos += seg->getLength();
		prevSegEndIndex++;
		prevSeg = seg;
	}

	overalLength = currLinearPos;

	//get additional lanes
	std::list< TrafficLane*> additionalLanes;
	for (int c = 0; c < worldSplineGraph->getNumChildren(); c++) {
		Unigine::NodePtr cn = worldSplineGraph->getChild(c);
		if (cn->getType() == Unigine::Node::WORLD_SPLINE_GRAPH) {
			//additional line founded
			AdditionalLane* additionalLane = new AdditionalLane(
				trafficSim, carriageway, Unigine::WorldSplineGraph::cast(cn));

			additionalLanes.push_back(additionalLane);
		}
	}

	Position3D pos = this->startOfLane();

	//get additional lanes linear spans
	scanNeighboringLanes(additionalLanes);

}

void TrafficLane::scanNeighboringLanes(std::list<TrafficLane *> &neighboringLanes)
{
	LinearSpan* currentLSLeft = nullptr;
	LinearSpan* currentLSRight = nullptr;
	for (int s = 0; s < segmentPositions.size(); s++) {
		if (currentLSLeft == nullptr || currentLSRight == nullptr) {
			//searching for start points of lanes
			std::list<TrafficLane*>::iterator i = neighboringLanes.begin();
			while (i != neighboringLanes.end()) {//https://stackoverflow.com/a/596180/8020304
				Position3D pos = (*i)->startOfLane();
				LinearPosition lp = LinearPosition(segmentPositions[s]);
				int checkResult = pos.isParallelLineInFrontOf(&lp);
				if (checkResult != 0) {
					//start of lane founded
					LinearSpan* ls = nullptr;

					ls = new LinearSpan;
					ls->start = lp;
					ls->dataType = LaneType::AdditionalLane_;
					ls->data = (*i);


					LinearSpan lsThisLane;
					lsThisLane.start = (*i)->startOfLaneLinear();
					lsThisLane.end = (*i)->endOfLaneLinear();
					lsThisLane.dataType = this->laneType;
					lsThisLane.data = this;

					if (checkResult == -1) {
						assert(currentLSLeft == nullptr);
						currentLSLeft = ls;
						(*i)->lanesToTheRight.append(lsThisLane);
					}
					else {
						assert(currentLSRight == nullptr);
						currentLSRight = ls;
						(*i)->lanesToTheLeft.append(lsThisLane);
					}


					neighboringLanes.erase(i++);
				}
				else
				{
					++i;
				}
			}
		}

		if (currentLSLeft != nullptr || currentLSRight != nullptr) {
			//searching for end points of lanes
			Unigine::Vector< LinearSpan*> lanesToSearchEnd;
			if (currentLSLeft != nullptr)
				lanesToSearchEnd.append(currentLSLeft);
			if (currentLSRight != nullptr)
				lanesToSearchEnd.append(currentLSRight);

			for (int l = 0; l < lanesToSearchEnd.size(); l++) {
				LinearSpan* ls = lanesToSearchEnd[l];
				Position3D endPos = ((AdditionalLane*)ls->data)->endOfLane();
				LinearPosition lp = LinearPosition(segmentPositions[s]);

				Position3D test = lp.getPos3D();

				int checkResult = endPos.isParallelLineInFrontOf(&lp);
				if (checkResult != 0) {
					//end of lane founded
					ls->end = lp;

					if (checkResult == -1) {
						assert(currentLSLeft == ls);
						lanesToTheLeft.append(*currentLSLeft);
						currentLSLeft = nullptr;
					}
					else
					{
						assert(currentLSRight == ls);
						lanesToTheRight.append(*currentLSRight);
						currentLSRight = nullptr;
					}



				}

			}


		}

	}
}


TrafficLane::~TrafficLane()
{
}


void TrafficLane::update() {
	//update all vehicles on lane
	Unigine::Vector<std::list<Vehicle*>::iterator> toErase;
	std::list<Vehicle*>::iterator it;
	for (it = vehicles.begin(); it != vehicles.end(); ++it) {
		if (!(*it)->isReachedEndOfRoad()) {
			(*it)->update();
		}
		else
		{
			toErase.append(it);
		}

	}

	//remove from list end delete
	for (int i = 0; i < toErase.size(); i++) {
		delete (*toErase[i]);
		vehicles.erase(toErase[i]);
	}
}


LinearPosition TrafficLane::getNextObstacle(LinearPosition pos, bool ignoreFirst) {
	LinearPosition empty = LinearPosition::Null();

	if (obstacles.size() == 0)
	{

		return empty;
	}

	//binary search for nearest obstacle ahead
	int lpIndex = searchNearestLinearPosAhead(obstacles, 0, obstacles.size() - 1, pos);

	if (lpIndex >= 0) {
		return !ignoreFirst ? obstacles[lpIndex] :
			lpIndex < obstacles.size() - 1 ? obstacles[lpIndex + 1] : empty;
	}
	else
	{

		return empty;
	}

}




