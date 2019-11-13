#include "Vehicle.h"
#include <UnigineGame.h>




Vehicle::Vehicle(�arriageway* carriageway,
	TrafficLane* trafficLane, Unigine::NodeDummyPtr node,
	float speedLimit, LinearPosition startLinPos)
{
	this->carriageway = carriageway;
	this->trafficLane = trafficLane;
	laneToTheLeft = trafficLane->lanesToTheLeftBegin();
	laneToTheRight = trafficLane->lanesToTheRightBegin();
	this->node = node;
	this->speedLimit = speedLimit;

	//get length of node
	Unigine::WorldBoundBox bb = node->getWorldBoundBox();
	length = bb.getMax().y - bb.getMin().y;

	//insertion base point is center of back bottom edge of bounding box
	double xDisplacement = node->getPosition().x - (bb.getMin().x + bb.getMax().x) / 2;
	double yDisplacement = node->getPosition().y - bb.getMin().y;
	double zDisplacement = node->getPosition().z - bb.getMin().z;
	if (xDisplacement != 0 || yDisplacement != 0 || zDisplacement != 0) {
		for (int c = 0; c < node->getNumChildren(); c++) {
			Unigine::NodePtr child = node->getChild(c);
			child->setWorldPosition(child->getWorldPosition() + Unigine::Math::dvec3(xDisplacement, yDisplacement, zDisplacement));
		}
	}



	//place node into start position
	TrafficLane* lane = trafficLane;
	Position3D vp = lane->startOfLane();
	moveOnPos(vp);
	currLinearPosOnLane = startLinPos;

	//������� ������ WorldSplineGraph ��� ���������� ������������
	changeLaneTrack = Unigine::WorldSplineGraph::create();
	changeLaneTrack->setWorldTransform(carriageway->changeLaneTracksTransf);
#ifdef DEBUG 
	/*{
		Unigine::MeshPtr sphere = Unigine::Mesh::create();
		sphere->addSphereSurface("testMarker", 0.5f, 8, 8);

		testMarkerLeft = Unigine::ObjectMeshStatic::create(sphere);
		testMarkerLeft->release();
		Unigine::Editor::get()->addNode(testMarkerLeft->getNode());
		testMarkerLeft->setMaterialParameter("albedo_color", Unigine::Math::vec4(1, 0, 0, 1), 0);
		testMarkerLeft->setEnabled(0);

		sphere = Unigine::Mesh::create();
		sphere->addSphereSurface("testMarker", 0.5f, 8, 8);

		testMarkerRight = Unigine::ObjectMeshStatic::create(sphere);
		testMarkerRight->release();
		Unigine::Editor::get()->addNode(testMarkerRight->getNode());
		testMarkerRight->setMaterialParameter("albedo_color", Unigine::Math::vec4(1, 0, 0, 1), 0);
		testMarkerRight->setEnabled(0);
	}*/


	carriageway->vehn++;


	if (carriageway->vehn == 7) {
		testigVeh = true;
		carriageway->vehn = 0;
	}
#endif
}


Vehicle::~Vehicle()
{

}

void Vehicle::update() {

#ifdef DEBUG
	bool thisFrameChangedLane = false;
#endif // DEBUG


	TrafficLane* lane = trafficLane;

	float time = Unigine::Game::get()->getIFps();



performAction:
	switch (currentActivity)
	{
	case VehicleActivity::ChangeLane:
	{
		//����������� ����������� �������
		//posOnLaneAfterChange

		std::list<Vehicle*>::iterator nextIt = std::next(tempChangeLaneIt);
		while (nextIt != trafficLaneChangeTo->getQueueEnd()
			&& (*nextIt)->getCurrPosOnLane(trafficLaneChangeTo).absLinearPos < posOnLaneAfterChange.absLinearPos)
		{
			nextIt = std::next(nextIt);
		}


		ObstacleType obstacleType;
		LinearPosition obstacleLP = LinearPosition::Null();
		double clearDist = getClearDist(nextIt, posOnLaneAfterChange,
			obstacleType, obstacleLP, trafficLaneChangeTo, !movingThroughObstacle.isEmpty());
		float toMoveChangeLaneTrack = changeLaneTrackSeg->getLength() - currPosOnChangeLaneTrack;
		assert(toMoveChangeLaneTrack > 0);
		clearDist += toMoveChangeLaneTrack;

		//���� ������������ ���� �� ������, ������� ��� �� ��������,
		//�� ��������� ������, ������� ��������� ������� �� ������ ������
		//�� ������ ���� �� ����� �������� ����� ���� ������������
		std::list<Vehicle*>::iterator nextItOldLane = std::next(vehicleIterator);
		if (currPosOnChangeLaneTrack < changeLaneTrackSeg->getLength() / 3
			&& posOnLaneAfterChange.absLinearPos - this->changeLaneDist < 0
			&& nextItOldLane != trafficLane->getQueueEnd()) {
			LinearPosition nextVehOldLane = (*nextItOldLane)->getCurrPosOnLane(trafficLane);
			double toNextVehOldLane = nextVehOldLane.absLinearPos - currLinearPosOnLane.absLinearPos;
			if (clearDist > toNextVehOldLane)
				clearDist = toNextVehOldLane;
		}

		float currDynEnv = 0;
		switch (this->changeLaneBehabior)
		{
		case ChangeLaneBehabior::Persistent:
			currDynEnv = getBoldDynamicEnvelop();
			break;
		case ChangeLaneBehabior::Aggressive:
			currDynEnv = getAggressiveDynamicEnvelop();
			break;
		default:
			currDynEnv = getDynamicEnvelop();
			break;
		}


		//����������� ��������� � ������ �����������
		float currAcceleration = 0;
		if (clearDist >= currDynEnv) {
			//way is clear -> accelerate to speed limit
			if (velocity < speedLimit) {
				switch (this->changeLaneBehabior)
				{
				case ChangeLaneBehabior::Persistent:
					currAcceleration = fastAcceleration;
					break;
				case ChangeLaneBehabior::Aggressive:
					currAcceleration = urgentAcceleration;
					break;
				default:
					currAcceleration = standartAcceleration;
					break;
				}
			}
			else
			{
				currAcceleration = 0;//equally slow motion
				velocity = speedLimit;
			}

		}
		else
		{
			//slow down to a complete stop if can't change lane
			double distToStop = clearDist - reserveDistBetweenCars - length;
			if (distToStop < 0) {
				currAcceleration = 0;//we must stop already
				velocity = 0;
			}
			else
			{
				currAcceleration = -Unigine::Math::pow(velocity, 2) / (2 * distToStop);
			}
		}

		//����������� ����� ���������� ������������
		if (currAcceleration != 0) {
			velocity = velocity + currAcceleration * time;
		}

		if (velocity < UNIGINE_EPSILON)
			velocity = 0;

		if (velocity != 0) {
			float s = velocity * time;

			currPosOnChangeLaneTrack += s;
			distFromLastChangeLanesScanning += s;
			distFromLastNeighborLanesScanning += s;

			if (currPosOnChangeLaneTrack >= changeLaneTrackSeg->getLength()) {
				//������������ ���������
				//����������� ����
				{
					currLinearPosOnLane.copyFrom(posOnLaneAfterChange);
					if (currPosOnChangeLaneTrack > changeLaneTrackSeg->getLength()) {
						s = currPosOnChangeLaneTrack - changeLaneTrackSeg->getLength();
						currLinearPosOnLane.increaseLinearPos(s);
					}
					moveOnPos(currLinearPosOnLane.getPos3D());
				}


				//���� ����������� ��������� ����� ���������� ������������
				//�������� ����� �� StraightMove
				currentActivity = VehicleActivity::StraightMove;

				if (!vehicleIteratorReplaced) {
					//��������� ������ ���� � ������� ����� ������
					trafficLaneChangeTo->replaceVehicleItTo(
						tempChangeLaneIt, trafficLane, vehicleIterator);
					//������� ��������� ���� �� ������� ����� ������
					trafficLaneChangeTo->deleteTempChangeLaneIt(tempChangeLaneIt);
					//�������� ��������� trafficLane
					trafficLane = trafficLaneChangeTo;
				}


				//�������� ������ � ������� ������/�����
				laneToTheLeft = trafficLane->lanesToTheLeftBegin();
				laneToTheRight = trafficLane->lanesToTheRightBegin();
				posOnLaneToTheLeft = LinearPosition::Null();
				posOnLaneToTheRight = LinearPosition::Null();
				if (neighborVehiclesLeft)
					delete[] neighborVehiclesLeft;
				if (neighborVehiclesRight)
					delete[] neighborVehiclesRight;
				neighborVehiclesLeft = nullptr;
				neighborVehiclesRight = nullptr;

				//����� ������ � ������� ������/�����, ���������� �� �������� �������,
				//������� �� �������� ������� � ��������� ������ �����, ������� ��������� � ��������� StraightMove
				//���� ����� ��������

				if (vehicleIterator != trafficLane->getQueueStart()) {
					std::list<Vehicle*>::iterator it;
					for (it = std::prev(vehicleIterator); it != trafficLane->getQueueStart(); it--) {
						if ((*it)->currentActivity != VehicleActivity::ChangeLane) {

							if ((*it)->neighborVehiclesLeft == nullptr) {
								int a = 0;
							}

							(*it)->copyNeighborLanesDataTo(this);//���������� ������
							break;
						}
					}
				}


				//�������� ���������� ������������ � ������ ����������
				changeLaneTrack->clear();
				posOnLaneAfterChange = LinearPosition::Null();
				trafficLaneChangeTo = nullptr;
				tempLPosOnLaneChangeTo = LinearPosition::Null();
				currPosOnChangeLaneTrack = 0.0f;
				distFromLastChangeLanesScanning = 0.0f;
				this->changeLaneDist = 0;
				this->changeLaneBehabior = ChangeLaneBehabior::Standard;
				vehicleIteratorReplaced = false;
				//TODO: ����� ������������ �������� ���������� ����������� �� �������� ��� ���� ������

#ifdef DEBUG
				endChangeLaneCount++;
				thisFrameChangedLane = true;
#endif

			}
			else
			{
				//���������� �������� �� ���������� ������������
				//����������� ����
				float linearParam = currPosOnChangeLaneTrack / changeLaneTrackSeg->getLength();

				float nonLinearParam = changeLaneTrackSeg->linearToParametric(linearParam);
				Unigine::Math::dvec3 pt = changeLaneTrackSeg->calcPoint(nonLinearParam);
				Unigine::Math::vec3 tan = changeLaneTrackSeg->calcTangent(nonLinearParam);
				Unigine::Math::vec3 up = changeLaneTrackSeg->calcUpVector(nonLinearParam);

				Position3D vp = Position3D(changeLaneTrackSeg, pt, tan, up);
				moveOnPos(vp);



				//������������ ��������� �� ������� ����� �������� ��������������� 
				//(tempLPosOnLaneChangeTo � currLinearPosOnLane)
				if (distFromLastChangeLanesScanning > 1e-3f) {


#ifdef DEBUG
					int n = 0;
#endif
					if (!vehicleIteratorReplaced) {
						while (vp.isInFrontOf(&currLinearPosOnLane, false) == 0)
						{
#ifdef DEBUG
							n++;
#endif
							if (currLinearPosOnLane.moveToNextSegment()) {
								//������ ����������. ������ �� ������
								break;
							}

						}
					}



					/*
					//� �����-�� ������ �� ����� ��������� ������ �������
					//� ���� ������ ������ �������� �� ������ �������
					if (nextItOldLane != trafficLane->getQueueEnd()) {
						double nextCarPos = (*nextItOldLane)->getCurrPosOnLane(trafficLane).absLinearPos;
						if (currLinearPosOnLane.absLinearPos > nextCarPos)
						{
							//��������� ������ ���� � ������� ����� ������
							trafficLaneChangeTo->replaceVehicleItTo(
								tempChangeLaneIt, trafficLane, vehicleIterator);
							//������� ��������� ���� �� ������� ����� ������
							trafficLaneChangeTo->deleteTempChangeLaneIt(tempChangeLaneIt);
							//�������� ��������� trafficLane
							trafficLane = trafficLaneChangeTo;

							tempChangeLaneIt = vehicleIterator;

							vehicleIteratorReplaced = true;
						}
					}
					*/


#ifdef DEBUG
					n = 0;
#endif
					while (vp.isInFrontOf(&tempLPosOnLaneChangeTo, false) == 0)
					{
#ifdef DEBUG
						n++;
#endif
						if (posOnLaneAfterChange.absLinearPos - this->changeLaneDist < 0)
							break;//����� ������ ����� ��� �� �������� TODO: ��� ������ ����� ���������

						if (tempLPosOnLaneChangeTo.moveToNextSegment()) {
							//������ ����������. ������ �� ������
							break;
						}

					}




					distFromLastChangeLanesScanning = 0;
				}

#ifdef DEBUG
				//������������ ��������� currLinearPosOnLane � tempLPosOnLaneChangeTo
				{
					Position3D p = currLinearPosOnLane.getPos3D();
					Unigine::Math::dmat4 transf;
					transf.setTranslate(p.absPos);
					Unigine::Visualizer::get()->renderCapsule(0.5f, 3.0f, transf,
						Unigine::Math::vec4(1, 0, 1, 1));
				}

				{
					Position3D p = tempLPosOnLaneChangeTo.getPos3D();
					Unigine::Math::dmat4 transf;
					transf.setTranslate(p.absPos);
					Unigine::Visualizer::get()->renderCapsule(0.5f, 3.0f, transf,
						Unigine::Math::vec4(1, 0, 1, 1));
				}

#endif

			}

		}
	}
	break;
	case StraightMove:
	{
#pragma region ������������ �������� �����
		//if (testigVeh) 
		//{
		if (distFromLastNeighborLanesScanning > 1e-3f)
			//to avoid errors update neighbor positions only if moved significaly
		{
			//update current laneToTheLeft and laneToTheRight
			laneToTheLeft = updateNeighborLane(laneToTheLeft, trafficLane->lanesToTheLeftEnd());
			laneToTheRight = updateNeighborLane(laneToTheRight, trafficLane->lanesToTheRightEnd());

			if (laneToTheLeft != trafficLane->lanesToTheLeftEnd())
				updatePosOnNeighborLane(&posOnLaneToTheLeft, *laneToTheLeft);
			else
				posOnLaneToTheLeft = LinearPosition::Null();//������ ��� ����� � ���� ������� ���
			if (laneToTheRight != trafficLane->lanesToTheRightEnd())
				updatePosOnNeighborLane(&posOnLaneToTheRight, *laneToTheRight);
			else
				posOnLaneToTheRight = LinearPosition::Null();

			//#ifdef DEBUG 
			//			//������������ ��������� �� �������� �������
			//			if (!posOnLaneToTheLeft.isEmpty()) {
			//				testMarkerLeft->setEnabled(1);
			//				Position3D pos = posOnLaneToTheLeft.getPos3D();
			//				testMarkerLeft->setWorldPosition(pos.absPos);
			//				testMarkerLeft->setWorldDirection(pos.tangent, pos.up, Unigine::Math::AXIS_Y);
			//			}
			//			else
			//			{
			//				testMarkerLeft->setEnabled(0);
			//			}
			//
			//			if (!posOnLaneToTheRight.isEmpty()) {
			//				testMarkerRight->setEnabled(1);
			//				Position3D pos = posOnLaneToTheRight.getPos3D();
			//				testMarkerRight->setWorldPosition(pos.absPos);
			//				testMarkerRight->setWorldDirection(pos.tangent, pos.up, Unigine::Math::AXIS_Y);
			//			}
			//			else
			//			{
			//				testMarkerRight->setEnabled(0);
			//			}
			//#endif

			distFromLastNeighborLanesScanning = 0;//increase if moved
		}



#ifdef DEBUG
		int left = 0;
		int right = 0;
#endif // DEBUG



		//������������ ��������� ����� �� �������� �������
		//���� ��������� �� �������� ������ ��������, �� �������� �������� ����������� �����������,
		//������� ��������� ����� ������ � ��������� �� �������� ������
		//����� ������ ��� ����� ����� ������������, �� ��������� ������� ��� ������ ����� ���� ��������
		if (laneToTheLeft != trafficLane->lanesToTheLeftEnd()) {
			TrafficLane* neighborLane = (*laneToTheLeft)->data;
			if (neighborVehiclesLeft == nullptr)//TODO: ��� ������������ ������� ������ ��� ����� ������ � �������� �����
			{
				neighborVehiclesLeft = new std::list<Vehicle*>::iterator[2]
				{ neighborLane->getQueueEnd(), neighborLane->getQueueEnd() };
			}


			if (!posOnLaneToTheLeft.isEmpty())
			{
				if (neighborLane->endOfLaneLinear().absLinearPos - posOnLaneToTheLeft.absLinearPos > changeLaneMinDistance) {
					updateNeighborVehicles(neighborVehiclesLeft, neighborLane, posOnLaneToTheLeft);
#ifdef DEBUG
					left = 1;
#endif // DEBUG
				}
				else
				{
					//����� �� �������� ������ ��� ����� �� ������� ��� ������������ - ����� �� �����������
					neighborVehiclesLeft[0] = neighborLane->getQueueEnd();
					neighborVehiclesLeft[1] = neighborLane->getQueueEnd();
#ifdef DEBUG
					left = 2;
#endif // DEBUG
				}
			}
			else
			{
				//����������� � ������ ��������
				//������ ����� ������ ������ �� ���� ������
				neighborVehiclesLeft[0] = neighborLane->getQueueEnd();
				neighborVehiclesLeft[1] = neighborLane->getQueueStart();
#ifdef DEBUG
				left = 3;
#endif // DEBUG
			}

#ifdef DEBUG ������������
			if (testigVeh) {
				if (neighborVehiclesLeft[0] != neighborLane->getQueueEnd()) {
					(*neighborVehiclesLeft[0])->highlight();
					Unigine::Visualizer::get()->renderLine3D(node->getWorldPosition(),
						(*neighborVehiclesLeft[0])->getCurrPosOnLane(neighborLane).getPos3D().absPos,
						Unigine::Math::vec4(0, 0, 1, 1));
				}

				if (neighborVehiclesLeft[1] != neighborLane->getQueueEnd()) {
					(*neighborVehiclesLeft[1])->highlight();
					Unigine::Visualizer::get()->renderLine3D(node->getWorldPosition(),
						(*neighborVehiclesLeft[1])->getCurrPosOnLane(neighborLane).getPos3D().absPos,
						Unigine::Math::vec4(0, 0, 1, 1));
				}
			}
#endif	
		}
		if (laneToTheRight != trafficLane->lanesToTheRightEnd()) {
			TrafficLane* neighborLane = (*laneToTheRight)->data;

			if (neighborVehiclesRight == nullptr)//TODO: ��� ������������ ������� ������ ��� ����� ������ � �������� �����
			{
				neighborVehiclesRight = new std::list<Vehicle*>::iterator[2]
				{ neighborLane->getQueueEnd(), neighborLane->getQueueEnd() };
			}

			if (!posOnLaneToTheRight.isEmpty()) {
				if (neighborLane->endOfLaneLinear().absLinearPos - posOnLaneToTheRight.absLinearPos
		> changeLaneMinDistance)
				{
					updateNeighborVehicles(neighborVehiclesRight, neighborLane, posOnLaneToTheRight);
#ifdef DEBUG
					right = 1;
#endif // DEBUG
				}
				else
				{
					//����� �� �������� ������ ��� ����� �� ������� ��� ������������
					neighborVehiclesRight[0] = neighborLane->getQueueEnd();
					neighborVehiclesRight[1] = neighborLane->getQueueEnd();
#ifdef DEBUG
					right = 2;
#endif // DEBUG
				}
			}
			else
			{
				//������ ����� ������ ������ �� ������
				neighborVehiclesRight[0] = neighborLane->getQueueEnd();
				neighborVehiclesRight[1] = neighborLane->getQueueStart();
#ifdef DEBUG
				right = 3;
#endif // DEBUG
			}

#ifdef DEBUG ������������
			if (testigVeh) {
				if (neighborVehiclesRight[0] != neighborLane->getQueueEnd()) {
					(*neighborVehiclesRight[0])->highlight();
					Unigine::Visualizer::get()->renderLine3D(node->getWorldPosition(),
						(*neighborVehiclesRight[0])->getCurrPosOnLane(neighborLane).getPos3D().absPos,
						Unigine::Math::vec4(0, 0, 1, 1));
				}

				if (neighborVehiclesRight[1] != neighborLane->getQueueEnd()) {
					(*neighborVehiclesRight[1])->highlight();
					Unigine::Visualizer::get()->renderLine3D(node->getWorldPosition(),
						(*neighborVehiclesRight[1])->getCurrPosOnLane(neighborLane).getPos3D().absPos,
						Unigine::Math::vec4(0, 0, 1, 1));
				}
			}
#endif
		}


		//}
#pragma endregion


		//����������� ����������� �������
		std::list<Vehicle*>::iterator nextIt = std::next(vehicleIterator);
		while (nextIt != trafficLane->getQueueEnd()
			&& (*nextIt)->getCurrPosOnLane(trafficLane).absLinearPos < currLinearPosOnLane.absLinearPos)
		{
			nextIt = std::next(nextIt);
		}
		ObstacleType obstacleType;
		LinearPosition obstacleLP = LinearPosition::Null();
		double clearDist = getClearDist(nextIt, currLinearPosOnLane,
			obstacleType, obstacleLP, trafficLane, !movingThroughObstacle.isEmpty());


		//here the decision on current acceleration has to be made. 
		//if the path is free, then we accelerate with standard acceleration. 
		//if something interferes, then current acceleration is such that, with an equally slow motion, stop at a free site
		float currAcceleration;
		float currDynEnv = getDynamicEnvelop();
		if (clearDist >= currDynEnv) {
			//way is clear -> accelerate to speed limit
			if (velocity < speedLimit) {
				currAcceleration = standartAcceleration;
			}
			else
			{
				currAcceleration = 0;//equally slow motion
				velocity = speedLimit;
			}

		}
		else
		{
			bool closeToEndOfLane = !trafficLane->getLeadToEndOfRoad() && trafficLane->endOfLaneLinear()
				.absLinearPos - currLinearPosOnLane.absLinearPos <= needToChangeLaneIfItEndsAfter;
			bool trafficJam = velocity <= trafficJamVelocity;

			//start to change lane if possible
			LinearPosition canChangeLP = LinearPosition::Null();
			LinearPosition currPosOnLaneChangeTo = LinearPosition::Null();
			TrafficLane* changeTo = nullptr;
			std::list<Vehicle*>::iterator* itAheadOnLaneChangeTo = nullptr;
			double changeLaneDist = 0;
			ChangeLaneBehabior changeLaneBehabior = ChangeLaneBehabior::Standard;
			int decision = 0;

			//���� ������� ��� ��� �� ��������� ����� ����, �� ������� ������������
			if (!endChangeLanePrevFrame
				&& !(obstacleType == ObstacleType::PaymentCollectionPoint
					|| !movingThroughObstacle.isEmpty())) {
				//���������������� ������������� ������
				//TODO: ��������������� ���� ��� ������ ������������
				if (laneToTheRight != trafficLane->lanesToTheRightEnd()) {
					canChangeLP = changeLaneDecision(*laneToTheRight,
						posOnLaneToTheRight, obstacleType, neighborVehiclesRight,
						currDynEnv, closeToEndOfLane, trafficJam, changeLaneBehabior, changeLaneDist);
					if (!canChangeLP.isEmpty())
					{
						decision = 1;
						changeTo = (*laneToTheRight)->data;
						currPosOnLaneChangeTo = posOnLaneToTheRight;
						itAheadOnLaneChangeTo = &neighborVehiclesRight[1];
					}
				}
				//���� ������ ������, �� ��������� ����� �� �����
				if (decision == 0 && laneToTheLeft != trafficLane->lanesToTheLeftEnd()) {
					canChangeLP = changeLaneDecision(*laneToTheLeft,
						posOnLaneToTheLeft, obstacleType, neighborVehiclesLeft,
						currDynEnv, closeToEndOfLane, trafficJam, changeLaneBehabior, changeLaneDist);
					if (!canChangeLP.isEmpty())
					{
						decision = 1;
						changeTo = (*laneToTheLeft)->data;
						currPosOnLaneChangeTo = posOnLaneToTheLeft;
						itAheadOnLaneChangeTo = &neighborVehiclesLeft[1];
					}
				}
			}








			switch (decision)
			{
			case 1:
			{
				assert(!canChangeLP.isEmpty());
				//assert(!currPosOnLaneChangeTo.isEmpty());
				assert(changeTo);
				assert(itAheadOnLaneChangeTo);
				//��������� ������-���� ���������� ������������
				createChangeLaneTrack(currLinearPosOnLane.getPos3D(), canChangeLP.getPos3D());
				currentActivity = VehicleActivity::ChangeLane;
				posOnLaneAfterChange = canChangeLP;
				trafficLaneChangeTo = changeTo;
				tempLPosOnLaneChangeTo = !currPosOnLaneChangeTo.isEmpty() ?
					currPosOnLaneChangeTo : trafficLaneChangeTo->startOfLaneLinear();
				tempChangeLaneIt = trafficLaneChangeTo->createTempChangeLaneIterator
				(this, *itAheadOnLaneChangeTo);//TODO: ��� ������ - ������ tempChangeLaneIt ����� �����������

				currPosOnChangeLaneTrack = 0;
				distFromLastChangeLanesScanning = 0;
				this->changeLaneDist = changeLaneDist;
				this->changeLaneBehabior = changeLaneBehabior;
				vehicleIteratorReplaced = false;

				//� ���� �� ���� ���������� �������� �� ���������� ����� ������
				goto performAction;

			}
			break;

			default:
			{
				//slow down to a complete stop if can't change lane
				double distToStop = clearDist - reserveDistBetweenCars - length;
				if (distToStop < 0) {
					currAcceleration = 0;//we must stop already
					velocity = 0;
				}
				else
				{
					currAcceleration = -Unigine::Math::pow(velocity, 2) / (2 * distToStop);
				}
			}
			}




		}




		//movement
		//calculate new velocity with current acceleration
		//move car


		if (currAcceleration != 0) {
			velocity = velocity + currAcceleration * time;
		}

		if (velocity < UNIGINE_EPSILON) velocity = 0;

		if (velocity != 0) {
			float s = velocity * time;

			//Unigine::SplineSegmentPtr oldSeg = currLinearPosOnLane.splSegment;

			if (currLinearPosOnLane.increaseLinearPos(s)) {
				if (lane->getLeadToEndOfRoad()) {

					reachedEndOfRoad();
				}

			}
			else
			{
				Position3D vp = currLinearPosOnLane.getPos3D();
				moveOnPos(vp);
				distFromLastNeighborLanesScanning += s;

				if (!movingThroughObstacle.isEmpty()
					&& currLinearPosOnLane.absLinearPos >= movingThroughObstacle.getSegEndLinearPos()) {
					//we path through obstacle!
					movingThroughObstacle = LinearPosition::Null();
				}
			}
		}


		if (obstacleType == ObstacleType::PaymentCollectionPoint && velocity == 0) {
			//we stopped on Payment Collection Point
			//wait some time
			currentActivity = VehicleActivity::Wait;
			timeToWait = timeToWaitOnPaymentCollectionPoint;
			movingThroughObstacle = obstacleLP;
		}
	}
	break;
	case Wait:
		if (timeToWait >= 0.0f)
		{
			timeToWait -= Unigine::Game::get()->getIFps();
			if (timeToWait < 0.0f)
			{
				currentActivity = VehicleActivity::StraightMove;
				timeToWait = 0;
			}
		}
		break;
	default:
		break;
	}



#ifdef DEBUG 
	if (testigVeh) {
		//���������� ���� ������
		Unigine::Visualizer::get()->renderNodeBoundSphere(node->getNode(),
			Unigine::Math::vec4(1, 0, 0, 1));
	}

	//������������ ���������� ������������
	Unigine::Vector<Unigine::SplineSegmentPtr> segments;
	changeLaneTrack->getSplineSegments(segments);
	assert(segments.size() <= 1);
	if (segments.size() == 1) {
		Unigine::SplineSegmentPtr path = segments[0];
		Unigine::Math::dvec3 prevPt = carriageway->changeLaneTracksTransf *
			path->getStartPoint()->getPosition();
		double dist = 0.5;
		while (dist < path->getLength())
		{
			float linearParam = dist / path->getLength();
			float nonLinearParam = path->linearToParametric(linearParam);
			Unigine::Math::dvec3 pt = carriageway->changeLaneTracksTransf * path->calcPoint(nonLinearParam);

			Unigine::Visualizer::get()->renderLine3D(prevPt, pt,
				Unigine::Math::vec4(0, 1, 0, 1));

			dist += 0.5;
			prevPt = pt;
		}

		Unigine::Math::dvec3 endPt = carriageway->changeLaneTracksTransf *
			path->getEndPoint()->getPosition();
		Unigine::Visualizer::get()->renderLine3D(prevPt, endPt,
			Unigine::Math::vec4(0, 1, 0, 1));
	}

	if (thisFrameChangedLane) endChangeLanePrevFrame = true;
	else endChangeLanePrevFrame = false;



#endif

}

double Vehicle::getClearDist(std::list<Vehicle*>::iterator nextIt,
	LinearPosition linPos, ObstacleType &obstacleType,
	LinearPosition &obstacleLP, TrafficLane * lane, bool ignoreFirstObstacle)
{

	double clearDist = DBL_MAX;
	obstacleType = ObstacleType::None;
	obstacleLP = LinearPosition::Null();

	if (!lane->getLeadToEndOfRoad()) {
		clearDist = lane->getOveralLength() - linPos.absLinearPos;
		obstacleType = ObstacleType::EndOfLane;
		//TODO: obstacleLP?
	}

	//get next obstacle
	LinearPosition obstacleLp = lane->getNextObstacle(linPos, ignoreFirstObstacle);

	if (nextIt != lane->getQueueEnd() && !obstacleLp.isEmpty()) {
		//what is closer?
		double distToNextCar = (*nextIt)->getCurrPosOnLane(lane).absLinearPos - linPos.absLinearPos;
		double distToObstacle = obstacleLp.absLinearPos - linPos.absLinearPos;
		if (distToNextCar < distToObstacle) {
			clearDist = distToNextCar;
			obstacleType = ObstacleType::MovingVehicle;
			obstacleLP = (*nextIt)->getCurrPosOnLane(lane);
		}
		else
		{
			clearDist = distToObstacle;
			obstacleType = ObstacleType::PaymentCollectionPoint;
			obstacleLP = obstacleLp;
		}
	}
	else if (nextIt != lane->getQueueEnd())
	{
		//there is some vehicle ahead
		clearDist = (*nextIt)->getCurrPosOnLane(lane).absLinearPos - linPos.absLinearPos;
		obstacleType = ObstacleType::MovingVehicle;
		obstacleLP = (*nextIt)->getCurrPosOnLane(lane);
	}
	else if (!obstacleLp.isEmpty())
	{
		//there is some obstacle ahead
		clearDist = obstacleLp.absLinearPos - linPos.absLinearPos;
		obstacleType = ObstacleType::PaymentCollectionPoint;
		obstacleLP = obstacleLp;
	}

	return clearDist;
}



float Vehicle::getVelocityToFitIntoSpan(double span) {
	double stoppingDist = span - length - reserveDistBetweenCars;

	if (stoppingDist < UNIGINE_EPSILON) {
		return 0;
	}

	double calcVelocity = Unigine::Math::dsqrt(2 * stoppingDist * standartDamping);
	return calcVelocity > UNIGINE_EPSILON ? calcVelocity : 0;
}
