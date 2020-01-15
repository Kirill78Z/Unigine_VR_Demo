#include "MovementPath.h"
#include <UnigineEditor.h>



MovementPath::MovementPath(Unigine::WorldSplineGraphPtr node)
{
	worldSplineGraph = node;

	//��������� �������: � ������ ������� ����������� ��� �������� (� ������ � � ����� ����� ���� ���� �������)
	Unigine::Vector<Unigine::SplinePointPtr> points;
	worldSplineGraph->getSplinePoints(points);

	Unigine::Vector<Unigine::SplineSegmentPtr> segments;
	worldSplineGraph->getSplineSegments(segments);
	startSegment = segments[0];
	
	//TODO: ��� ��������� ��������� �������� � ������??
	for (int p = 0; p < points.size(); p++) {
		Unigine::SplinePointPtr vert = points[p];
		int numOfSeg = vert->getNumSegments();
		assert(numOfSeg == 2 || 
			((p == 0 || p == points.size()-1) 
				&& numOfSeg == 1));
	}



	//TODO: ������� ������������ ���������� �������� � �������� �������

	Unigine::NodePtr test_obj = Unigine::Editor::get()->getNodeByName("supply_ship");
	MovingObject* testObj = new MovingObject(test_obj);
	testObj->currLinearPos = getStartOfPath();
	testObj->currLinearPos.increaseLinearPos(100);
	testObj->updatePos();
	testObj->velocity = 11.1111;
	movingObjects.append(testObj);
}


MovementPath::~MovementPath()
{
}

void MovementPath::update()
{
	for (MovingObject* obj : movingObjects) {
		obj->update();
	}
}
