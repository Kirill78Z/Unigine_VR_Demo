#pragma once
#include "TrafficSimulation.h"
#include "Ñarriageway.h"
#include <list>
#include <UnigineWorlds.h>

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

	double getSegEndLinearPos() {
		return segStartLinearPos + splSegment->getLength();
	}

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




class TrafficLane
{
public:
	TrafficLane();
	~TrafficLane();
};

