#pragma once
#include <UnigineMathLib.h>
#include <UnigineWorlds.h>
#include "Position3D.h"


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

	LinearPosition() {}

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
			Unigine::SplineSegmentPtr nextSeg = getNextSegment();

			if (nextSeg) {
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


	Unigine::SplineSegmentPtr getNextSegment() {
		Unigine::SplineSegmentPtr nextSeg;

		Unigine::SplinePointPtr pt = splSegment->getEndPoint();
		assert(pt->getNumSegments() <= 2);

		if(pt->getNumSegments()<2) return nextSeg;

		Unigine::Vector<Unigine::SplineSegmentPtr> segments;
		pt->getSplineSegments(segments);

		for (int s = 0; s < segments.size(); s++) {
			Unigine::SplineSegmentPtr seg = segments[s];
			if (seg != splSegment) {
				nextSeg = seg;
				break;
			}

		}

		assert(nextSeg);
		return nextSeg;
	}





	Position3D getPos3D() {
		float distOnSplineSeg = this->distOnSplineSeg();

		float segLen = splSegment->getLength();

		assert(distOnSplineSeg <= segLen);

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

