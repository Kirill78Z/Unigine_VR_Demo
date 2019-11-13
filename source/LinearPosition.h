#pragma once
#include <UnigineMathLib.h>
#include <UnigineWorlds.h>
#include "Position3D.h"


struct LinearPosition
{
	double segStartLinearPos = -1;
	Unigine::SplineSegmentPtr splSegment;
	double absLinearPos = -1;


#ifdef DEBUG
	int lastIncrease = -1;
#endif


	void copyFrom(LinearPosition other) {
		segStartLinearPos = other.segStartLinearPos;
		splSegment = other.splSegment;
		absLinearPos = other.absLinearPos;
	}



	double getSegEndLinearPos() {
		return segStartLinearPos + splSegment->getLength();
	}

	static LinearPosition Null() {
		LinearPosition lp = LinearPosition(Unigine::SplineSegmentPtr::Ptr(), -1, -1);
		return lp;
	}

	LinearPosition() {}

	LinearPosition(Unigine::SplineSegmentPtr splSegment,//TODO: максимальная длина
		double segStartLinearPos, double absLinearPos) {
		this->segStartLinearPos = segStartLinearPos;
		this->splSegment = splSegment;
		this->absLinearPos = absLinearPos;
	}

	~LinearPosition()
	{
		
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

		/*float len = splSegment->getLength();
		assert(distOnSplineSeg < len);*/

		return distOnSplineSeg;
	}


	//returns true if end of spline graph reached
	bool increaseLinearPos(double s) {
		assert(s > 0);
		absLinearPos += s;
#ifdef DEBUG
		lastIncrease = 1;
#endif

		//splSegment
		float distOnSplineSeg = this->distOnSplineSeg();

#ifdef DEBUG
		int n = 0;
#endif
		while (distOnSplineSeg >= splSegment->getLength())
		{
#ifdef DEBUG
			n++;
#endif
			//move to next segment
			Unigine::SplineSegmentPtr nextSeg = getNextSegment();

			if (nextSeg) {
				segStartLinearPos += splSegment->getLength();
				splSegment = nextSeg;
				distOnSplineSeg = this->distOnSplineSeg();
			}
			else
			{
				//Уточнить absLinearPos, чтобы оно было не более максимальной длины!
				absLinearPos = segStartLinearPos + splSegment->getLength();

				assert(this->distOnSplineSeg() == splSegment->getLength());
				return true;
			}

		}

		assert(this->distOnSplineSeg() < splSegment->getLength());
		return false;

	}

	//returns true if end of spline graph reached
	bool moveToNextSegment() {

		/*float distToSegEnd = splSegment->getLength() - distOnSplineSeg();
		assert(distToSegEnd >= 0);*/
		//absLinearPos += distToSegEnd;


		absLinearPos = segStartLinearPos + splSegment->getLength();
#ifdef DEBUG
		lastIncrease = 2;
#endif

		Unigine::SplineSegmentPtr nextSeg = getNextSegment();
		if (nextSeg) {
			segStartLinearPos += splSegment->getLength();
			splSegment = nextSeg;

			assert(this->distOnSplineSeg() < splSegment->getLength());
			return false;
		}
		else
		{
			assert(this->distOnSplineSeg() == splSegment->getLength());
			return true;
		}
	}



	Unigine::SplineSegmentPtr getNextSegment() {
		Unigine::SplineSegmentPtr nextSeg;

		Unigine::SplinePointPtr pt = splSegment->getEndPoint();
		assert(pt->getNumSegments() <= 2);

		if (pt->getNumSegments() < 2) return nextSeg;

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

		Position3D vp = Position3D(splSegment, pt, tan, up);
		return vp;
	}
};

