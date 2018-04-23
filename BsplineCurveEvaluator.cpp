#include "BsplineCurveEvaluator.h"
#include "BezierCurveEvaluator.h"
#include "mat.h"
#include "vec.h"

void BsplineCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts, std::vector<Point>& ptvEvaluatedCurvePts, const float & fAniLength, const bool & bWrap) const
{
	ptvEvaluatedCurvePts.clear();

	vector<Point> my_ptvCtrlPts;
	if (bWrap) {
		// add the last 2 points before starting point
		my_ptvCtrlPts.push_back(Point((ptvCtrlPts.end() - 2)->x - fAniLength, (ptvCtrlPts.end() - 2)->y));
		my_ptvCtrlPts.push_back(Point((ptvCtrlPts.end() - 1)->x - fAniLength, (ptvCtrlPts.end() - 1)->y));
		// the middle points
		my_ptvCtrlPts.insert(my_ptvCtrlPts.end(), ptvCtrlPts.begin(), ptvCtrlPts.end());
		// add the first 2 points after ending point
		my_ptvCtrlPts.push_back(Point((ptvCtrlPts.begin())->x + fAniLength, (ptvCtrlPts.begin() )->y));
		my_ptvCtrlPts.push_back(Point((ptvCtrlPts.begin()+1)->x + fAniLength, (ptvCtrlPts.begin()+1)->y));
	}
	else {
		// front pts, end pts *3
		my_ptvCtrlPts.push_back(ptvCtrlPts.front());
		my_ptvCtrlPts.push_back(ptvCtrlPts.front());
		my_ptvCtrlPts.insert(my_ptvCtrlPts.end(), ptvCtrlPts.begin(), ptvCtrlPts.end());
		my_ptvCtrlPts.push_back(ptvCtrlPts.back());
		my_ptvCtrlPts.push_back(ptvCtrlPts.back());
		// the first and last segments of the curve horizontal.
		ptvEvaluatedCurvePts.push_back(Point(0.0, ptvCtrlPts.front().y));
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts.back().y));
	}

	int iCtrlPtCount = my_ptvCtrlPts.size();

	BezierCurveEvaluator bezierCurveEvaluator;

	Mat4d M=Mat4d(
		1, 4, 1, 0,
		0, 4, 2, 0,
		0, 2, 4, 0,
		0, 1, 4, 1)/6.0;

	int pointIndex = 0;
	for (pointIndex = 0; pointIndex + 3 < iCtrlPtCount; pointIndex ++ ) {//c2 continuity
		//convert B to V
		const Vec4d px(my_ptvCtrlPts[pointIndex].x, my_ptvCtrlPts[pointIndex + 1].x, my_ptvCtrlPts[pointIndex + 2].x, my_ptvCtrlPts[pointIndex + 3].x);
		const Vec4d py(my_ptvCtrlPts[pointIndex].y, my_ptvCtrlPts[pointIndex + 1].y, my_ptvCtrlPts[pointIndex + 2].y, my_ptvCtrlPts[pointIndex + 3].y);
		Vec4d Vx = M*px;
		Vec4d Vy = M*py;

		vector<Point> bezierControl;
		for (int i = 0; i < 4; ++i) {
			bezierControl.push_back(Point(Vx[i], Vy[i]));
		}
		vector<Point> bezierEvaluatedCurvePts;
		bezierCurveEvaluator.evaluateCurve(bezierControl, bezierEvaluatedCurvePts, fAniLength, false);
		ptvEvaluatedCurvePts.insert(ptvEvaluatedCurvePts.end(), bezierEvaluatedCurvePts.begin(), bezierEvaluatedCurvePts.end() - 2); //don't include the horalizal line at the starting/end
		// for some points out of the [0,fAniLength] length....just ignore it *facepalm*
	}
}
