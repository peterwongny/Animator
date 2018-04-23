#include "CatmullRomCurveEvaluator.h"
#include "mat.h"
#include "vec.h"

void CatmullRomCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts, std::vector<Point>& ptvEvaluatedCurvePts, const float & fAniLength, const bool & bWrap) const
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
		my_ptvCtrlPts.push_back(Point((ptvCtrlPts.begin())->x + fAniLength, (ptvCtrlPts.begin())->y));
		my_ptvCtrlPts.push_back(Point((ptvCtrlPts.begin() + 1)->x + fAniLength, (ptvCtrlPts.begin() + 1)->y));
	}
	else {
		/*my_ptvCtrlPts.push_back(Point(0, ptvCtrlPts.front().y));
		my_ptvCtrlPts.insert(my_ptvCtrlPts.end(), ptvCtrlPts.begin(), ptvCtrlPts.end());
		my_ptvCtrlPts.push_back(Point(fAniLength, ptvCtrlPts.back().y));*/

		my_ptvCtrlPts.push_back(ptvCtrlPts.front());
		my_ptvCtrlPts.insert(my_ptvCtrlPts.end(), ptvCtrlPts.begin(), ptvCtrlPts.end());
		my_ptvCtrlPts.push_back(ptvCtrlPts.back());

		// the first and last segments of the curve horizontal
		ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts.front().y));
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts.back().y));
	}
	
	int iCtrlPtCount = my_ptvCtrlPts.size();

	Mat4d M = Mat4d(
		-1, 3, -3, 1,
		2, -5, 4, -1,
		-1, 0, 1, 0,
		0, 2, 0, 0) / 2.0;

	//below script is nearly copy from berizer, only change the +=3 to ++ since catmull have continuity

	int pointIndex = 0;
	for (pointIndex = 0; pointIndex + 3 < iCtrlPtCount; pointIndex ++) {
		ptvEvaluatedCurvePts.push_back(my_ptvCtrlPts[pointIndex]);
		ptvEvaluatedCurvePts.push_back(my_ptvCtrlPts[pointIndex + 3]);
		const Vec4d px(my_ptvCtrlPts[pointIndex].x, my_ptvCtrlPts[pointIndex + 1].x, my_ptvCtrlPts[pointIndex + 2].x, my_ptvCtrlPts[pointIndex + 3].x);
		const Vec4d py(my_ptvCtrlPts[pointIndex].y, my_ptvCtrlPts[pointIndex + 1].y, my_ptvCtrlPts[pointIndex + 2].y, my_ptvCtrlPts[pointIndex + 3].y);
		double t;
		int dividCount = (my_ptvCtrlPts[pointIndex + 3].x - my_ptvCtrlPts[pointIndex].x) * 3;
		for (int i = 0; i < dividCount; i++) {
			t = double(i) / double(dividCount); //t in range [0,1]
			const Vec4d vecT(t*t*t, t*t, t, 1);
			const Vec4d B = vecT*M;
			Point Q = Point(B*px, B*py); //1x4 x 4x4 x 4x1
			if (Q.x>fAniLength&&bWrap) {//the wrapped point
				Q.x = fmod(Q.x, fAniLength);
			}
			ptvEvaluatedCurvePts.push_back(Q);
		}
	}
}
