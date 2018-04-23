#include "BezierCurveEvaluator.h"
#include "mat.h"
#include "vec.h"


void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts, std::vector<Point>& ptvEvaluatedCurvePts, const float & fAniLength, const bool & bWrap) const
{
	vector<Point> my_ptvCtrlPts(ptvCtrlPts);
	if (bWrap) {
		my_ptvCtrlPts.push_back(Point(fAniLength+ ptvCtrlPts[0].x, ptvCtrlPts[0].y));
	}

	int iCtrlPtCount = my_ptvCtrlPts.size();
	ptvEvaluatedCurvePts.clear();
	Mat4d M(
			-1, 3, -3, 1,
			3, -6, 3, 0,
			-3, 3, 0, 0,
			1, 0, 0, 0);

	int pointIndex = 0;
	for (pointIndex = 0; pointIndex+3 < iCtrlPtCount; pointIndex += 3) {
		ptvEvaluatedCurvePts.push_back(my_ptvCtrlPts[pointIndex]);
		const Vec4d px(my_ptvCtrlPts[pointIndex].x, my_ptvCtrlPts[pointIndex + 1].x, my_ptvCtrlPts[pointIndex + 2].x, my_ptvCtrlPts[pointIndex + 3].x);
		const Vec4d py(my_ptvCtrlPts[pointIndex].y, my_ptvCtrlPts[pointIndex + 1].y, my_ptvCtrlPts[pointIndex + 2].y, my_ptvCtrlPts[pointIndex + 3].y);
		//haven't 4*2 matrix?
		double t;
		int dividCount = (my_ptvCtrlPts[pointIndex + 3].x - my_ptvCtrlPts[pointIndex].x )*3;
		for (int i = 0; i < dividCount; i++) {
			t = double(i) / double(dividCount); //t in range [0,1]
			const Vec4d vecT(t*t*t, t*t , t, 1);
			const Vec4d B = vecT*M;
			Point Q = Point(B*px, B*py); //1x4 x 4x4 x 4x1
			if(Q.x>fAniLength&&bWrap) {//the wrapped point
				Q.x = Q.x - fAniLength;
			}
			ptvEvaluatedCurvePts.push_back(Q);
		}
		ptvEvaluatedCurvePts.push_back(my_ptvCtrlPts[pointIndex + 3]);
	}

	if(bWrap){
		if((iCtrlPtCount-1)%3==0){
			//wrapped
		}else {
			for (; pointIndex < iCtrlPtCount-1; pointIndex++) {//adding the rest of points
				ptvEvaluatedCurvePts.push_back(my_ptvCtrlPts[pointIndex]);
			}
			float y;
			// linear wrapping
			if ((ptvCtrlPts[0].x + fAniLength) - ptvCtrlPts[iCtrlPtCount - 1].x > 0.0f) {
				y = (ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x) +
					ptvCtrlPts[iCtrlPtCount - 1].y * ptvCtrlPts[0].x) /
					(ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x);
			}
			else
				y = ptvCtrlPts[0].y;
			ptvEvaluatedCurvePts.push_back(Point(0, y));
			ptvEvaluatedCurvePts.push_back(Point(fAniLength, y));
		}
	}
	else {
		//the first and last segments of the curve horizontal.
		ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
		for (; pointIndex < iCtrlPtCount; pointIndex++) {//adding the rest of points
			ptvEvaluatedCurvePts.push_back(ptvCtrlPts[pointIndex]);
		}
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[pointIndex - 1].y));
	}

}
