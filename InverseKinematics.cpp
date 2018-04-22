#include "InverseKinematics.h"




InverseKinematics::InverseKinematics()
{
}

InverseKinematics::InverseKinematics(Vec3f origin, float armlength1, float armlength2)
	:armlength1(armlength1), armlength2(armlength2), origin(origin)
{

}

void InverseKinematics::SetOrigin(float x, float y, float z)
{
	origin[0] = x;
	origin[1] = y;
	origin[2] = z;
}

void InverseKinematics::SetArmlength1(float l)
{
	armlength1 = l;
}

void InverseKinematics::SetArmlength2(float l)
{
	armlength2 = l;
}

void InverseKinematics::SetQ(float arm1y, float arm1x, float arm2y, float arm2x)
{
	q[0] = arm1y / 180* M_PI;
	q[1] = arm1x / 180 * M_PI;
	q[2] = arm2y / 180 * M_PI;
	q[3] = arm2x / 180 * M_PI;

}

InverseKinematics::~InverseKinematics()
{
}

Mat4d rotateXm(float angle);
Mat4d rotateYm(float angle);
Vec4f matTvec4(Mat4d mat, Vec4f vec);

Vec3f InverseKinematics::ForwardKinematics()
{

	//for (int i = 0; i != 4; i++) {
	//	if (q[i] == 0) { q[i] = 0.0001; }
	//}

	Mat4d arm1rotateYm = rotateYm(q[0]);
	Mat4d arm1rotateXm = rotateXm(q[1]);

	Mat4d arm2rotateYm = rotateYm(q[2]);
	Mat4d arm2rotateXm = rotateXm(q[3]);

	//compute joint
	Vec4f arm1;
	arm1[1] = -armlength1;
	Vec4f joint4d = matTvec4(arm1rotateXm, matTvec4(arm1rotateYm, arm1));
	//Vec4f joint4d = matTvec4(arm1rotateYm, matTvec4(arm1rotateXm, arm1));
	joint[0] = joint4d[0]; joint[1] = joint4d[1]; joint[2] = joint4d[2];

	joint = joint + origin;


	//compute endeffector
	Vec4f arm2;
	arm2[1] = -armlength2;

	printf("arm2: %f, %f, %f, %f\n", arm2[0], arm2[1], arm2[2], arm2[3]);
	
	Vec4f end4d = matTvec4(arm2rotateXm, matTvec4(arm2rotateYm, arm2));
	//Vec4f end4d = matTvec4(arm2rotateYm, arm2);

	printf("end4d: %f, %f, %f, %f\n", end4d[0], end4d[1], end4d[2], end4d[3]);

	end4d = matTvec4(arm1rotateXm, matTvec4(arm1rotateYm, end4d));
	//end4d = matTvec4(arm1rotateYm, end4d);

	endeffector[0] = end4d[0]; endeffector[1] = end4d[1]; endeffector[2] = end4d[2];

	endeffector = endeffector + joint;

	printf("endeffector: %f, %f, %f\n", endeffector[0], endeffector[1], endeffector[2]);

	return endeffector;
}

//overloading, all variables are local
Vec3f InverseKinematics::ForwardKinematics(Vec4f qq)
{

	Mat4d arm1rotateYm = rotateYm(qq[0]);
	Mat4d arm1rotateXm = rotateXm(qq[1]);

	Mat4d arm2rotateYm = rotateYm(qq[2]);
	Mat4d arm2rotateXm = rotateXm(qq[3]);

	//compute joint
	Vec4f arm1;
	arm1[1] = -armlength1;
	Vec4f joint4d = matTvec4(arm1rotateXm, matTvec4(arm1rotateYm, arm1));
	//Vec4f joint4d = matTvec4(arm1rotateYm, matTvec4(arm1rotateXm, arm1));

	Vec3f joint;
	joint[0] = joint4d[0]; joint[1] = joint4d[1]; joint[2] = joint4d[2];

	joint = joint + origin;


	//compute endeffector
	Vec4f arm2;
	arm2[1] = -armlength2;

	Vec4f end4d = matTvec4(arm2rotateXm, matTvec4(arm2rotateYm, arm2));
	//Vec4f end4d = matTvec4(arm2rotateYm, matTvec4(arm2rotateXm, arm2));

	end4d = matTvec4(arm1rotateXm, matTvec4(arm1rotateYm, end4d));

	Vec3f endeffector;

	endeffector[0] = end4d[0]; endeffector[1] = end4d[1]; endeffector[2] = end4d[2];

	endeffector = endeffector + joint;


	return endeffector;
}


Mat4d InverseKinematics::Jacobian()
{
	Mat4d result;
	for (int i = 0; i != 4; i++) {
		Vec4f qq = q;
		qq[i] += 0.001;
		Vec3f forw = ForwardKinematics(qq);

		qq[i] -= 0.002;
		Vec3f back = ForwardKinematics(qq);

		result[0][i] = (forw[0] - back[0]) / 0.002;
		result[1][i] = (forw[1] - back[1]) / 0.002;
		result[2][i] = (forw[2] - back[2]) / 0.002;
		result[3][i] = 0;
	}

	return result;
}

Vec4f InverseKinematics::getResult(Vec3f target)
{

	if (target.length() >= armlength1 + armlength2) {
		Vec3f newT = (target - origin);
		newT.normalize();
		target = newT * (armlength1 + armlength2) + origin;
	}


	for (int i = 0; i != 2000; i++) {

		Vec3f e = target - ForwardKinematics();


		Mat4d jacobian = Jacobian();
		Mat4d jacobianT = jacobian.transpose();

		Vec4f delta;
		
		delta[0] = jacobian[0][0] * e[0] + jacobian[0][1] * e[1] + jacobian[0][2] * e[2];
		delta[1] = jacobian[1][0] * e[0] + jacobian[1][1] * e[1] + jacobian[1][2] * e[2];
		delta[2] = jacobian[2][0] * e[0] + jacobian[2][1] * e[1] + jacobian[2][2] * e[2];
		delta[3] = jacobian[3][0] * e[0] + jacobian[3][1] * e[1] + jacobian[3][2] * e[2];

		q = q + delta*0.01;

		if (delta.length2() < 0.01 || e.length2() < 0.01) return q / M_PI * 180;
	}






	return q / M_PI * 180;
}


float findAngleBetween(float x1, float y1, float x2, float y2);
int findRotateDirection(float x1, float y1, float x2, float y2);

Vec4f InverseKinematics::getResultccd(Vec3f target)
{
	
	
	for (int i = 0; i != 20; i++) {

		ForwardKinematics();


		Vec3f direction = target - joint;
		//direction.normalize();

		Vec3f arm2 = endeffector - joint;
		//Vec3f arm2 =  joint - endeffector;
		//arm2.normalize();

		//find angle in xy plane for arm2 (q[2])
		float angle_xy_arm2 = findAngleBetween(arm2[0], arm2[1], direction[0], direction[1]);
		int rotate_dir_xy_arm2 = findRotateDirection(arm2[0], arm2[1], direction[0], direction[1]);

		q[2] = q[2] + angle_xy_arm2*rotate_dir_xy_arm2;

		printf("direction before: %f, %f, %f\n", direction[0], direction[1], direction[2]);
		printf("arm2 before: %f, %f, %f\n", arm2[0], arm2[1], arm2[2]);

		ForwardKinematics();

		direction = target - joint;
		//direction.normalize();

		arm2 = endeffector - joint;

		//arm2.normalize();

		printf("direction after: %f, %f, %f\n", direction[0], direction[1], direction[2]);
		printf("arm2 after: %f, %f, %f\n", arm2[0], arm2[1], arm2[2]);


		//q[2] = 0;

		//find angle in yz plane for arm2 (q[3])
		float angle_yz_arm2 = findAngleBetween(arm2[1], arm2[2], direction[1], direction[2]);
		int rotate_dir_yz_arm2 = findRotateDirection(arm2[1], arm2[2], direction[1], direction[2]);

		q[3] = q[3] + angle_yz_arm2*rotate_dir_yz_arm2;

		//printf("%f", q[1]);

		////arm1
		//Vec3f directionA1 = target - origin;
		//directionA1.normalize();

		////Vec3f arm2 = endeffector - joint;
		//Vec3f arm1 = origin - endeffector;
		//arm2.normalize();

		////find angle in xy plane for arm2 (q[2])
		//float angle_xy_arm1 = findAngleBetween(arm1[0], arm1[1], directionA1[0], directionA1[1]);
		//int rotate_dir_xy_arm1 = findRotateDirection(arm1[0], arm1[1], directionA1[0], directionA1[1]);

		//q[0] = q[0] + angle_xy_arm1*rotate_dir_xy_arm1;
		////q[2] = 0;

		////find angle in yz plane for arm2 (q[3])
		//float angle_yz_arm1 = findAngleBetween(arm1[1], arm1[2], directionA1[1], directionA1[2]);
		//int rotate_dir_yz_arm1 = findRotateDirection(arm1[1], arm1[2], directionA1[1], directionA1[2]);

		//q[1] = q[1] + angle_yz_arm1*rotate_dir_yz_arm1;


	}


	return q/M_PI*180;
}


//need a new way to calculate angle
float findAngleBetween(float x1, float y1, float x2, float y2) {
	float dot_product = x1 * x2 + y1 * y2;
	float length1 = sqrt(x1*x1 + y1*y1);
	float length2 = sqrt(x2*x2 + y2*y2);

	float diff = dot_product / length1 / length2;

	//handle opposite direction
	//if (diff <= 0.00001 && (x1*x2 < 0 || y1*y2 < 0)) return M_PI;

	//if (diff <= -1.0 || diff >= 1.0) return 0.0;

	if (diff <= -1.0) diff = -1.0;
	if (diff >= 1.0) diff = 1.0;

	float ans = acos(diff);
	
	return ans;
}

int findRotateDirection(float x1, float y1, float x2, float y2) {
	float cross_product = x1 * y2 - x2 * y1;
	if (cross_product >= 0)
		return 1;
	return -1;
}

Vec4f matTvec4(Mat4d mat, Vec4f vec) {
	Vec4f result;
	result[0] = mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2] + mat[0][3] * vec[3];
	result[1] = mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2] + mat[1][3] * vec[3];
	result[2] = mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2] + mat[2][3] * vec[3];
	result[3] = mat[3][0] * vec[0] + mat[3][1] * vec[1] + mat[3][2] * vec[2] + mat[3][3] * vec[3];

	return result;
}


//Rz in lecture note
//normal is Z
//normal in sample.cpp, y and z is interchanged
Mat4d rotateYm(float angle) {
	Mat4d result;
	result[2][2] = 1;
	result[1][1] = cos(angle);
	result[0][0] = cos(angle);
	result[0][1] = -sin(angle);
	result[1][0] = sin(angle);

	result[3][3] = 1;

	return result;
}

//Rx in lecture note
//normal is x
Mat4d rotateXm(float angle) {
	Mat4d result;
	result[0][0] = 1;
	result[1][1] = cos(angle);
	result[2][2] = cos(angle);
	result[1][2] = -sin(angle);
	result[2][1] = sin(angle);

	result[3][3] = 1;

	return result;
}

