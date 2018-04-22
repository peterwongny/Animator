#pragma once

#include "vec.h"
#include "mat.h"
#include "modelerview.h"

#include "modelerglobals.h"

class InverseKinematics
{
	float armlength1;
	float armlength2;
	Vec3f origin;

	//q[0] == arm1_y
	//q[1] == arm1_x
	//q[2] == arm2_y
	//q[3] == arm3_x 
 	Vec4f q;
	Vec4f up_constrain;
	Vec4f down_constrain;

	//Vec3f joint;
	Vec3f endeffector;


public:
	InverseKinematics();
	InverseKinematics(Vec3f origin, float armlength1, float armlength2);

	void SetOrigin(float x, float y, float z);
	void SetArmlength1(float l);
	void SetArmlength2(float l);
	void SetQ(float arm1y, float arm1x, float arm2y, float arm2x);

	Vec3f joint;
	
	
	~InverseKinematics();

	Vec3f ForwardKinematics();

	Vec3f ForwardKinematics(Vec4f q);

	Mat4d Jacobian();

	Vec4f getResult(Vec3f target);

	Vec4f getResultccd(Vec3f target);

};

