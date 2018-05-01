// The sample model.  You should build a file
// very similar to this for when you make your model.
#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "particleSystem.h"
#include <FL/gl.h>
#include <GL/glu.h>

#include "modelerglobals.h"
#include "bitmap.h"
#include "InverseKinematics.h"

static unsigned char* textureImage;
static unsigned char* textureImageLeg;
static int frameTimer;

void drawLA(const float baseRadius, int depth);
void drawLB(const float baseRadius, int depth);
void drawLC(const float baseRadius, int depth);
void drawTorus(double innerRadius, double outterRadius);

void drawLA(const float baseRadius, int depth) {
	float height = VAL(LSHEIGHT);
	float scale = VAL(LSSCALE);
	if (depth <= 0)
		return;
	glPushMatrix();
		drawCylinder(height, baseRadius, baseRadius*scale);
		glTranslated(0, 0, height+VAL(LSSPACE));
		float shift = baseRadius*scale*0.5;
		glPushMatrix();
			glTranslated(shift, shift, 0);
			drawLB(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
		glPushMatrix();
		glTranslated(shift, -shift, 0);
		drawLA(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
		glPushMatrix();
		glTranslated(-shift, shift, 0);
		drawLC(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
		glPushMatrix();
		glTranslated(-shift, -shift, 0);
		drawLB(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
	glPopMatrix();
	return;
}

void drawLB(const float baseRadius, int depth) {
	float height = VAL(LSHEIGHT);
	float scale = VAL(LSSCALE);
	if (depth <= 0)
		return;
	glPushMatrix();
		glPushMatrix();
			float innerRadius = (baseRadius + baseRadius*scale) / 6;
			glTranslated(0, 0, height / 3);
			glPushMatrix();
			glRotated(90, 1, 0, 0);
			drawTorus(innerRadius, (baseRadius * 2 + baseRadius*scale) / 3);
			glPopMatrix();
			glTranslated(0, 0, height / 3);
			glPushMatrix();
			glRotated(90, 1, 0, 0);
			drawTorus(innerRadius, (baseRadius + baseRadius*scale * 2) / 3);
			glPopMatrix();
		glPopMatrix();
		glTranslated(0, 0, height + VAL(LSSPACE));
		float shift = baseRadius*scale*0.5;
		glPushMatrix();
		glTranslated(shift, shift, 0);
		drawLB(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
		glPushMatrix();
		glTranslated(shift, -shift, 0);
		drawLC(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
		glPushMatrix();
		glTranslated(-shift, shift, 0);
		drawLC(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
		glPushMatrix();
		glTranslated(-shift, -shift, 0);
		drawLA(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
	glPopMatrix();
	return;
}


void drawLC(const float baseRadius, int depth) {
	float height = VAL(LSHEIGHT);
	float scale = VAL(LSSCALE);
	if (depth <= 0)
		return;
	glPushMatrix();
		glPushMatrix();
			float innerRadius = min((baseRadius + baseRadius*scale) / 6,height/3);
			glTranslated(0, 0, height / 3);
			drawSphere(innerRadius);
			float shiftSphere = (baseRadius + baseRadius*scale * 2) / 3 / 2;
			glTranslated(0, 0, height / 3);
			glPushMatrix();
				glTranslated(shiftSphere, shiftSphere, 0);
				drawSphere(innerRadius);
			glPopMatrix();
			glPushMatrix();
			glTranslated(-shiftSphere, shiftSphere, 0);
			drawSphere(innerRadius);
			glPopMatrix();
			glPushMatrix();
			glTranslated(shiftSphere, -shiftSphere, 0);
			drawSphere(innerRadius);
			glPopMatrix();
			glPushMatrix();
			glTranslated(-shiftSphere, -shiftSphere, 0);
			drawSphere(innerRadius);
			glPopMatrix();
		glPopMatrix();
		glTranslated(0, 0, height + VAL(LSSPACE));
		float shift = baseRadius*scale*0.5;
		glPushMatrix();
		glTranslated(shift, shift, 0);
		drawLA(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
		glPushMatrix();
		glTranslated(shift, -shift, 0);
		drawLC(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
		glPushMatrix();
		glTranslated(-shift, shift, 0);
		drawLB(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
		glPushMatrix();
		glTranslated(-shift, -shift, 0);
		drawLA(baseRadius*scale*0.5, depth - 1);
		glPopMatrix();
	glPopMatrix();
	return;
}

void drawTriangle(const double p1[3], const double p2[3], const double p3[3]) {
	drawTriangle(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], p3[0], p3[1], p3[2]);
}

void drawTrapezoidPrism(double h, double w1, double l1, double w2, double l2)
{
	double a[3] = { -w1 / 2,h / 2,-l1 / 2 };
	double b[3] = { w1 / 2,h / 2,-l1 / 2 };
	double c[3] = { w1 / 2,h / 2,l1 / 2 };
	double d[3] = { -w1 / 2,h / 2,l1 / 2 };
	double e[3] = { -w2 / 2,-h / 2,-l2 / 2 };
	double f[3] = { w2 / 2,-h / 2,-l2 / 2 };
	double g[3] = { w2 / 2,-h / 2,l2 / 2 };
	double hp[3] = { -w2 / 2,-h / 2,l2 / 2 };
	glPushMatrix();
	//12 triangles

	drawTriangle(a, c, b);
	drawTriangle(a, d, c);

	drawTriangle(a, b, f);
	drawTriangle(a, e, f);
	drawTriangle(b, c, g);
	drawTriangle(b, g, f);
	drawTriangle(c, d, hp);
	drawTriangle(c, g, hp);
	drawTriangle(d, a, e);
	drawTriangle(d, e, hp);

	drawTriangle(e, g, f);
	drawTriangle(e, hp, g);

	glPopMatrix();
}


void drawQuadTextured(const double p1[3], const double p2[3], const double p3[3], const double p4[3]) {
	double a, b, c, d, e, f;
	a = p2[0] - p1[0];
	b = p2[1] - p1[1];
	c = p2[2] - p1[2];

	d = p3[2] - p1[2];
	e = p3[2] - p1[2];
	f = p3[2] - p1[2];

	glPushMatrix();
		glEnable(GL_TEXTURE_2D);
		glBegin(GL_QUADS);
		glNormal3d(b*f - c*e, c*d - a*f, a*e - b*d);
		glTexCoord2f(0, 0); glVertex3f(p1[0], p1[1], p1[2]);
		glTexCoord2f(0, 1); glVertex3f(p2[0], p2[1], p2[2]);
		glTexCoord2f(1, 1); glVertex3f(p3[0], p3[1], p3[2]);
		glTexCoord2f(1, 0); glVertex3f(p4[0], p4[1], p4[2]);
		glEnd();
		glDisable(GL_TEXTURE_2D);
	glPopMatrix();

}

void drawTrapezoidPrismTextured(double h, double w1, double l1, double w2, double l2)
{
	double a[3] = { -w1 / 2,h / 2,-l1 / 2 };
	double b[3] = { w1 / 2,h / 2,-l1 / 2 };
	double c[3] = { w1 / 2,h / 2,l1 / 2 };
	double d[3] = { -w1 / 2,h / 2,l1 / 2 };
	double e[3] = { -w2 / 2,-h / 2,-l2 / 2 };
	double f[3] = { w2 / 2,-h / 2,-l2 / 2 };
	double g[3] = { w2 / 2,-h / 2,l2 / 2 };
	double hp[3] = { -w2 / 2,-h / 2,l2 / 2 };

	glPushMatrix();
		//6 quad
		drawQuadTextured(a, b, c, d);

		drawQuadTextured(a, b, f, e);
		drawQuadTextured(b, c, g, f);
		drawQuadTextured(c, d, hp, g);
		drawQuadTextured(d, a, e, hp);

		drawQuadTextured(d, e, f, g);
	glPopMatrix();
}


void drawTrapezoidPrismShift_w(double h, double w1, double l1, double w2, double l2)
{
	double a[3] = { 0,h / 2,-l1 / 2 };
	double b[3] = { w1 ,h / 2,-l1 / 2 };
	double c[3] = { w1 ,h / 2,l1 / 2 };
	double d[3] = { 0,h / 2,l1 / 2 };
	double e[3] = { 0,-h / 2,-l2 / 2 };
	double f[3] = { w2 ,-h / 2,-l2 / 2 };
	double g[3] = { w2 ,-h / 2,l2 / 2 };
	double hp[3] = {0,-h / 2,l2 / 2 };
	glPushMatrix();
	//12 triangles

	drawTriangle(a, c, b);
	drawTriangle(a, d, c);

	drawTriangle(a, b, f);
	drawTriangle(a, e, f);
	drawTriangle(b, c, g);
	drawTriangle(b, g, f);
	drawTriangle(c, d, hp);
	drawTriangle(c, g, hp);
	drawTriangle(d, a, e);
	drawTriangle(d, e, hp);

	drawTriangle(e, g, f);
	drawTriangle(e, hp, g);

	glPopMatrix();
}

void drawTorus(double innerRadius, double outterRadius)
{
	ModelerDrawState *mds = ModelerDrawState::Instance();
	int divisions;

	switch (mds->m_quality)
	{
	case HIGH:
		divisions = 32; break;
	case MEDIUM:
		divisions = 20; break;
	case LOW:
		divisions = 12; break;
	case POOR:
		divisions = 8; break;
	}

	float twoPI = 2 * M_PI;

	for (int i = 0; i != divisions; i++) {
		glBegin(GL_QUAD_STRIP);
		for (int j = 0; j != divisions + 1; j++) {
			for (int k = 0; k != 2; k++) {

				double x, y, z;

				x = (innerRadius*cos(j * twoPI / (divisions)) + outterRadius) * cos((i + k) * twoPI / (divisions - 1));
				y = (innerRadius*sin(j * twoPI / (divisions)));
				z = (innerRadius*cos(j * twoPI / (divisions)) + outterRadius) * sin((i + k) *twoPI / (divisions - 1));

				glVertex3d(x, y, z);
			}


		}
		glEnd();
	}
}



void legTexture(GLuint textureID) {
	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, textureImageLeg);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
}

// To make a SampleModel, we inherit off of ModelerView
class SampleModel : public ModelerView 
{
	ParticleSystem* left_head_ps;
	InverseKinematics leftLegIK;
public:
    SampleModel(int x, int y, int w, int h, char *label) 
        : ModelerView(x,y,w,h,label) {	
		left_head_ps = new ParticleSystem();
		ModelerApplication::Instance()->SetParticleSystem(left_head_ps);
	}

	~SampleModel() {
		if (textureImage!=NULL)delete textureImage;
		if (textureImageLeg!=NULL)delete textureImageLeg;
	}

    virtual void draw();
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createSampleModel(int x, int y, int w, int h, char *label)
{ 
    return new SampleModel(x,y,w,h,label); 
}

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out SampleModel
void SampleModel::draw()
{
    // This call takes care of a lot of the nasty projection 
    // matrix stuff.  Unless you want to fudge directly with the 
	// projection matrix, don't bother with this ...
    ModelerView::draw();
	//background color
	glClearColor(VAL(BGCOLORR), VAL(BGCOLORG), VAL(BGCOLORB), 1.0);

	GLfloat lightPosition0[4] = {VAL(L1X), VAL(L1Y), VAL(L1Z), 0};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition0);
	GLfloat lightIntensity0[4] = { VAL(LINTENSITY), VAL(LINTENSITY), VAL(LINTENSITY), 1 };
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightIntensity0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightIntensity0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightIntensity0);

	//load image
	if (textureImage == NULL) {
		int width, height;
		textureImage = readBMP("clothesTexture.bmp", width, height);
	}
	if (VAL(CHANGELEGTEXTURE) == 1 && textureImageLeg == NULL) {
		int width, height;
		textureImageLeg = readBMP("textureLeg2.bmp", width, height);
	}

	//animation: walk
	float anim_rightLeg_angle, anim_leftLeg_angle, anim_rightThigh_angle, anim_leftThigh_angle, anim_upperBody_angle;
	anim_rightLeg_angle = anim_leftLeg_angle = anim_rightThigh_angle = anim_leftThigh_angle= anim_upperBody_angle = 0;
	if (ModelerApplication::Instance()->m_animating) {
		frameTimer = frameTimer>=60? 0: frameTimer + 1;
		float front_Thigh_max = 30;
		float front_Leg_max = 25;
		float back_Thigh_max = 15;
		float back_Leg_max = 10;
		float upperBody_max = 8;
		if (frameTimer<30) {
			float ratio = frameTimer<15?(float)frameTimer/15: 1.0f - ((float)frameTimer - 15) / 15;
			anim_rightThigh_angle = ratio * -front_Thigh_max;
			anim_rightLeg_angle = ratio*front_Leg_max;
			anim_leftThigh_angle = ratio*back_Thigh_max;
			anim_leftLeg_angle = ratio*back_Leg_max;
			anim_upperBody_angle = ratio*upperBody_max;
		}
		else{
			float ratio = frameTimer<45?(float)(frameTimer-30) / 15: 1.0f - ((float)frameTimer - 45) / 15;
			anim_leftThigh_angle = ratio * -front_Thigh_max;
			anim_leftLeg_angle = ratio*front_Leg_max;
			anim_rightThigh_angle = ratio * back_Thigh_max;
			anim_rightLeg_angle = ratio * back_Leg_max;
			anim_upperBody_angle = ratio*-upperBody_max;
		}
	}
		
	// Create one OpenGL texture
	GLuint textureID;
	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, textureImage);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glDisable(GL_TEXTURE_2D);
	
	// draw the floor
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(COLOR_GREEN);
	glPushMatrix();
		glTranslated(-10,0,-10);
		drawBox(20,0.01f,20);
	glPopMatrix();

	// draw the model
	setAmbientColor(.1f,.1f,.1f);
	glPushMatrix();
	glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));
	Mat4d model_offset = { 1,0,0,VAL(XPOS),
		0,1,0,VAL(YPOS),
		0,0,1,VAL(ZPOS),
		0,0,0,1 };

		float skinColor[3] = { (float)243 / 255, (float)225 / 255, (float)210 / 255 };
		float clothesColor[3] = { (float)15 / 255, (float)10 / 255, (float)10 / 255 };

		//1 unit = 10 cm
		float headRadius = VAL(HEADRADIUS);
		float torseWidth = 20 * 0.1;
		float bodyThickness = VAL(BODYTHICKNESS) * 0.1;
		float chestHeight = 18 * 0.1;
		float shoulderClothesRadius = 4 * 0.1;
		float shoulderRadius = 3 * 0.1;
		float waistHeight_upper = 12 * 0.1;
		float waistHeight_lower = 10 * 0.1;
		float armRadius = 2.5 * 0.1;
		float armLength = 25 * 0.1;
		float thighHeight = VAL(THIGHHEIGHT) * 0.1;
		float legHeight = VAL(LEGHEIGHT) * 0.1;
		float pantsRatio = 0.3;
		float footHeight = 5 * 0.1;
		float hipRadius = 7 * 0.1;
		float thighRadius = 6 * 0.1; 
		float legRadius = 3.5 * 0.1;
		float footRadius = 2 * 0.1;
		float hipShift = torseWidth / 4; 
		float toeHeight = 0.2;
		float headScale = VAL(HEADSCALE);

		Vec4f ikResult;

		//upper body
		glPushMatrix();
			glTranslated(0, footHeight+ legHeight+ thighHeight+ hipShift / 2 + waistHeight_lower+ waistHeight_upper+ chestHeight, 0.0);
			glRotated(VAL(UPPERBODYANGLE)+anim_upperBody_angle, 0, 1, 0);

			Mat4d body_trans = { 1,0,0,0,
				0,1,0,footHeight + legHeight + thighHeight + hipShift / 2 + waistHeight_lower + waistHeight_upper + chestHeight,
				0,0,1,0,
				0,0,0,1 };

			Mat4d body_rot = { cos(VAL(UPPERBODYANGLE) + anim_upperBody_angle),0,sin(VAL(UPPERBODYANGLE) + anim_upperBody_angle),0,
				0,1,0,0,
				-sin(VAL(UPPERBODYANGLE) + anim_upperBody_angle),0,cos(VAL(UPPERBODYANGLE) + anim_upperBody_angle),0,
				0,0,0,1 };


			//head and neck
			glPushMatrix();
				glTranslated(0.0, 2.0, 0.0);
				if (VAL(LOD) >= 1) {
				// draw head
				glPushMatrix();
					setDiffuseColor(skinColor[0], skinColor[1], skinColor[2]);
					glScaled(1, headScale,1);
					glTranslated(0, -headRadius *headScale/2, headRadius*0.1);
					drawSphere(headRadius);
				glPopMatrix();
				}
				// draw neck
				glTranslated(0, -headRadius*headScale * 3 / 4, -headRadius*0.1);
				glRotated(90, 1.0, 0.0, 0.0);
				setDiffuseColor(skinColor[0], skinColor[1], skinColor[2]);
				drawCylinder(1.75, 0.28, 0.32);
			glPopMatrix();

			//torso and arms
			glPushMatrix();
				if (VAL(LOD) >0) {
				//upper torso
				glPushMatrix();
					setDiffuseColor(clothesColor[0], clothesColor[1], clothesColor[2]);
					if (VAL(LOD) >0) {
					//chest
					glPushMatrix();
						glTranslated(0.0, -chestHeight/2, 0.0);
						drawTrapezoidPrismTextured(chestHeight, torseWidth, bodyThickness*4/5, torseWidth , bodyThickness);
					glPopMatrix();
					//waist
					glPushMatrix();
						glTranslated(0.0, -chestHeight - waistHeight_upper /2, 0.0);
						drawTrapezoidPrismTextured(waistHeight_upper, torseWidth, bodyThickness, torseWidth*3/4, bodyThickness*3/4);
					glPopMatrix();
					glPushMatrix();
						glTranslated(0.0, -chestHeight - waistHeight_upper - waistHeight_lower /2, 0.0);
						drawTrapezoidPrismTextured(waistHeight_lower, torseWidth * 3 / 4, bodyThickness * 3 / 4, torseWidth, bodyThickness);
					glPopMatrix();
					}
				glPopMatrix();
				glPushMatrix();//arms
					if (VAL(LOD) >1) {
					glPushMatrix();//right arm
						glTranslated(torseWidth / 2 + armRadius, 0, 0);
						//shoulder
						setDiffuseColor(clothesColor[0], clothesColor[1], clothesColor[2]);
						glTranslated(0, -shoulderRadius, 0);
						drawSphere(shoulderClothesRadius);
						glRotated(90.0, 1.0, 0.0, 0.0);
						//right upper arm
						if (VAL(CHANGEHANDS)) {
							setDiffuseColor(COLOR_GOLD); 
							glRotated(15.0, 0.0, 1.0, 0.0);
						}
						else {
							setDiffuseColor(skinColor[0], skinColor[1], skinColor[2]);
						}
						drawCylinder(armLength, shoulderRadius, armRadius);
						//right forearm
						if (VAL(LOD) >2) {
						glPushMatrix();
							glTranslated(0.0, 0, armLength);
							if (!VAL(CHANGEHANDS)) { 
								glPushMatrix();
								glRotated(VAL(RIGHTHANDANGLE), 1, 0, 0);
								drawCylinder(armLength, armRadius, armRadius);
								glPopMatrix();
							}
							else {
								drawCylinder(armLength*1.2, armRadius, armRadius * 2.5);
								if (VAL(LOD)>3){//L-system
									glPushMatrix();
									glTranslated(0.0, 0.0, armLength * 1.2 + VAL(LSSPACE));
									drawLA(armRadius*2.5, VAL(LSDEPTH));
									glPopMatrix();
								}
							}
						glPopMatrix();
						}
					glPopMatrix();
					glPushMatrix();//left arm
						glTranslated(-torseWidth / 2 - armRadius, 0, 0);

						Mat4d left_arm1 = { 1,0,0,-torseWidth / 2 - armRadius,
							0,1,0,0,
							0,0,1,0,
							0,0,0,1 };

						//shoulder
						setDiffuseColor(clothesColor[0], clothesColor[1], clothesColor[2]);
						glTranslated(0, -shoulderRadius, 0);

						Mat4d left_arm2 = { 1,0,0,0,
							0,1,0,-shoulderRadius,
							0,0,1,0,
							0,0,0,1 };

						drawSphere(shoulderClothesRadius);
						glRotated(90.0, 1.0, 0.0, 0.0);

						Mat4d left_arm3 = { 1,0,0,0,
							0,0,-1,0,
							0,1,0,0,
							0,0,0,1 };


						//left upper arm
						if (VAL(CHANGEHANDS)) {
							setDiffuseColor(COLOR_GOLD);
							glRotated(-15.0, 0.0, 1.0, 0.0);
						}
						else {
							setDiffuseColor(skinColor[0], skinColor[1], skinColor[2]);
						}
						drawCylinder(armLength, shoulderRadius, armRadius);
						//left forearm
						if (VAL(LOD) >2) {
						glPushMatrix();
							glTranslated(0.0, 0, armLength);

							Mat4d left_arm4 = { 1,0,0,0,
								0,1,0,0,
								0,0,1, armLength*2,
								0,0,0,1 };


							//ps
							Mat4d leftHandOffSet = model_offset * body_trans*body_rot*left_arm1*left_arm2*left_arm3*left_arm4;
							left_head_ps->setTrans_matrix(leftHandOffSet);

							if (!VAL(CHANGEHANDS))drawCylinder(armLength, armRadius, armRadius);
							else {
								drawCylinder(armLength * 1.2, armRadius, armRadius * 2.5);
								if (VAL(LOD)>3) {//L-system
									glPushMatrix();
									glTranslated(0.0, 0.0, armLength * 1.2 + VAL(LSSPACE));
									drawLA(armRadius*2.5, VAL(LSDEPTH));
									glPopMatrix();
								}
							}
						glPopMatrix();
						}
					glPopMatrix();
					}
				glPopMatrix();//end of arms
				}
			glPopMatrix();//arms and torso
		glPopMatrix();//end of upper body
		//lower body
		glPushMatrix();
			setDiffuseColor(clothesColor[0], clothesColor[1], clothesColor[2]);
			glTranslated(0, footHeight + legHeight + thighHeight + hipShift / 2, 0.0);
			if (VAL(LOD) >0) {
			glPushMatrix();//hip
				glTranslated(0, 0, -hipShift*0.15);		
				if (VAL(LOD) > 1) {
				glPushMatrix();//right
					glTranslated(hipShift, 0, 0);
					drawSphere(hipRadius);
				glPopMatrix();
				glPushMatrix();//left
					glTranslated(-hipShift, 0, 0);
					drawSphere(hipRadius);
				glPopMatrix();
				}
			glPopMatrix();//end of hip
			}
			glPushMatrix();//legs

			

				if (VAL(LOD) >1) {
				glPushMatrix();//right leg
					glTranslated(-thighRadius, 0, 0);
					//pants
					glRotated(90.0+ VAL(RIGHTTHIGHANGLE)+anim_rightThigh_angle, 1.0, 0.0, 0.0);
					glTranslated(0, 0, hipShift / 2);
					setDiffuseColor(clothesColor[0], clothesColor[1], clothesColor[2]);
					drawCylinder(thighHeight * pantsRatio, thighRadius, thighRadius*pantsRatio +legRadius*(1- pantsRatio));
					//thigh
					if (VAL(CHANGELEGTEXTURE) == 1) {
						legTexture(textureID);
					}
					glTranslated(0, 0, thighHeight * pantsRatio);
					setDiffuseColor(skinColor[0], skinColor[1], skinColor[2]);
					drawCylinder(thighHeight * (1-pantsRatio), thighRadius*pantsRatio + legRadius*(1 - pantsRatio), legRadius);
					//knee
					glTranslated(0, 0, thighHeight * (1 - pantsRatio));
					drawSphere(legRadius);
					if (VAL(CHANGELEGTEXTURE) == 1) glDisable(GL_TEXTURE_2D);
					//right leg
					if (VAL(LOD) >2) {
					glPushMatrix();
						if (VAL(CHANGELEGTEXTURE) == 1)legTexture(textureID);
						setDiffuseColor(skinColor[0], skinColor[1], skinColor[2]);
						glRotated(VAL(RIGHTLEGANGLE)+anim_rightLeg_angle, 1, 0, 0);
						drawCylinder(legHeight, legRadius, footRadius);
						//ankle
						glTranslated(0, 0, legHeight);
						drawSphere(footRadius);
						if (VAL(CHANGELEGTEXTURE) == 1) glDisable(GL_TEXTURE_2D);
						if (VAL(LOD) >3) {
						glPushMatrix();//foot
							setDiffuseColor(clothesColor[0], clothesColor[1], clothesColor[2]);
							glTranslated(0, -footRadius, (footHeight - 0.2) / 2);
							glRotated(-90, 0, 1, 0);
							glRotated(90, 0, 0, 1);
							drawTrapezoidPrismShift_w((footHeight - toeHeight), footRadius * 2, footRadius * 2, 2, footRadius * 2);
							glTranslated(0, -((footHeight - toeHeight) / 2 + toeHeight / 2), 0 );
							drawTrapezoidPrismShift_w(toeHeight, 2, footRadius * 2, 2, footRadius * 2);
						glPopMatrix();//end of foot
						}
					glPopMatrix();//end of right leg
					}
				glPopMatrix();//end of right leg
				}
				if (VAL(LOD) >1) {

					//for IK
					/*Vec4f ikResult;*/
					if (VAL(INVERSEKINEMATICS)) {
						//leftLegIK.SetOrigin(thighRadius, thighHeight + legHeight + footHeight, 0);
						leftLegIK.SetOrigin(thighRadius, 0, 0);
						leftLegIK.SetArmlength1(thighHeight);
						leftLegIK.SetArmlength2(legHeight + footHeight);
						//leftLegIK.SetQ(VAL(LEFTTHIGHANGLE_X), VAL(LEFTTHIGHANGLE), 0, VAL(LEFTLEGANGLE));

						Vec3f destination; destination[0] = VAL(IKX); destination[1] = VAL(IKY) ; destination[2] = VAL(IKZ);
						destination *= 0.1;
						destination[1] -= (thighHeight + legHeight + footHeight);
						destination[0] += thighRadius;

						glPushMatrix();
						setDiffuseColor(0, 0, 1);
						glTranslated(destination[0] - 0.25, destination[1] - 0.25, destination[2] - 0.25);
						drawBox(0.5, 0.5, 0.5);

						glPopMatrix();

						ikResult = leftLegIK.getResult(destination);
					}
					else {
						//leftLegIK.SetQ(VAL(LEFTTHIGHANGLE_X), VAL(LEFTTHIGHANGLE), 0, VAL(LEFTLEGANGLE));
						leftLegIK.SetQ(0, VAL(LEFTTHIGHANGLE), 0, VAL(LEFTLEGANGLE));

					}

					glPushMatrix();//left leg
					glTranslated(thighRadius, 0, 0);
					//pants
					if (VAL(INVERSEKINEMATICS)) {
						glRotated( 90 + ikResult[1], 1, 0, 0);
						glRotated(ikResult[0], 0, 1, 0);
						//printf("%f, %f   ", ikResult[0], ikResult[1]);
					}
					else {
						glRotated(90.0 + VAL(LEFTTHIGHANGLE) + anim_leftThigh_angle, 1.0, 0.0, 0.0);
						//glRotated(VAL(LEFTTHIGHANGLE_X), 0.0, 1.0, 0.0);
					}

					glTranslated(0, 0, hipShift / 2);
					setDiffuseColor(clothesColor[0], clothesColor[1], clothesColor[2]);
					drawCylinder(thighHeight * pantsRatio, thighRadius, thighRadius*pantsRatio + legRadius*(1 - pantsRatio));
					//thigh
					glTranslated(0, 0, thighHeight * pantsRatio);
					setDiffuseColor(skinColor[0], skinColor[1], skinColor[2]);
					if (VAL(CHANGELEGTEXTURE) == 1)legTexture(textureID);
					drawCylinder(thighHeight * (1 - pantsRatio), thighRadius*pantsRatio + legRadius*(1 - pantsRatio), legRadius);
					//knee
					glTranslated(0, 0, thighHeight * (1 - pantsRatio));
					drawSphere(legRadius);

					if (VAL(CHANGELEGTEXTURE) == 1)glDisable(GL_TEXTURE_2D);
					//left leg
					if (VAL(LOD) >2) {
						glPushMatrix();
						if (VAL(CHANGELEGTEXTURE) == 1)legTexture(textureID);
						setDiffuseColor(skinColor[0], skinColor[1], skinColor[2]);
						if (VAL(INVERSEKINEMATICS)) {
							glRotated(ikResult[3], 1, 0, 0);
							glRotated(ikResult[2], 0, 1, 0);
							//printf("%f, %f   ", ikResult[2], ikResult[3]);
						}
						else {
							glRotated(VAL(LEFTLEGANGLE) + anim_leftLeg_angle, 1, 0, 0);
						}
						drawCylinder(legHeight, legRadius, footRadius);
						//ankle
						glTranslated(0, 0, legHeight);
						drawSphere(footRadius);
						if (VAL(CHANGELEGTEXTURE) == 1)glDisable(GL_TEXTURE_2D);
						if (VAL(LOD) >3) {
							glPushMatrix();//foot
							setDiffuseColor(clothesColor[0], clothesColor[1], clothesColor[2]);
							glTranslated(0, -footRadius, (footHeight - 0.2) / 2);
							glRotated(-90, 0, 1, 0);
							glRotated(90, 0, 0, 1);
							drawTrapezoidPrismShift_w((footHeight - toeHeight), footRadius * 2, footRadius * 2, 2, footRadius * 2);
							glTranslated(0, -((footHeight - toeHeight) / 2 + toeHeight / 2), 0);
							drawTrapezoidPrismShift_w(toeHeight, 2, footRadius * 2, 2, footRadius * 2);
							glPopMatrix();//end of foot
						}
						glPopMatrix();//end of left leg
					}
					glPopMatrix();//end of left leg
				}


				//test

				//leftLegIK.SetOrigin(thighRadius, thighHeight + legHeight + footHeight,0);
				//leftLegIK.SetArmlength1(thighHeight);
				//leftLegIK.SetArmlength2(legHeight + footHeight);
				//leftLegIK.SetQ(VAL(LEFTTHIGHANGLE_X), VAL(LEFTTHIGHANGLE), 0, VAL(LEFTLEGANGLE));
				//leftLegIK.SetQ(ikResult[0], ikResult[1], ikResult[2], ikResult[3]);
/*
				Vec3f endpoint = leftLegIK.ForwardKinematics();

				glPushMatrix();
				setDiffuseColor(1, 0, 0);
				glTranslated(endpoint[0] - 0.25, endpoint[1] - 0.25, endpoint[2] - 0.25);
				drawBox(0.5, 0.5, 0.5);

				glPopMatrix();

				glPushMatrix();
				setDiffuseColor(0, 1, 0);
				glTranslated(leftLegIK.joint[0] - 0.25, leftLegIK.joint[1] - 0.25, leftLegIK.joint[2] - 0.25);
				drawBox(0.5, 0.5, 0.5);

				glPopMatrix();*/
				//testend



			glPopMatrix();//end of legs
		glPopMatrix();// end of lower body
	
		
	glPopMatrix();
	endDraw();
}

int main()
{
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	// stepsize, defaultvalue)
    ModelerControl controls[NUMCONTROLS];
	controls[LOD] = ModelerControl("Level of Detail", 0, 4, 1.0f, 4);
	controls[L1X] = ModelerControl("Light X Position", -30, 30, 1, 4);
	controls[L1Y] = ModelerControl("Light Y Position", -30, 30, 1, 2);
	controls[L1Z] = ModelerControl("Light Z Position", -30, 30, 1, -4);
	controls[BGCOLORR] = ModelerControl("Background Color R", 0, 1, 0.01f, 1);
	controls[BGCOLORG] = ModelerControl("Background Color G", 0, 1, 0.01f, 1);
	controls[BGCOLORB] = ModelerControl("Background Color B", 0, 1, 0.01f, 1);
    controls[XPOS] = ModelerControl("X Position", -5, 5, 0.1f, 0);
    controls[YPOS] = ModelerControl("Y Position", 0, 5, 0.1f, 0);
    controls[ZPOS] = ModelerControl("Z Position", -5, 5, 0.1f, 0);
	controls[RIGHTHANDANGLE] = ModelerControl("Right Hand Angle", -180, 0, 1, 0.0);
	controls[HEADRADIUS] = ModelerControl("Head Radius", 0.5, 1.5, 0.1f, 0.9f);
	controls[HEADSCALE] = ModelerControl("Head Y-Sccale", 1, 1.5, 0.1f, 1.2f);
	controls[BODYTHICKNESS] = ModelerControl("Body Thickness", 8, 18, 1, 13.0);
	controls[THIGHHEIGHT] = ModelerControl("Thigh Length", 20, 60, 1, 40.0);
	controls[LEGHEIGHT] = ModelerControl("Leg Length", 20, 60, 1, 35.0);
	controls[RIGHTTHIGHANGLE] = ModelerControl("Right Thigh Angle", -90, 90, 1, 0.0);
	controls[LEFTTHIGHANGLE] = ModelerControl("Left Thigh Angle", -90, 90, 1, 0.0);
	//controls[LEFTTHIGHANGLE_X] = ModelerControl("Left Thigh Angle_x", 0, 50, 1, 0.0);
	controls[RIGHTLEGANGLE] = ModelerControl("Right Leg Angle", 0, 90, 1, 0.0);
	controls[LEFTLEGANGLE] = ModelerControl("Left Leg Angle", 0, 90, 1, 0.0);
	controls[UPPERBODYANGLE] = ModelerControl("Upper Body Angle", -90, 90, 1, 0.0);
    controls[LINTENSITY] = ModelerControl("Light Intensity", -1, 1, 0.01f, 0.5);
	controls[CHANGELEGTEXTURE] = ModelerControl("Change Legs Texture", 0, 1, 1, 0);

	controls[CHANGEHANDS] = ModelerControl("Change Hands (Display L-System)", 0, 1, 1, 0);
	controls[LSDEPTH] = ModelerControl("L-System: depth", 0, 5, 1, 3);
	controls[LSHEIGHT] = ModelerControl("L-System: height", 0, 2, 0.1, 1.2);
	controls[LSSPACE] = ModelerControl("L-System: space", 0, 1, 0.1f, 0.1);
	controls[LSSCALE] = ModelerControl("L-System: scale", 1, 3, 0.1f, 1.6);

	controls[INVERSEKINEMATICS] = ModelerControl("Inverse Kinematics", 0, 1, 1, 0);
	controls[IKX] = ModelerControl("target x", -50, 50, 1, 0);
	controls[IKY] = ModelerControl("target y", -50, 50, 1, 0);
	controls[IKZ] = ModelerControl("target z", -50, 50, 1, 50);
	//controls[IKCONSTRAINT] = ModelerControl("IK constraints", 0, 1, 1, 0);
	//controls[ROTATE] = ModelerControl("Rotate", -135, 135, 1, 0);


	controls[FRAMEALL] = ModelerControl("Frame All", 0, 1, 1, 0);
	textureImage = NULL;
	textureImageLeg = NULL;

    ModelerApplication::Instance()->Init(&createSampleModel, controls, NUMCONTROLS);
    return ModelerApplication::Instance()->Run();
}