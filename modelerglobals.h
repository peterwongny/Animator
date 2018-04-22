#ifndef _MODELER_GLOBALS_H
#define _MODELER_GLOBALS_H

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502
#endif

// This is a list of the controls for the SampleModel
// We'll use these constants to access the values 
// of the controls from the user interface.
enum SampleModelControls
{
	LOD, XPOS, YPOS, ZPOS,
	L1X, L1Y, L1Z, LINTENSITY,
	BGCOLORR, BGCOLORG, BGCOLORB,

	HEADRADIUS, HEADSCALE, BODYTHICKNESS, THIGHHEIGHT, LEGHEIGHT,
	UPPERBODYANGLE, RIGHTTHIGHANGLE, LEFTTHIGHANGLE, RIGHTLEGANGLE, LEFTLEGANGLE,

	CHANGELEGTEXTURE, CHANGEHANDS,
	LSHEIGHT, LSSPACE, LSSCALE,LSDEPTH,
	FRAMEALL,
	INVERSEKINEMATICS, IKX, IKY, IKZ, //IKCONSTRAINT,
	NUMCONTROLS
};

// Colors
#define COLOR_RED		1.0f, 0.0f, 0.0f
#define COLOR_GREEN		0.0f, 1.0f, 0.0f
#define COLOR_BLUE		0.0f, 0.0f, 1.0f
#define COLOR_GOLD		0.5426f, 0.4574f, 0.0f

// We'll be getting the instance of the application a lot; 
// might as well have it as a macro.
#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))


#endif