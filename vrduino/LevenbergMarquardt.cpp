#include "LevenbergMarquardt.h"
#include "MatrixMath.h"
#include "Utils.h"

// inputs : photoDiodeLocations - 1x8 array of displacements of photodiodes
// 								  relative to center in mm. 
LevenbergMarquardtPose::LevenbergMarquardtPose(const float* photoDiodeLocations) {
	memcpy(object2D, photoDiodeLocations, 8 * sizeof(float));
}

// Given x, computes h as h=g(x). Maps angles/positions to homography values.
// inputs: x - 1x6 array of current estimate on rotation/translation parameters
// 			   [thetaX thetaY thetaZ X Y Z]
// outputs: h - 1x9 array of homography values
// 				[h1 h2 h3 h4 h5 h6 h7 h8 h9]
void LevenbergMarquardtPose::eval_g_at_x(const float *x, float *h)
{
	float thetaX = x[0];
	float thetaY = x[1];
	float thetaZ = x[2];
	float tX = x[3];
	float tY = x[4];
	float tZ = x[5];
	h[0] = cos(thetaY)*cos(thetaZ) - sin(thetaX)*sin(thetaY)*sin(thetaZ);
	h[1] = -cos(thetaX)*sin(thetaZ);
	h[2] = tX;
	h[3] = cos(thetaY)*sin(thetaZ) + sin(thetaX)*sin(thetaY)*cos(thetaZ);
	h[4] = cos(thetaX)*cos(thetaZ);
	h[5] = tY;
	h[6] = cos(thetaX)*sin(thetaY);
	h[7] = -sin(thetaX);
	h[8] = -tZ;

	// for (int i = 0; i < 6; i++)
	// 	Serial.printf("%f\n",x[i]);
	// for (int i = 0; i < 9; i++)
	// 	Serial.printf("%f\n",h[i]);
}

// Given h, computes f as f=g(h). Map from homography values to x,y coordinates
// of each of the photodiodes on the light house "camera"
// inputs: h - 1x9 array of homography values
// 			   [h1 h2 h3 h4 h5 h6 h7 h8 h9]
// outputs: f - 1x8 array of estimated x,y coordinates of all 4 photodiodes
//			    [x1 y1 x2 y2 ... x4 y4]
void LevenbergMarquardtPose::eval_f_at_h(const float *h, float *f)
{
	for (int i = 0; i < 4; i++) {
		float x = object2D[2*i];
		float y = object2D[2*i+1];
		f[2*i] = (h[0]*x + h[1]*y + h[2]) / (h[6]*x + h[7]*y + h[8]);
		f[2*i+1] = (h[3]*x + h[4]*y + h[5]) / (h[6]*x + h[7]*y + h[8]);
	}
	// for (int i = 0; i < 8; i++) {
	// 	Serial.println(f[i]);
	// }
}

// Compute jacobian matrix for g(x) algebraically
// inputs: x - 1x6 array of current estimate on rotation/translation parameters
// 			   [thetaX thetaY thetaZ X Y Z]
// outputs: Jg - 9x6 2D array jacobian of g(x)
void LevenbergMarquardtPose::get_jacobian_g(const float *x, float Jg[][6])
{
	float thetaX = x[0];
	float thetaY = x[1];
	float thetaZ = x[2];

	Jg[0][0] = -cos(thetaX)*sin(thetaY)*sin(thetaZ);
	Jg[0][1] = -sin(thetaY)*cos(thetaZ) - sin(thetaX)*cos(thetaY)*sin(thetaZ);
	Jg[0][2] = -cos(thetaY)*sin(thetaZ) - sin(thetaX)*sin(thetaY)*cos(thetaZ);
	Jg[0][3] = 0.0;
	Jg[0][4] = 0.0;
	Jg[0][5] = 0.0;

	Jg[1][0] = sin(thetaX)*sin(thetaZ);
	Jg[1][1] = 0.0;
	Jg[1][2] = -cos(thetaX)*cos(thetaZ);
	Jg[1][3] = 0.0;
	Jg[1][4] = 0.0;
	Jg[1][5] = 0.0;

	Jg[2][0] = 0.0;
	Jg[2][1] = 0.0;
	Jg[2][2] = 0.0;
	Jg[2][3] = 1.0;
	Jg[2][4] = 0.0;
	Jg[2][5] = 0.0;

	Jg[3][0] = cos(thetaX)*sin(thetaY)*cos(thetaZ);
	Jg[3][1] = -sin(thetaY)*sin(thetaZ) + sin(thetaX)*cos(thetaY)*cos(thetaZ);
	Jg[3][2] = cos(thetaY)*cos(thetaZ) - sin(thetaX)*sin(thetaY)*sin(thetaZ);
	Jg[3][3] = 0.0;
	Jg[3][4] = 0.0;
	Jg[3][5] = 0.0;

	Jg[4][0] = -sin(thetaX)*cos(thetaZ);
	Jg[4][1] = 0.0;
	Jg[4][2] = -cos(thetaX)*sin(thetaZ);
	Jg[4][3] = 0.0;
	Jg[4][4] = 0.0;
	Jg[4][5] = 0.0;

	Jg[5][0] = 0.0;
	Jg[5][1] = 0.0;
	Jg[5][2] = 0.0;
	Jg[5][3] = 0.0;
	Jg[5][4] = 1.0;
	Jg[5][5] = 0.0;

	Jg[6][0] = -sin(thetaX)*sin(thetaY);
	Jg[6][1] = cos(thetaX)*cos(thetaY);
	Jg[6][2] = 0.0;
	Jg[6][3] = 0.0;
	Jg[6][4] = 0.0;
	Jg[6][5] = 0.0;

	Jg[7][0] = -cos(thetaX);
	Jg[7][1] = 0.0;
	Jg[7][2] = 0.0;
	Jg[7][3] = 0.0;
	Jg[7][4] = 0.0;
	Jg[7][5] = 0.0;

	Jg[8][0] = 0.0;
	Jg[8][1] = 0.0;
	Jg[8][2] = 0.0;
	Jg[8][3] = 0.0;
	Jg[8][4] = 0.0;
	Jg[8][5] = -1.0;

	// for (int j = 0; j < 6; j++) {
	// 	Serial.printf("%f,",x[j]);
	// }
	// Serial.printf("\n");

	// for (int i = 0; i < 9; i++) {
	// 	for (int j = 0; j < 6; j++) {
	// 		Serial.printf("%f,",Jg[i][j]);
	// 	}
	// 	Serial.printf("\n");
	// }
}

// Compute jacobian matrix for f(h) algebraically
// inputs: h - 1x9 array of homography values
// 			   [h1 h2 h3 h4 h5 h6 h7 h8 h9]
// outputs: Jf - 8x9 2D array jacobian of g(x)
void LevenbergMarquardtPose::get_jacobian_f(const float *h, float Jf[][9])
{
	for (int i = 0; i < 4; i++) {
		float x = object2D[2*i];
		float y = object2D[2*i+1];
		float numx = (h[0]*x + h[1]*y + h[2]);
		float denom = (h[6]*x + h[7]*y + h[8]);
		Jf[2*i][0] = x / denom;
		Jf[2*i][1] = y / denom;
		Jf[2*i][2] = 1.0 / denom;
		Jf[2*i][3] = 0.0;
		Jf[2*i][4] = 0.0;
		Jf[2*i][5] = 0.0;
		Jf[2*i][6] = - x * numx / (denom * denom);
		Jf[2*i][7] = - y * numx / (denom * denom);
		Jf[2*i][8] = - numx / (denom * denom);

		float numy = (h[3]*x + h[4]*y + h[5]);
		Jf[2*i+1][0] = 0.0;
		Jf[2*i+1][1] = 0.0;
		Jf[2*i+1][2] = 0.0;
		Jf[2*i+1][3] = x / denom;
		Jf[2*i+1][4] = y / denom;
		Jf[2*i+1][5] = 1.0 / denom;
		Jf[2*i+1][6] = -x * numy / (denom * denom);
		Jf[2*i+1][7] = -y * numy / (denom * denom);
		Jf[2*i+1][8] = - numy / (denom * denom);
	}
}

// Computes the step in which to move our current estimate on x.
// Returns false if matrix inverse operation could not be succesfully completed.
//
// inputs: J - 8x6 2D array of joint jacobian
//		   f - 1x8 array of estimated x,y coordinates of all 4 photodiodes
//			   [x1 y1 x2 y2 ... x4 y4]
// 		   projection2D - 1x8 array of measured x,y coordinates of all 4 photodiodes
//						  [x1 y1 x2 y2 ... x4 y4]
//
// outputs: deltaX - 1x6 array of the step in which to move x
//					 [deltaThetaX deltaThetaY deltaThetaZ deltaX deltaY deltaZ]
bool LevenbergMarquardtPose::compute_delta_x(const float* J, const float* f, const float* projection2D, float* deltaX) {
	MatrixMath mm;
	int inv = 0;
	float JT[6][8];
	mm.Transpose((float*)J, 8, 6, (float*)JT);
	float JTJ[6][6];
	mm.Multiply((float*)JT,(float*)J, 6,8,6,(float*)JTJ);
	for (int i = 0; i < 6; i++) {
		JTJ[i][i] += lambdaLM * JTJ[i][i];
	}
	if (!mm.Invert((float*)JTJ,6)) return false;

	float A[6][8];
	mm.Multiply((float*)JTJ,(float*)JT,6,6,8,(float*)A);

	for (int i = 0; i < 6; i++) {
		deltaX[i] = 0.0;
		for (int j = 0; j < 8; j++) {
			deltaX[i] += A[i][j]*(projection2D[j] - f[j]);
		}
	}

	// for (int i = 0; i < 6; i++) {
 //      Serial.printf("%f,", deltaX[i]);
 //    }

	return true;
}


// Estimates 3D pose of VRduino via Levengerg-Marquardt Algorithm.
// inputs : projection2D - 1x8 array of the x,y projection of each diode
//                         onto the light house "sensor" plane  
//			poseEstimate  -	1x6 array specifying the initial guess of the 3D
//							   	pose of the VRduino (could be random, output
//							   	from homography method, or estimate from last
// 							   	frame)
// output:  pose3D       - member variable storing VRduino 3D pose
// 						   [thetax, thetay, thetaz,x,y,z]
bool LevenbergMarquardtPose::computePosition(const float* projection2D,
        const float* poseEstimate) {

	MatrixMath mm;
	bool successLM = true;

	float x[6];
	memcpy(x,poseEstimate,6*sizeof(float));

	float h[9];
	float f[8];
	float Jf[8][9];
	float Jg[9][6];
	float J[8][6];
	float deltaX[6];

	residual[0] = get_residual(x,projection2D);

	for (int it = 1; it <= kMaxIters; it++) {
		eval_g_at_x(x,h);
		eval_f_at_h(h,f);
		get_jacobian_g(x, Jg);
		get_jacobian_f(h, Jf);
		mm.Multiply((float*)Jf,(float*)Jg,8,9,6,(float*)J);
		successLM &= compute_delta_x((float*)J,f,projection2D,deltaX);
		for (int i = 0; i < 6; i++) {
			x[i] += deltaX[i];
		}
		residual[it] = get_residual(x,projection2D);
	}

	memcpy(pose3D, x, 6*sizeof(float));

	return successLM;
}

// Compute L2 error between estimated x,y diode positions on light house
// "sensor" and the measured ones.
// inputs: x - 1x6 array of current estimate on rotation/translation parameters
// 			   [thetaX thetaY thetaZ X Y Z]
//  	   projection2D - 1x8 array of the x,y projection of each diode
//                        onto the light house "sensor" plane  
// outputs : scalar L2 error
float LevenbergMarquardtPose::get_residual(const float *x, const float *projection2D)
{
	//float h[12];
	float h[9] = {};
	eval_g_at_x(x, h);
	float f[8] = {};
	eval_f_at_h(h, f);

	return (projection2D[0] - f[0]) * (projection2D[0] - f[0]) + (projection2D[1] - f[1]) * (projection2D[1] - f[1]) + (projection2D[2] - f[2]) * (projection2D[2] - f[2]) + (projection2D[3] - f[3]) * (projection2D[3] - f[3]) + (projection2D[4] - f[4]) * (projection2D[4] - f[4]) + (projection2D[5] - f[5]) * (projection2D[5] - f[5]) + (projection2D[6] - f[6]) * (projection2D[6] - f[6]) + (projection2D[7] - f[7]) * (projection2D[7] - f[7]);
}

// Compute jacobian matrix for g(x) numerically. This is purely for debug
// purposes, as a reference to check if the algebraic jacobian is correctly
// implemented.
// inputs: x - 1x6 array of current estimate on rotation/translation parameters
// 			   [thetaX thetaY thetaZ X Y Z]
// outputs: Jg - 9x6 2D array jacobian of g(x)
void LevenbergMarquardtPose::get_jacobian_g_numeric(const float *x, float Jg[][6])
{

	float deltaX = 0.01;

	float h_x[9];
	eval_g_at_x(x, h_x);

	for (int i = 0; i < 6; i++) {

		float xx[6];
		memcpy(xx, x, 6 * sizeof(float));
		xx[i] += deltaX;

		float h_xx[9];
		eval_g_at_x(xx, h_xx);

		for (int j = 0; j < 9; j++)
		{
			Jg[j][i] = (h_xx[j] - h_x[j]) / deltaX;
		}
	}
}

// Compute jacobian matrix for f(h) numerically. This is purely for debug
// purposes, as a reference to check if the algebraic jacobian is correctly
// implemented.
// inputs: h - 1x9 array of homography values
// 			   [h1 h2 h3 h4 h5 h6 h7 h8 h9]
// outputs: Jg - 9x6 2D array jacobian of g(x)
void LevenbergMarquardtPose::get_jacobian_f_numeric(const float *h, float Jf[][9])
{

	float deltaX = 0.000001;

	float f_x[8];
	eval_f_at_h(h, f_x);

	for (int i = 0; i < 9; i++) {

		float hh[9];
		memcpy(hh, h, 9 * sizeof(float));
		hh[i] += deltaX;

		float f_xx[8];
		eval_f_at_h(hh, f_xx);

		for (int j = 0; j < 8; j++)
		{
			Jf[j][i] = (f_xx[j] - f_x[j]) / deltaX;
		}
	}
}

// Debug function to compare algebraic jacobians with numeric
void LevenbergMarquardtPose::check_jacobians(void)
{
	// Testing get_jacobian_g()
	float testx[6];
	for (int j = 0; j < 6; j++) {
		testx[j] = float(random(999999)) / 999999.0;
	}
	float Jg1[9][6];
	float Jg2[9][6];
	get_jacobian_g(testx, Jg1);
	get_jacobian_g_numeric(testx, Jg2);
	float errorJg = Utils::ComputeL2Error((float*)Jg1, (float*)Jg2, 9,6);
	Serial.printf("Error Jg: %.9g \n", errorJg);

	// Testing get_jacobian_f()
	float testh[9];
	for (int j = 0; j < 9; j++) {
		testh[j] = (float(random(999999)) / 999999.0);
	}
	float Jf1[8][9];
	float Jf2[8][9];
	get_jacobian_f(testh, Jf1);	
	get_jacobian_f_numeric(testh, Jf2);
	float errorJf = Utils::ComputeL2Error((float*)Jf1, (float*)Jf2, 8,9);
	Serial.printf("Error Jf: %.9g \n", errorJf);

}