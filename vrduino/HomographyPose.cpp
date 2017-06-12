#include "HomographyPose.h"
#include "MatrixMath.h"
#include "Utils.h"

// inputs : photoDiodeLocations - displacements of photodiodes relative to center
HomographyPose::HomographyPose(float* photoDiodeLocations) {
  memcpy(object2D, photoDiodeLocations, 8 * sizeof(float));
}

// Updates A and b member variables based on new feature projection information
// inputs : projection2D - 1x8 array of the x,y projection of each diode
//                         onto the light house "sensor" plane
void HomographyPose::updateParameters(float* projection2D) {
	memcpy(b, projection2D, 8 * sizeof(float));
	/*A[0] = { object2D[0], object2D[1], 1, 0, 0, 0, -1.0*object2D[0] * b[0], -1.0*object2D[1] * b[0] };
	A[1] = { 0, 0, 0, object2D[0], object2D[1], 1, -1.0*object2D[1] * b[1], -1.0*object2D[1] * b[1] };
	A[2] = { object2D[2], object2D[3], 1, 0, 0, 0, -1.0*object2D[2] * b[2], -1.0*object2D[3] * b[2] };
	A[3] = { 0, 0, 0, object2D[2], object2D[3], 1, -1.0*object2D[3] * b[2], -1.0*object2D[3] * b[3] };
	A[4] = { object2D[4], object2D[5], 1, 0, 0, 0, -1.0*object2D[4] * b[4], -1.0*object2D[5] * b[4] };
	A[5] = { 0, 0, 0, object2D[4], object2D[5], 1, -1.0*object2D[5] * b[4], -1.0*object2D[5] * b[5] };
	A[6] = { object2D[6], object2D[7], 1, 0, 0, 0, -1.0*object2D[6] * b[6], -1.0*object2D[7] * b[6] };
	A[7] = { 0, 0, 0, object2D[6], object2D[7], 1, -1.0*object2D[7] * b[6], -1.0*object2D[7] * b[7] };*/
	float A_mat [8][8] = {	{ object2D[0], object2D[1], 1, 0, 0, 0, -1.0*object2D[0] * b[0], -1.0*object2D[1] * b[0] },
							{ 0, 0, 0, object2D[0], object2D[1], 1, -1.0*object2D[0] * b[1], -1.0*object2D[1] * b[1] },
							{ object2D[2], object2D[3], 1, 0, 0, 0, -1.0*object2D[2] * b[2], -1.0*object2D[3] * b[2] },
							{ 0, 0, 0, object2D[2], object2D[3], 1, -1.0*object2D[2] * b[3], -1.0*object2D[3] * b[3] },
							{ object2D[4], object2D[5], 1, 0, 0, 0, -1.0*object2D[4] * b[4], -1.0*object2D[5] * b[4] },
							{ 0, 0, 0, object2D[4], object2D[5], 1, -1.0*object2D[4] * b[5], -1.0*object2D[5] * b[5] },
							{ object2D[6], object2D[7], 1, 0, 0, 0, -1.0*object2D[6] * b[6], -1.0*object2D[7] * b[6] },
							{ 0, 0, 0, object2D[6], object2D[7], 1, -1.0*object2D[6] * b[7], -1.0*object2D[7] * b[7] } };
	
	memcpy(A, A_mat, 64 * sizeof(float));
}

// inputs : A,b member variables
// output : h member variable
bool HomographyPose::solveForHomography() {
	if (Matrix.Invert((float *)A, 8) == 0) return false;
	Matrix.Multiply((float *)A, (float *)b, 8, 8, 1, (float*)h);
	return true;
}

// inputs : h member variable
// outputs: normalization factor - normalization factor to reduce the columnns
//                                 of the rotations of the homography to sum
//                                 to one.
float HomographyPose::solveFor3DPosition() {
  float normalizationFactor = 2.0/(sqrt(h[0]*h[0] + h[3]*h[3] + h[6]*h[6]) + sqrt(h[1]*h[1] + h[4]*h[4] + h[7]*h[7]));

  position3D[0] = normalizationFactor * h[2];
  position3D[1] = normalizationFactor * h[5];
  position3D[2] = -normalizationFactor;
  
  return normalizationFactor;
}

// Computes the 3D position of an object based on the photodiode features
// projected onto a plane a unit distance from the light house. The function
// returns false if it is not able to compute the inverse of the marix A in
// solveForHomography().
// inputs : projection2D - 1x8 array of the x,y projection of each diode
//                         onto the light house "sensor" plane
// outputs : position3D member variable
bool HomographyPose::computePosition(float* projection2D) {
	alpha = 0.8;
	updateParameters(projection2D);
	if (!solveForHomography()) return false;
	float prior_z = position3D[2];
	float normFactor = solveFor3DPosition();
	position3D[2] = alpha * prior_z + (1 - alpha) * position3D[2];
	residual = computeResidual(projection2D, normFactor);
  return true;
}

// inputs : projection2D        - 1x8 array of the x,y projection of each diode
//                                onto the light house "sensor" plane
//          normalizationFactor - normalization factor to reduce the columnns
//                                of the rotations of the homography to sum
//                                to one.
// 
// outputs : res                - residual computed with L2 loss metric
float HomographyPose::computeResidual(float* projection2D, float normalizationFactor) {
  float res = 0.0;
  float Rt[3][3] = { {h[0],h[1],h[2]},{h[3],h[4],h[5]},{h[6],h[7],1.0} }; // For simplicity, this was left unscaled, since the final computation cancels the scale.
  for (int i = 0; i < 4; i++) {
	  float px = (h[0] * object2D[2 * i] + h[1] * object2D[2 * i + 1] + h[2]) / (h[6] * object2D[2 * i] + h[7] * object2D[2 * i + 1] + 1);
	  float py = (h[3] * object2D[2 * i] + h[4] * object2D[2 * i + 1] + h[5]) / (h[6] * object2D[2 * i] + h[7] * object2D[2 * i + 1] + 1);
	  res =  res + (px - projection2D[2 * i]) * (px - projection2D[2 * i]) + (py - projection2D[2 * i + 1]) * (py - projection2D[2 * i + 1]);
  }
  return res;
}
