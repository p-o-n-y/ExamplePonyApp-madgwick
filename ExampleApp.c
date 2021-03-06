#define _USE_MATH_DEFINES

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "pony.h"

void read_imu_from_file_aist(void);
void calibr(void);
void madgwick(void);
void write_quat_to_file(void);



int main()
{
	if ((
		pony->add_plugin(read_imu_from_file_aist)
		&& pony->add_plugin(calibr)
		&& pony->add_plugin(madgwick)
		&& pony->add_plugin(write_quat_to_file)
		))
	{
		if (pony->init("{imu: in = \"newData.txt\" Mwx = 7177.32; Mwy = 7848.36; Mwz = 7925.76; Mfx = 2.0019; Afx = -254.4204; Mfy = 2.0015; Afy = 261.7441; Mfz = 2.0009; Afz = -260.9625; fs = 250; error degree = 5} out = \"out3.txt\""));
		{
			while (pony->step());
		}
	}
	return 0;
}



void read_imu_from_file_aist(void)
{
	static FILE * fpimu = NULL;
	char buffer[1024];
	int scanned;
	static double* f = NULL, *w = NULL;

	if (pony->mode > 0)
	{
		if (feof(fpimu))
		{
			pony->mode = -1;
			return;
		}
		fgets(buffer, 1023, fpimu);

		scanned = sscanf(buffer, "%*g %lg %lg %lg  %lg %lg %lg", &w[0], &w[1], &w[2], &f[0], &f[1], &f[2]);
		if (scanned < 6)
		{
			pony->mode = -2;
		}
	}
	else if (pony->mode == 0)
	{
		f = pony->imu->f;
		w = pony->imu->w;
		char * fileName;
		int len, i;
		char* fnameptr;
		fnameptr = pony_locate_token("in = ", pony->imu->cfg, pony->imu->cfglength, 0);
		if (fnameptr!=NULL)
		{
			len = 0;
			while (fnameptr[len] != '"')
			{
				len++;
			}

			if (len == 0)
			{
				pony->mode = -100;
				return;
			}

			fileName = (char*)malloc(sizeof(char) * (len + 1));

			if (fileName == NULL)
			{
				pony->mode = -101;
				return;
			}

			for (i = 0; i < len; i++)
			{
				fileName[i] = fnameptr[i];
			}
			fileName[len] = '\0';
		}
		else
		{
			fileName = "imuDataIn.txt";
		}

		fpimu = fopen(fileName, "r");

		if (fpimu == NULL)
		{
			pony->mode = -102;
		}

		if (fnameptr != NULL)
		{
			free(fileName);
		}
	}
	else
	{
		if (fpimu != NULL)
		{
			fclose(fpimu);
			fpimu = NULL;
		}
		f = NULL;
		w = NULL;
	}
}

void calibr(void)
{
	static double Mwx, Mwy, Mwz, Mfx, Mfy, Mfz, Afx, Afy, Afz, Awx, Awy, Awz;
	static double* f = NULL, *w = NULL;

	if (pony->mode > 0)
	{
		w[0] = (w[0] - Awx) / Mwx / pony->imu_const.rad2deg; //gyroscope
		w[1] = (w[1] - Awy) / Mwy / pony->imu_const.rad2deg;
		w[2] = (w[2] - Awz) / Mwz / pony->imu_const.rad2deg;
		f[0] = (f[0] - Afx) / Mfx; //accelerometer
		f[1] = (f[1] - Afy) / Mfy;
		f[2] = (f[2] - Afz) / Mfz;
	}
	else if (pony->mode == 0)
	{
		f = pony->imu->f;
		w = pony->imu->w;
		char* numptr;

		numptr = pony_locate_token("Mwx =", pony->imu->cfg, pony->imu->cfglength, 0);
		Mwx = numptr == NULL ? 1 : atof(numptr);

		numptr = pony_locate_token("Mwy =", pony->imu->cfg, pony->imu->cfglength, 0);
		Mwy = numptr == NULL ? 1 : atof(numptr);

		numptr = pony_locate_token("Mwz =", pony->imu->cfg, pony->imu->cfglength, 0);
		Mwz = numptr == NULL ? 1 : atof(numptr);

		numptr = pony_locate_token("Mfx =", pony->imu->cfg, pony->imu->cfglength, 0);
		Mfx = numptr == NULL ? 1 : atof(numptr);

		numptr = pony_locate_token("Mfy =", pony->imu->cfg, pony->imu->cfglength, 0);
		Mfy = numptr == NULL ? 1 : atof(numptr);

		numptr = pony_locate_token("Mfz =", pony->imu->cfg, pony->imu->cfglength, 0);
		Mfz = numptr == NULL ? 1 : atof(numptr);

		numptr = pony_locate_token("Awx =", pony->imu->cfg, pony->imu->cfglength, 0);
		Awx = numptr == NULL ? 0 : atof(numptr);

		numptr = pony_locate_token("Awy =", pony->imu->cfg, pony->imu->cfglength, 0);
		Awy = numptr == NULL ? 0 : atof(numptr);

		numptr = pony_locate_token("Awz =", pony->imu->cfg, pony->imu->cfglength, 0);
		Awz = numptr == NULL ? 0 : atof(numptr);

		numptr = pony_locate_token("Afx =", pony->imu->cfg, pony->imu->cfglength, 0);
		Afx = numptr == NULL ? 0 : atof(numptr);

		numptr = pony_locate_token("Afy =", pony->imu->cfg, pony->imu->cfglength, 0);
		Afy = numptr == NULL ? 0 : atof(numptr);

		numptr = pony_locate_token("Afz =", pony->imu->cfg, pony->imu->cfglength, 0);
		Afz = numptr == NULL ? 0 : atof(numptr);
	}
	else
	{
		f = NULL;
		w = NULL;
	}
}

void madgwick(void)
{
	static double* f = NULL, *w = NULL, *q = NULL, *deltat = NULL;
	static double gyroMeasError, beta;

	if (pony->mode > 0)
	{
		double norm; // vector norm
		double SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
		double f_1, f_2, f_3; // objective function elements
		double J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; //objective function Jacobian elements

		double SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;
		// estimated direction of the gyroscope error
		// Axulirary variables to avoid reapeated calcualtions
		double halfSEq_1 = 0.5f * q[0];
		double halfSEq_2 = 0.5f * q[1];
		double halfSEq_3 = 0.5f * q[2];
		double halfSEq_4 = 0.5f * q[3];
		double twoSEq_1 = 2.0f * q[0];
		double twoSEq_2 = 2.0f * q[1];
		double twoSEq_3 = 2.0f * q[2];
		// Normalise the accelerometer measurement
		norm = sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
		f[0] /= norm;
		f[1] /= norm;
		f[2] /= norm;
		// Compute the objective function and Jacobian
		f_1 = twoSEq_2 * q[3] - twoSEq_1 * q[2] - f[0];
		f_2 = twoSEq_1 * q[1] + twoSEq_3 * q[3] - f[1];
		f_3 = 1.0f - twoSEq_2 * q[1] - twoSEq_3 * q[2] - f[2];
		J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
		J_12or23 = 2.0f * q[3];
		J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
		J_14or21 = twoSEq_2;
		J_32 = 2.0f * J_14or21; // negated in matrix multiplication
		J_33 = 2.0f * J_11or24; // negated in matrix multiplication

								// Compute the gradient (matrix multiplication)
		SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
		SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
		SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
		SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
		// Normalise the gradient
		norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
		SEqHatDot_1 /= norm;
		SEqHatDot_2 /= norm;
		SEqHatDot_3 /= norm;
		SEqHatDot_4 /= norm;
		// Compute the quaternion derrivative measured by gyroscopes
		SEqDot_omega_1 = -halfSEq_2 * w[0] - halfSEq_3 * w[1] -
			halfSEq_4 * w[2];
		SEqDot_omega_2 = halfSEq_1 * w[0] + halfSEq_3 * w[2] -
			halfSEq_4 * w[1];
		SEqDot_omega_3 = halfSEq_1 * w[1] - halfSEq_2 * w[2] +
			halfSEq_4 * w[0];
		SEqDot_omega_4 = halfSEq_1 * w[2] + halfSEq_2 * w[1] -
			halfSEq_3 * w[0];
		// Compute then integrate the estimated quaternion derrivative
		q[0] += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * *deltat;
		q[1] += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * *deltat;
		q[2] += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * *deltat;
		q[3] += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * *deltat;
		// Normalise quaternion
		norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		q[0] /= norm;
		q[1] /= norm;
		q[2] /= norm;
		q[3] /= norm;
	}
	else if (pony->mode == 0)
	{
		f = pony->imu->f;
		w = pony->imu->w;
		q = pony->sol.q;
		deltat = &(pony->sol.dt);
		q[0] = 1; //might read these from config later
		q[1] = 0;
		q[2] = 0;
		q[3] = 0;
		char* numptr;
		numptr = pony_locate_token("fs =", pony->imu->cfg, pony->imu->cfglength, 0);
		if (numptr != NULL)
		{
			*deltat = atof(numptr);
		}
		else
		{
			*deltat = 0.05;
		}
		numptr = pony_locate_token("error degree =", pony->imu->cfg, pony->imu->cfglength, 0);
		if (numptr != NULL)
		{
			gyroMeasError = atof(numptr) / pony->imu_const.rad2deg;
		}
		else
		{
			gyroMeasError = 5.0f / pony->imu_const.rad2deg;
		}

		beta = sqrt(3.0f / 4.0f) * gyroMeasError;
	}
	else
	{
		f = NULL;
		w = NULL;
		q = NULL;
		deltat = NULL;
	}
}

void write_quat_to_file(void)
{
	static FILE * fpout = NULL;
	static double* q = NULL;

	if (pony->mode > 0)
	{
		fprintf(fpout, "% +9.6f % +9.6f % +9.6f % +9.6f\n", q[0], q[1], q[2], q[3]);
	}
	else if (pony->mode == 0)
	{
		char* fileName;
		int len, i;
		char* fnameptr;
		q = pony->sol.q;
		fnameptr = pony_locate_token("out = ", pony->cfg, pony->cfglength, 0);
		if (fnameptr != NULL)
		{
			len = 0;
			while (fnameptr[len] != '"')
			{
				len++;
			}

			if (len == 0)
			{
				pony->mode = -200;
				return;
			}

			fileName = (char*)malloc(sizeof(char) * (len + 1));

			if (fileName == NULL)
			{
				pony->mode = -201;
				return;
			}

			for (i = 0; i < len; i++)
			{
				fileName[i] = fnameptr[i];
			}
			fileName[len] = '\0';
		}
		else
		{
			fileName = "dataOut.txt";
		}

		fpout = fopen(fileName, "w");

		if (fpout == NULL)
		{
			pony->mode = -202;
		}

		if (fnameptr != NULL)
		{
			free(fileName);
		}
	}
	else
	{
		if (fpout != NULL)
		{
			fclose(fpout);
			fpout = NULL;
		}
		q = NULL;
	}
}