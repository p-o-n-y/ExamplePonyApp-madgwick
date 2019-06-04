#define _USE_MATH_DEFINES

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "pony.h"

void read_imu_from_file_aist(void);
void calibr(void);
void madgwick(void);
void write_quat_to_file(void);



int main()
{
	if (!(
		pony_add_plugin(read_imu_from_file_aist)
		&& pony_add_plugin(calibr)
		&& pony_add_plugin(madgwick)
		&& pony_add_plugin(write_quat_to_file)
		))
	{
		if (pony_init("{imu: in = \"newData.txt\" Mwx = 7177.32; Mwy = 7848.36; Mwz = 7925.76; Mfx = 2.0019; Afx = -254.4204; Mfy = 2.0015; Afy = 261.7441; Mfz = 2.0009; Afz = -260.9625; fs = 250; error degree = 5} out = \"out3.txt\""));
		{
			while (pony_step());
		}
	}
	return 0;
}



void read_imu_from_file_aist(void)
{
	static char * fileName;
	static FILE * fpimu;
	static char buffer[1024];
	int scanned;
	static double* f, *w;

	if (pony.bus.mode > 0)
	{
		if (feof(fpimu))
		{
			pony.bus.mode = -1;
			return;
		}
		fgets(buffer, 1023, fpimu);

		scanned = sscanf(buffer, "%*g %lg %lg %lg  %lg %lg %lg", &w[0], &w[1], &w[2], &f[0], &f[1], &f[2]);
		if (scanned < 6)
		{
			pony.bus.mode = -1;
		}
	}
	else if (pony.bus.mode == 0)
	{
		f = (*pony.bus.imu).f.val;
		w = (*pony.bus.imu).w.val;
		int len;
		if (pony_extract_string_length((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "in = \"", &len))
		{
			fileName = malloc(sizeof(char*) * len + 1);
			pony_extract_string((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "in = \"", &fileName);
			fileName[len] = '\0';
		}
		else
		{
			fileName = "imuDataIn.txt";
		}

		fpimu = fopen(fileName, "r");

	}
	else
	{
		fclose(fpimu);

		free(fileName);
	}
}

void calibr(void)
{
	static float Mwx, Mwy, Mwz, Mfx, Mfy, Mfz, Afx, Afy, Afz, Awx, Awy, Awz;
	static double* f, *w;

	if (pony.bus.mode > 0)
	{
		double temp[3];
		temp[0] = ((w[0] - Awx) * M_PI) / (1.9937 * 648000); //gyroscope
		temp[1] = ((w[1] - Awy) * M_PI) / (2.1801 * 648000); //Mwx = ; Mwy = ; Mwz = 
		temp[2] = ((w[2] - Awz) * M_PI) / (2.2016 * 648000);

		w[0] = (w[0] - Awx) / Mwx * (M_PI / 180); //gyroscope
		w[1] = (w[1] - Awy) / Mwy * (M_PI / 180);
		w[2] = (w[2] - Awz) / Mwz * (M_PI / 180);
		f[0] = (f[0] - Afx) / Mfx; //accelerometer
		f[1] = (f[1] - Afy) / Mfy;
		f[2] = (f[2] - Afz) / Mfz;
	}
	else if (pony.bus.mode == 0)
	{
		f = (*pony.bus.imu).f.val;
		w = (*pony.bus.imu).w.val;
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Mwx = ", &Mwx))
		{
			Mwx = 1;
		}
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Mwy = ", &Mwy))
		{
			Mwy = 1;
		}
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Mwz = ", &Mwz))
		{
			Mwz = 1;
		}
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Mfx = ", &Mfx))
		{
			Mfx = 1;
		}
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Mfy = ", &Mfy))
		{
			Mfy = 1;
		}
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Mfz = ", &Mfz))
		{
			Mfz = 1;
		}
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Afx = ", &Afx))
		{
			Afx = 0;
		}
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Afy = ", &Afy))
		{
			Afy = 0;
		}
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Afz = ", &Afz))
		{
			Afz = 0;
		}
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Awx = ", &Awx))
		{
			Awx = 0;
		}
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Awy = ", &Awy))
		{
			Awy = 0;
		}
		if (!pony_extract_float((*pony.bus.imu).cfg, (*pony.bus.imu).cfglength, "Awz = ", &Awz))
		{
			Awz = 0;
		}
	}
	else
	{

	}
}

void madgwick(void)
{
	static double* f, *w, *q, *deltat;
	static double gyroMeasError, beta;

	if (pony.bus.mode > 0)
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

								// Compute the gradient (matrixmultiplication)
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
	else if (pony.bus.mode == 0)
	{
		double rate, degree;
		f = (*pony.bus.imu).f.val;
		w = (*pony.bus.imu).w.val;
		q = (*pony.bus.imu).q.val;
		deltat = &pony.bus.imu->dt.val;
		q[0] = 1; //might read these from config later
		q[1] = 0;
		q[2] = 0;
		q[3] = 0;

		if (pony_extract_double(pony.bus.imu->cfg, pony.bus.imu->cfglength, "error degree = ", &degree))
		{
			gyroMeasError = M_PI * (degree / 180.0f);
		}
		else
		{
			gyroMeasError = M_PI * (5.0f / 180.0f);
		}

		beta = sqrt(3.0f / 4.0f) * gyroMeasError;
	}
	else
	{

	}
}

void write_quat_to_file(void)
{
	static char* fileName;
	static FILE * fpout;
	static double* q;

	if (pony.bus.mode > 0)
	{
		fprintf(fpout, "% +9.6f % +9.6f % +9.6f % +9.6f\n", q[0], q[1], q[2], q[3]);
	}
	else if (pony.bus.mode == 0)
	{
		q = (*pony.bus.imu).q.val;
		int len;
		if (pony_extract_string_length(pony.bus.cfg, pony.bus.cfglength, "out = \"", &len))
		{
			fileName = malloc(sizeof(char*) * len + 1);
			pony_extract_string(pony.bus.cfg, pony.bus.cfglength, "out = \"", &fileName);
			fileName[len] = '\0';
		}
		else
		{
			fileName = "dataOut.txt";
		}

		fpout = fopen(fileName, "w");
	}
	else
	{
		fclose(fpout);
	}
}