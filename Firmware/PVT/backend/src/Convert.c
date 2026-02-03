//----------------------------------------------------------------------
// Convert.c:
//   functions to convert between different data
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "DataTypes.h"
#include <math.h>

static int DaysAcc[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define CUBIC(x) ((x)*(x)*(x))

//*************** convert from ECEF coordinate to LLH coordinate ****************
// Parameters:
//   ecef_pos: pointer to ECEF coordinate
//   llh_pos: pointer to LLH coordinate
// Return value:
//   none
void EcefToLlh(const KINEMATIC_INFO *ecef_pos, LLH *llh_pos)
{
	double p;
	double theta;
	double n;

	if (!llh_pos || !ecef_pos)
		return;
	
	p = sqrt(ecef_pos->x * ecef_pos->x + ecef_pos->y * ecef_pos->y);

	if( p < 1e-10)	// north or south pole
	{
		llh_pos->lon = 0;
		llh_pos->lat = PI / 2;
		llh_pos->hae = (ecef_pos->z > 0) ? ecef_pos->z - WGS_AXIS_B : -ecef_pos->z - WGS_AXIS_B;
		return;
	}

	theta = atan(ecef_pos->z * WGS_AXIS_A / (p * WGS_AXIS_B));
	llh_pos->lat = atan((ecef_pos->z + WGS_E2_SQR * WGS_AXIS_B * CUBIC(sin (theta))) /
								  (p - WGS_E1_SQR * WGS_AXIS_A * CUBIC(cos (theta))));
	llh_pos->lon = atan2(ecef_pos->y, ecef_pos->x);
	
	n = WGS_AXIS_A / sqrt(1.0 - WGS_E1_SQR * sin(llh_pos->lat) * sin(llh_pos->lat));
	llh_pos->hae = p / cos(llh_pos->lat) - n;
}

//*************** convert from LLH coordinate to ECEF coordinate ****************
// Parameters:
//   llh_pos: pointer to LLH coordinate
//   ecef_pos: pointer to ECEF coordinate
// Return value:
//   none
void LlhToEcef(const LLH *llh_pos, KINEMATIC_INFO *ecef_pos)
{
	double n;

	if (!llh_pos || !ecef_pos)
		return;

	n = WGS_AXIS_A / sqrt(1.0 - WGS_E1_SQR * sin(llh_pos->lat) * sin(llh_pos->lat));

	ecef_pos->x = (n + llh_pos->hae) * cos (llh_pos->lat) * cos (llh_pos->lon);
	ecef_pos->y = (n + llh_pos->hae) * cos (llh_pos->lat) * sin (llh_pos->lon);
	ecef_pos->z = (n * (1.0 - WGS_E1_SQR) + llh_pos->hae) * sin (llh_pos->lat);  
}

//*************** convert from ECEF velocity to local coordinate ****************
//* to avoid duplicate calculation, position in KINEMATIC_INFO
// Parameters:
//   ecef_pos: pointer to ECEF coordinate
//   llh_pos: pointer to LLH coordinate
// Return value:
//   none
void VelocityToLocal(const KINEMATIC_INFO *ecef_pos, const PCONVERT_MATRIX pConvertMatrix, GROUND_SPEED *local_speed)
{
	local_speed->ve = (pConvertMatrix->x2e * ecef_pos->vx) + (pConvertMatrix->y2e * ecef_pos->vy);
	local_speed->vn = (pConvertMatrix->x2n * ecef_pos->vx) + (pConvertMatrix->y2n * ecef_pos->vy) + (pConvertMatrix->z2n * ecef_pos->vz);
	local_speed->vu = (pConvertMatrix->x2u * ecef_pos->vx) + (pConvertMatrix->y2u * ecef_pos->vy) + (pConvertMatrix->z2u * ecef_pos->vz);
	local_speed->Speed = sqrt(local_speed->ve * local_speed->ve + local_speed->vn + local_speed->vn);
	local_speed->Course = atan2(local_speed->ve, local_speed->vn);
	if (local_speed->Course < 0.0)
		local_speed->Course += (PI * 2);
}

//*************** convert from GLONASS time to UTC time ****************
// Parameters:
//   LeapYears: leap year
//   DayNumber: day number within 4 year
//   DayMsCount: millisecond within day
//   pUtcTime: pointer to UTC time
// Return value:
//   none
void GlonassTimeToUtc(int LeapYears, int DayNumber, int DayMsCount, PSYSTEM_TIME pUtcTime)
{
	int i, Seconds, LeapDay = 0;

	DayMsCount -= 10800000;
	if (DayMsCount < 0)
	{
		DayMsCount += 86400000;
		DayNumber --;
	}
	Seconds = DayMsCount / 1000;
	pUtcTime->Millisecond = DayMsCount - Seconds * 1000;
	LeapYears *= 4;
	DayNumber --;
	if (DayNumber >= (366 + 365 * 2))
	{
		DayNumber -= (366 + 365 * 2);
		LeapYears += 3;
	}
	else if (DayNumber >= (366 + 365))
	{
		DayNumber -= (366 + 365);
		LeapYears += 2;
	}
	else if (DayNumber >= 366)
	{
		DayNumber -= 366;
		LeapYears ++;
	}
	else if (DayNumber >= 60)
		DayNumber --;
	else if (DayNumber == 59)
		LeapDay = 1;

	for (i = 1; i < 12; i ++)
	{
		if (DayNumber < DaysAcc[i])
			break;
	}
	if (LeapDay)
	{
		pUtcTime->Month = 2;
		pUtcTime->Day = 29;
	}
	else
	{
		pUtcTime->Month = i;
		pUtcTime->Day = DayNumber - (DaysAcc[i-1] - 1);
	}
	pUtcTime->Year = 1992 + LeapYears;
	pUtcTime->Hour = Seconds / 3600;
	Seconds -= pUtcTime->Hour * 3600;
	pUtcTime->Minute = Seconds / 60;
	pUtcTime->Second = Seconds - pUtcTime->Minute * 60;
}

//*************** convert from UTC time to GLONASS time ****************
// Parameters:
//   pUtcTime: pointer to UTC time
//   pLeapYears: pointer to leap year
//   pDayNumber: pointer to day number within 4 year
//   pDayMsCount: pointer to millisecond within day
// Return value:
//   none
void UtcToGlonassTime(const PSYSTEM_TIME pUtcTime, int *LeapYears, int *DayNumber, int *DayMsCount)
{
	int Years, Days;

	*DayMsCount = (((pUtcTime->Hour * 60) + pUtcTime->Minute) * 60 + pUtcTime->Second) * 1000 + pUtcTime->Millisecond + 10800000;
	Years = pUtcTime->Year - 1992;
	Days = DaysAcc[pUtcTime->Month - 1] + pUtcTime->Day - 1;
	if ((Years % 4) != 0 || Days >= 59)
		Days ++;
	Days += (Years % 4) * 365;
	*DayNumber = Days + 1;
	*LeapYears = Years / 4;
}

//*************** convert from GPS time to UTC time ****************
//* This program handles the date from Jan. 1, 1984 00:00:00.00 UTC
//* till year 2099 !!! (do not treat year 2100 as common year)
// Parameters:
//   GpsWeek: GPS week number
//   WeekMsCount: millisecond within week
//   pUtcTime: pointer to UTC time
//   pUtcParam: pointer to UTC parameter (if it is NULL, do not apply leap second)
// Return value:
//   none
void GpsTimeToUtc(int GpsWeek, int WeekMsCount, PSYSTEM_TIME pUtcTime, PUTC_PARAM pUtcParam)
{
	int LeapYears, TotalDays, MsSeconds;

	// calculate total days and seconds
	// to prevent seconds less than zero after leap second adjust
	// add seconds of one week
	TotalDays = (GpsWeek - 1) * 7;
	MsSeconds = WeekMsCount + 604800000;
	if (pUtcParam)
	{
		if (pUtcParam->flag)
		{
			MsSeconds -= pUtcParam->TLS * 1000;
			if (pUtcParam->TLS != pUtcParam->TLSF)
			{
				if ((GpsWeek > pUtcParam->WNLSF) ||
					((GpsWeek == pUtcParam->WNLSF) && (MsSeconds / 86400000) > (pUtcParam->DN + 7)))
				{
					MsSeconds -= (pUtcParam->TLSF - pUtcParam->TLS) * 1000;
				}
			}
		}
		else	// default value
		{
			MsSeconds -= 18 * 1000;
		}
	}
	TotalDays += MsSeconds / 86400000;
	MsSeconds %= 86400000;

	// calculate year
	TotalDays -= 208 * 7;
	LeapYears = TotalDays / (366 + 365 * 3);
	TotalDays -= LeapYears * (366 + 365 * 3);

	GlonassTimeToUtc(LeapYears - 2, TotalDays + 1, MsSeconds + 10800000, pUtcTime);
}

//*************** convert from UTC time to GPS time ****************
// Parameters:
//   pUtcTime: pointer to UTC time
//   pGpsWeek: pointer to GPS week number
//   pWeekMsCount: pointer to millisecond within week
//   pUtcParam: pointer to UTC parameter
// Return value:
//   none
void UtcToGpsTime(const PSYSTEM_TIME pUtcTime, int *pGpsWeek, int *pWeekMsCount, PUTC_PARAM pUtcParam)
{
	int LeapYears, TotalDays, MsSeconds;

	UtcToGlonassTime(pUtcTime, &LeapYears, &TotalDays, &MsSeconds);
	MsSeconds -= 10800000;
	TotalDays --;	// convert to 1 based day count
	if (pUtcParam && pUtcParam->flag)
	{
		MsSeconds += pUtcParam->TLS * 1000;
	}
	else	// default value
	{
		MsSeconds += 18 * 1000;
	}
	if (MsSeconds >= 86400000)
	{
		MsSeconds -= 86400000;
		TotalDays ++;
	}
	else if (MsSeconds < 0)
	{
		MsSeconds += 86400000;
		TotalDays --;
	}
	TotalDays += (LeapYears + 2) * (366 + 365 * 3);
	*pGpsWeek  = TotalDays / 7 + 208;
	*pWeekMsCount = (TotalDays % 7) * 86400000 + MsSeconds;
}

//*************** calculate onversion matrix based on ECEF position ****************
//* this is a approximation calculation to avoid sin/cos calculation
//* |e|   |-y/P      x/P     0  | |x|
//* |n| = |-x*z/P/R -y*z/P/R P/R|*|y|
//* |u|   | x/R      y/R     z/R| |z|
// Parameters:
//   pReceiverPos: pointer to ECEF position
//   pConvertMatrix: pointer to conversion matrix
// Return value:
//   none
void CalcConvMatrix(const KINEMATIC_INFO *pReceiverPos, PCONVERT_MATRIX pConvertMatrix)
{
	double P, R;

	P = pReceiverPos->x * pReceiverPos->x + pReceiverPos->y * pReceiverPos->y;
	R = P + pReceiverPos->z * pReceiverPos->z;
	P = sqrt(P);
	R = sqrt(R);

	if (R < 1e-5)
		return;
	if (P < 1e-5)
	{
		pConvertMatrix->x2e = 0.0;
		pConvertMatrix->y2e = 1.0;
	}
	else
	{
		pConvertMatrix->x2e = -pReceiverPos->y / P;
		pConvertMatrix->y2e =  pReceiverPos->x / P;
	}
	pConvertMatrix->x2u =  pReceiverPos->x / R;
	pConvertMatrix->y2u =  pReceiverPos->y / R;
	pConvertMatrix->z2u =  pReceiverPos->z / R;
	pConvertMatrix->x2n = -pConvertMatrix->y2e * pConvertMatrix->z2u;
	pConvertMatrix->y2n =  pConvertMatrix->x2e * pConvertMatrix->z2u;
	pConvertMatrix->z2n =  P / R;
}

//*************** calculate DOP values based on xyz DOP and conversion matrix ****************
//* this is a approximation calculation to avoid sin/cos calculation
//*            | x2e y2e  0  |      | P[0] P[1] P[3] |
//* Cxyz2enu = | x2n y2n z2n |, P = | P[1] P[2] P[4] |
//*            | x2u y2u z2u |      | P[3] P[4] P[5] |
//* DOP matrix at ENU coordinate is Cxyz2enu*P*Cxyz2enu'
//* only diagonal elements are needed
//* TDOP is square root of P[9]
// Parameters:
//   PosInvMatrix: DOP matrix on xyz coordinate
//   pConvertMatrix: pointer to conversion matrix
//   DopArray: DOP value array
// Return value:
//   none
void CalcDopValues(const double *PosInvMatrix, const PCONVERT_MATRIX pConvertMatrix, double *DopArray)
{
	double x2, y2, z2, xy, xz, yz;
	double p0, p1, p2, p3, p4, p5;
	double pe, pn, pu;

	if (PosInvMatrix[0] <= 0)	// invalid
	{
		DopArray[0] = DopArray[1] = DopArray[2] = DopArray[3] = 99.;
		return;
	}

	p0 = PosInvMatrix[0]; p2 = PosInvMatrix[2]; p5 = PosInvMatrix[5];
	p1 = 2. * PosInvMatrix[1]; p3 = 2. * PosInvMatrix[3]; p4 = 2. * PosInvMatrix[4];

	x2 = pConvertMatrix->x2e * pConvertMatrix->x2e; y2 = pConvertMatrix->y2e * pConvertMatrix->y2e; xy = pConvertMatrix->x2e * pConvertMatrix->y2e;
	pe = x2 * p0 + xy * p1 + y2 * p2;

	x2 = pConvertMatrix->x2n * pConvertMatrix->x2n; y2 = pConvertMatrix->y2n * pConvertMatrix->y2n; z2 = pConvertMatrix->z2n * pConvertMatrix->z2n;
	xy = pConvertMatrix->x2n * pConvertMatrix->y2n; xz = pConvertMatrix->x2n * pConvertMatrix->z2n; yz = pConvertMatrix->y2n * pConvertMatrix->z2n;
	pn = x2 * p0 + y2 * p2 + z2 * p5 + xy * p1 + xz * p3 + yz * p4;

	x2 = pConvertMatrix->x2u * pConvertMatrix->x2u; y2 = pConvertMatrix->y2u * pConvertMatrix->y2u; z2 = pConvertMatrix->z2u * pConvertMatrix->z2u;
	xy = pConvertMatrix->x2u * pConvertMatrix->y2u; xz = pConvertMatrix->x2u * pConvertMatrix->z2u; yz = pConvertMatrix->y2u * pConvertMatrix->z2u;
	pu = x2 * p0 + y2 * p2 + z2 * p5 + xy * p1 + xz * p3 + yz * p4;

	if (pe < 0. || pn < 0. || pu < 0. || PosInvMatrix[9] < 0.)
	{
		DopArray[0] = DopArray[1] = DopArray[2] = DopArray[3] = 99.;
		return;
	}
	DopArray[0] = sqrt(pe + pn);
	DopArray[1] = sqrt(pu);
	DopArray[2] = sqrt(pe + pn + pu);
	DopArray[3] = sqrt(PosInvMatrix[9]);
}
