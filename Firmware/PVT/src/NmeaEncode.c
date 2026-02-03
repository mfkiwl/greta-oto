//----------------------------------------------------------------------
// NmeaEncode.c:
//   Do NMEA0183 encode
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------
#include <math.h>
#include <string.h>
#include "GlobalVar.h"
#include "SupportPackage.h"
#include "NmeaEncode.h"

#define APPEND_CHAR(p, character) do { *((p)++) = (character); } while(0)
#define MAX_GSV_SATS 36    // NMEA0183 standard, maximum 9 GSV sentences with each sentence has maximum 4 satellites

int NmeaOutputInterval[NMEA_MAX];

static const TalkerChar[] = { 'N', 'P', 'B', 'A', 'L' };
static int GnssSystemID[] = { 0, 1, 4, 3, 2 };
static int SvidBaseValue[] = { 1, 1, 1, 1, 65 };

static char *NmeaTerminate(char *String);
static char *ComposeGsv(int System, char *StringStart);

int NMEAEncode(PNMEA_INFO NmeaInfo, unsigned int NmeaMask, char *NmeaString)
{
	char LatLonString[64];
	char AltituteString[64];
	char TimeString[16];
	char DateString[16];
	char SpeedString[16];
	char CourseString[16];
	char HdopString[16];
	char VdopString[16];
	char PdopString[16];
	int Quality, TalkerIndex;
	char *StringStart = NmeaString, *p;
	double Lat, Lon;
	int TempInt, LatInt, LonInt;
	int SignFlag = 0;
	int MsgType, PosSystem, SatCount, SatIndex;
	unsigned int PosFlag;
	unsigned long long SatInUse;

	// set quality and flags
	Quality = (NmeaInfo->PosQuality > KeepPos) ? 1 : 0;
	PosFlag = g_SystemConfig.PvtConfigFlags & PVT_CONFIG_USE_SYS;
	TalkerIndex = (PosFlag == PVT_CONFIG_USE_GPS) ? 1 : (PosFlag == PVT_CONFIG_USE_BDS) ? 2 : (PosFlag == PVT_CONFIG_USE_GAL) ? 3 : 0;
	if (TalkerIndex == 0 && Quality)
	{
		PosFlag = NmeaInfo->PosFlag & PVT_USE_SYS;
		TalkerIndex = (PosFlag == PVT_USE_GPS) ? 1 : (PosFlag == PVT_USE_BDS) ? 2 : (PosFlag == PVT_USE_GAL) ? 3 : 0;
	}

	// latitude/longitude string update
	if ((NmeaMask & (MSG_MASK(GGA) | MSG_MASK(GLL) | MSG_MASK(RMC))) != 0)
	{
		p = LatLonString;
		APPEND_CHAR(p, ',');
		Lat = NmeaInfo->PosLLH.lat * 180 / PI;
		Lon = NmeaInfo->PosLLH.lon * 180 / PI;
		SignFlag |= (Lat < 0) ? 1 : 0;
		SignFlag |= (Lon < 0) ? 2 : 0;
		Lat = fabs(Lat);
		Lon = fabs(Lon);
		LatInt = (int)Lat;
		LonInt = (int)Lon;

		TempInt = (int)((Lat - LatInt) * 600000000. + 5);    // resolution to 1/1000000 minute, add one digit to round up
		if (TempInt >= 600000000)    // round up to whole degree
		{
			LatInt ++;
			TempInt = 0;
		}
		TempInt /= 10;
		p = PrintUint(p, LatInt, 2);
		p = PrintUint(p, TempInt / 1000000, 2);
		APPEND_CHAR(p, '.');
		p = PrintUint(p, TempInt % 1000000, 6);
		APPEND_CHAR(p, ',');
		APPEND_CHAR(p, (SignFlag & 1) ? 'S' : 'N');
		APPEND_CHAR(p, ',');

		TempInt = (int)((Lon - LonInt) * 600000000. + 5);    // resolution to 1/1000000 minute, add one digit to round up
		if (TempInt >= 600000000)    // round up to whole degree
		{
			LonInt ++;
			TempInt = 0;
		}
		TempInt /= 10;
		p = PrintUint(p, LonInt, 3);
		p = PrintUint(p, TempInt / 1000000, 2);
		APPEND_CHAR(p, '.');
		p = PrintUint(p, TempInt % 1000000, 6);
		APPEND_CHAR(p, ',');
		APPEND_CHAR(p, (SignFlag & 2) ? 'W' : 'E');
		APPEND_CHAR(p, '\0');
	}

	// if altitute string needed
	if ((NmeaMask & MSG_MASK(GGA)) != 0)
	{
		p = AltituteString;
		APPEND_CHAR(p, ',');
		p = PrintFloat(p, NmeaInfo->PosLLH.hae, 3);
		APPEND_CHAR(p, ',');
		APPEND_CHAR(p, 'M');
		APPEND_CHAR(p, ',');
		APPEND_CHAR(p, '0');
		APPEND_CHAR(p, ',');
		APPEND_CHAR(p, 'M');
		APPEND_CHAR(p, '\0');
	}

	// if time string needed
	if ((NmeaMask & (MSG_MASK(GGA) | MSG_MASK(GLL) | MSG_MASK(RMC) | MSG_MASK(ZDA))) != 0)
	{
		p = TimeString;
		APPEND_CHAR(p, ',');
		p = PrintUint(p, NmeaInfo->Time.Hour, 2);
		p = PrintUint(p, NmeaInfo->Time.Minute, 2);
		p = PrintUint(p, NmeaInfo->Time.Second, 2);
		APPEND_CHAR(p, '.');
		p = PrintUint(p, NmeaInfo->Time.Millisecond, 3);
	}

	// if date string needed
	if ((NmeaMask & MSG_MASK(RMC)) != 0)
	{
		p = DateString;
		APPEND_CHAR(p, ',');
		p = PrintUint(p, NmeaInfo->Time.Day, 2);
		p = PrintUint(p, NmeaInfo->Time.Month, 2);
		p = PrintUint(p, (NmeaInfo->Time.Year % 100), 2);
	}

	// if dop value needed
	if ((NmeaMask & (MSG_MASK(GGA) | MSG_MASK(GSA))) != 0)
	{
		HdopString[0] = ',';
		if (Quality)
			 PrintFloat(HdopString + 1, NmeaInfo->DopArray[0], 3);
		else
			HdopString[1] = '\0';
		if ((NmeaMask & (MSG_MASK(GSA))) != 0) // only GSA need VDOP and PDOP
		{
			VdopString[0] = PdopString[0] = ',';
			if (Quality)
			{
				PrintFloat(VdopString + 1, NmeaInfo->DopArray[1], 3);
				PrintFloat(PdopString + 1, NmeaInfo->DopArray[2], 3);
			}
			else
				VdopString[1] = PdopString[1] = '\0';
		}
	}

	// if speed and course needed
	if ((NmeaMask & (MSG_MASK(RMC) | MSG_MASK(VTG))) != 0)
	{
		p = SpeedString;
		APPEND_CHAR(p, ',');
		if (Quality)
			PrintFloat(p, NmeaInfo->GroundSpeed.Speed * 3600 / 1852, 3);
		else
			APPEND_CHAR(p, '\0');
		p = CourseString;
		APPEND_CHAR(p, ',');
		if (Quality)
			PrintFloat(p, RAD2DEG(NmeaInfo->GroundSpeed.Course), 2);
		else
			APPEND_CHAR(p, '\0');
	}

	for (MsgType = 0; MsgType <= NMEA_MAX; MsgType ++)
	{
		if (!(NmeaMask & (1 << MsgType)))
			continue;

		switch (MsgType)
		{
		case NMEA_GGA:
			strcpy(StringStart, "$GNGGA"); StringStart[2] = TalkerChar[TalkerIndex];
			p = StringStart + 6;
			strcpy(p, TimeString); p += 11;
			strcat(p, LatLonString); p += strlen(p);
			APPEND_CHAR(p, ',');
			p = PrintUint(p, Quality, 1);
			APPEND_CHAR(p, ',');
			p = PrintUint(p, NmeaInfo->SatCount, 1);
			strcpy(p, HdopString); p += strlen(p);
			strcat(p, AltituteString); p += strlen(p);
			APPEND_CHAR(p, ',');    // no differetial information
			APPEND_CHAR(p, ',');    // no differetial information
			APPEND_CHAR(p, '\0');    // terminate the string
			StringStart = NmeaTerminate(StringStart);
			break;
		case NMEA_GLL:
			strcpy(StringStart, "$GNGLL"); StringStart[2] = TalkerChar[TalkerIndex];
			p = StringStart + 6;
			strcpy(p, LatLonString);
			strcat(p, TimeString); p += strlen(p);
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, Quality ? 'A' : 'V');    // status
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, 'A');    // mode
			APPEND_CHAR(p, '\0');    // terminate the string
			StringStart = NmeaTerminate(StringStart);
			break;
		case NMEA_GSA:
			for (PosSystem = 1; PosSystem <= PVT_MAX_SYSTEM_ID; PosSystem ++)
			{
				if (TalkerIndex > 0 && (PosSystem != TalkerIndex))  // only one system to do positioning
					continue;
				if ((SatInUse = NmeaInfo->SatInUse[PosSystem-1]) == 0)
					continue;
				strcpy(StringStart, "$GNGSA,A,3"); StringStart[2] = TalkerChar[TalkerIndex]; StringStart[9] = Quality ? '3' : '1';
				p = StringStart + 10;
				for (SatIndex = 0, SatCount = 0; SatCount < 12 && SatInUse; SatIndex ++)
				{
					if (!(SatInUse & (1ULL << SatIndex)))
						continue;
					APPEND_CHAR(p, ',');
					p = PrintUint(p, SvidBaseValue[PosSystem] + SatIndex, 2);
					SatInUse &= ~(1ULL << SatIndex);
					SatCount ++;
				}
				for (; SatCount < 12; SatCount ++)
					APPEND_CHAR(p, ',');
				strcpy(p, PdopString);
				strcat(p, HdopString);
				strcat(p, VdopString);
				p += strlen(p);
				APPEND_CHAR(p, ',');
				p = PrintUint(p, GnssSystemID[PosSystem], 1);
				StringStart = NmeaTerminate(StringStart);
			}
			break;
		case NMEA_GSV:
			for (PosSystem = 1; PosSystem <= PVT_MAX_SYSTEM_ID; PosSystem ++)
			{
				if (!(g_SystemConfig.PvtConfigFlags & (1 << (PosSystem - 1))))
					continue;
				StringStart = ComposeGsv(PosSystem, StringStart);
			}
			break;
		case NMEA_RMC:
			strcpy(StringStart, "$GNRMC"); StringStart[2] = TalkerChar[TalkerIndex];
			p = StringStart + 6;
			strcpy(p, TimeString); p += strlen(p);
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, 'A');    // status
			strcpy(p, LatLonString);
			strcat(p, SpeedString);
			strcat(p, CourseString);
			strcat(p, DateString);
			p += strlen(p);
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, ',');    // no magnetic variation
			APPEND_CHAR(p, 'E');
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, 'A');    // mode
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, Quality ? 'A' : 'V');    // status
			APPEND_CHAR(p, '\0');    // terminate the string
			StringStart = NmeaTerminate(StringStart);
			break;
		case NMEA_VTG:
			strcpy(StringStart, "$GNVTG"); StringStart[2] = TalkerChar[TalkerIndex];
			p = StringStart + 6;
			strcpy(p, CourseString); p += strlen(p);
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, 'T');
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, 'M');
			strcpy(p, SpeedString); p += strlen(p);
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, 'N');
			APPEND_CHAR(p, ',');
			if (Quality)
				p = PrintFloat(p, NmeaInfo->GroundSpeed.Speed * 3.6, 3);
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, 'K');
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, 'A');
			APPEND_CHAR(p, '\0');    // terminate the string
			StringStart = NmeaTerminate(StringStart);
			break;
		case NMEA_ZDA:
			strcpy(StringStart, "$GNZDA"); StringStart[2] = TalkerChar[TalkerIndex];
			p = StringStart + 6;
			strcat(p, TimeString); p = p + strlen(p);
			APPEND_CHAR(p, ',');
			p = PrintUint(p, NmeaInfo->Time.Day, 2);
			APPEND_CHAR(p, ',');
			p = PrintUint(p, NmeaInfo->Time.Month, 2);
			APPEND_CHAR(p, ',');
			p = PrintUint(p, NmeaInfo->Time.Year, 4);
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, ',');
			APPEND_CHAR(p, '\0');    // terminate the string
			StringStart = NmeaTerminate(StringStart);
			break;
		}
	}
	*StringStart = '\0';
	
	return StringStart - NmeaString;
}

char *NmeaTerminate(char *String)
{
    unsigned char CheckSum = 0;

    String ++;        // skip the first '$'
    while (*String)
    {
        if (*String == '*' || *String == '\r' || *String == '\n')
            break;
        CheckSum ^= *String ++;
    }

    *String ++= '*';
    *String ++= (CheckSum >> 4) > 9 ? 'A' - 10 + (CheckSum >> 4) : '0' + (CheckSum >> 4);
    *String ++= (CheckSum & 0xf) > 9 ? 'A' - 10 + (CheckSum & 0xf) : '0' + (CheckSum & 0xf);
    *String ++= '\r';
    *String ++= '\n';
    *String = 0;
    return String;
}

char *ComposeGsv(int System, char *StringStart)
{
    unsigned char SatPrn[MAX_GSV_SATS];
    unsigned char SatEl[MAX_GSV_SATS];
    unsigned char SatCN0[MAX_GSV_SATS];
    unsigned short SatAz[MAX_GSV_SATS];
	int i, j, TotalSatNumber, GsvSatNumber, GsvMsgNumber;
	char *p = StringStart;
    PSATELLITE_INFO SatInfo;

	if (System == 1)
	{
		SatInfo = g_GpsSatelliteInfo;
		TotalSatNumber = TOTAL_GPS_SAT_NUMBER;
	}
	else if (System == 2)
	{
		SatInfo = g_BdsSatelliteInfo;
		TotalSatNumber = TOTAL_BDS_SAT_NUMBER;
	}
	else if (System == 3)
	{
		SatInfo = g_GalileoSatelliteInfo;
		TotalSatNumber = TOTAL_GAL_SAT_NUMBER;
	}
	else
		return StringStart;

    GsvSatNumber = 0;
    for (i = 0; i < TotalSatNumber; i ++)
    {
		if ((SatInfo[i].SatInfoFlag & SAT_INFO_ELAZ_VALID) && SatInfo[i].el > 0.00872664626)	// in view satellites (elevation > 0.5 degree)
		{
			SatPrn[GsvSatNumber] = SvidBaseValue[System] + i;
			SatEl[GsvSatNumber] = (unsigned char)(SatInfo[i].el * 180. / PI + 0.5);
            SatAz[GsvSatNumber] = (unsigned short)(SatInfo[i].az * 180. / PI);
            SatCN0[GsvSatNumber] = (unsigned char)((SatInfo[i].CN0 + 50) / 100);
            GsvSatNumber ++;
		}
		else if (SatInfo[i].CN0 > 1000)	// in track but el/az not yet calculated
		{
			SatPrn[GsvSatNumber] = SvidBaseValue[System] + i;
			SatEl[GsvSatNumber] = (unsigned char)100;
            SatAz[GsvSatNumber] = (unsigned short)0;
            SatCN0[GsvSatNumber] = (unsigned char)(SatInfo[i].CN0 + 50) / 100;
            GsvSatNumber ++;
		}
	}
	GsvMsgNumber = (GsvSatNumber + 3) / 4;

	for (i = 1; i <= GsvMsgNumber; i ++)
	{
		strcpy(StringStart, "$GNGSV,"); StringStart[2] = TalkerChar[System];
		p = StringStart + 7;
		p = PrintUint(p, GsvMsgNumber, 1);
		APPEND_CHAR(p, ',');
		p = PrintUint(p, i, 1);
		for (j = (i - 1) * 4; j < i * 4 && j < GsvSatNumber; j ++)
		{
			APPEND_CHAR(p, ',');
			p = PrintUint(p, SatPrn[j], 2);
			APPEND_CHAR(p, ',');
			if (SatEl[j] <= 90)	p = PrintUint(p, SatEl[j], 2);
			APPEND_CHAR(p, ',');
			if (SatEl[j] <= 90)	p = PrintUint(p, SatAz[j], 3);
			APPEND_CHAR(p, ',');
			if (SatCN0[j] > 0)
				p = PrintUint(p, SatCN0[j], 2);
		}
		APPEND_CHAR(p, ',');
		p = PrintUint(p, 0, 1);
		StringStart = NmeaTerminate(StringStart);
	}

    return StringStart;
}
