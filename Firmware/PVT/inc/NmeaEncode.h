//----------------------------------------------------------------------
// NmeaEncode.h:
//   Definitions for NMEA0183 encode
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------
#ifndef __NMEA_ENCODE_H__
#define __NMEA_ENCODE_H__

// definition of NMEA type mask
#define	NMEA_GGA		0
#define	NMEA_GSA		1
#define	NMEA_GSV		2
#define	NMEA_GLL		3
#define	NMEA_RMC		4
#define	NMEA_VTG		5
#define	NMEA_ZDA		6
#define NMEA_MAX		7

#define	MSG_MASK(message) (1 << (NMEA_##message))

int NMEAEncode(PNMEA_INFO NmeaInfo, unsigned int NmeaMask, char *NmeaString);
extern int NmeaOutputInterval[NMEA_MAX];

#endif //__NMEA_ENCODE_H__
