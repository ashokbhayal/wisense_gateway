/******************************************************************************
* FILE PURPOSE: 
*******************************************************************************
*
* FILE NAME: typedefs.h
*
* DESCRIPTION:
*
*  Copyright (c) 2009 Texas Instruments Inc.
*  All Rights Reserved This program is the confidential and proprietary
*  product of Texas Instruments Inc.  Any Unauthorized use, reproduction or
*  transfer of this program is strictly prohibited.
*
* HISTORY:
*  
*
******************************************************************************/

#ifndef __TYPEDEFS_H__
#define __TYPEDEFS_H__

#define FALSE 0
#define TRUE  1

typedef int BOOL;
typedef unsigned long long UINT64;
typedef unsigned long long uint64;
typedef long long SINT64;
typedef unsigned int UINT32;
typedef unsigned int uint32;
typedef int SINT32;
typedef unsigned short UINT16;
typedef unsigned short uint16;
typedef short SINT16;
typedef unsigned char UINT8;
typedef char SINT8;


typedef enum 
{
  EVEN = 0,
  ODD = 1
} parity_t;

#define PARITY(a) ((a) % 2)

#define ALIGN16(len)    (((len) + 1) & ~1)


// type used in phy.h
typedef struct
{
  SINT16 imag;
  SINT16 real;
}cplx16vcu, ComplexShortVCU;


#endif
