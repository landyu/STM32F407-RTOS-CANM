#include "parking.h"
#include "Printf.h"
u8 CarInf[64];
vu8 DevExisting = 0;
u8  DevInfo[64][64];


void  carin(u8 lotnum)
{
	printf("+++++++++++++++++++++++++++++++++carin  lotnum= %d\r\n",lotnum);
	if(lotnum >= 1 && lotnum < 64 )
	CarInf[lotnum] = 0xff;
}

void  carout(u8 lotnum)
{
	printf("+++++++++++++++++++++++++++++carout  lotnum= %d\r\n",lotnum);
	if(lotnum >= 1 && lotnum < 64 )
	CarInf[lotnum] = 0x0f;
}

void  nolot(u8 lotnum)
{
	//DBG(MyPrintf("nolot  lotnum= %d\r\n",(U16)lotnum);)
	//printf("lotnum = %d \r\n", lotnum);
	if(lotnum >= 1 && lotnum < 64 )
	memset(DevInfo[lotnum], 0x0, 64);
}

