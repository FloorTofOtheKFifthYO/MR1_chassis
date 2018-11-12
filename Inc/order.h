
#ifndef __order_H
#define __order_H
#ifdef __cplusplus
 extern "C" {
#endif


#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
#include "tim.h"
//#include "global.h"
//#include "flash.h"
#define MAXSIZE 100
#define MAXSENT 50
#define MAXNAME 50
#define MAXPARA 5

typedef struct sentence{
	void (*func)(float para[MAXPARA]);
	char name[MAXNAME];
	char notes[500];
}SENT;
extern SENT order[MAXSENT];
extern int buffercnt;
extern char buffer[MAXSIZE];
extern void initsent(SENT order[]);
extern void matching(char s[], SENT order[]);

#endif /*__ order_H */