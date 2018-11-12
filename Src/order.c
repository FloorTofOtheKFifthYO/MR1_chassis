#include "order.h"
int ordernum = 0;//命令的条数（不包括help） 
char buffer[MAXSIZE];
SENT order[MAXSENT];
int buffercnt;
/*********************用户函数区**********************/
void func1(float para[MAXPARA])
{
	printf("func1 parameter = %f %f %f %f %f\r\n",para[0],para[1],para[2],para[3],para[4]);
}

/*********************用户函数区结束**********************/


void addsent(char *name, void (*func)(float para[MAXPARA]), char notes[500],SENT order[]) 
{
	order[ordernum].func = func;
	strcpy(order[ordernum].name,name);
	strcpy(order[ordernum].notes ,notes);	
	ordernum++;
}
void initsent(SENT order[])
{
	addsent("func1",func1,"这是func1的用法",order);
}
void matchsent(char name[], float p[MAXPARA], SENT order[])
{
	int i;
	for(i = 0; i < ordernum; i++)
	{
		if(strcmp(name,order[i].name) == 0)
		{
			(*order[i].func)(p);
			break;
		}
	}
	if(i >= ordernum)
		printf("no matching\r\n");
}
void matchhelp(char name[], SENT order[])
{
	int i;
	for(i = 0; i < ordernum; i++)
	{
		if(strcmp(name,order[i].name) == 0)
		{
			printf("%s\r\n",order[i].notes);
			break;
		}
	}
	if(i >= ordernum)
		printf("no matching\r\n");
}
void matching(char s[], SENT order[]) 
{
	//char s[MAXSIZE];
		float para[MAXSENT] = {0};
		//gets(s);
		char name[MAXNAME] = "";
		int i,j;
		for(i = 0; i < MAXSIZE; i++)
		{
			if(s[i] == ' ' || s[i] == '\0') break;
			name[i] = s[i];
		}
		if(strcmp(name,"help")==0)
		{
			i++;
			char thename[MAXNAME] = "";
			for(j = 0; j < MAXNAME && s[i]!='\0'&&s[i] != ' '; j++,i++)
			{
				thename[j] = s[i];	
			}
			matchhelp(thename,order);
		}
		else  
		{
			for(j = 0; j < MAXPARA && s[i]!='\0'; j++)
			{
				i++;
				int flag = 0;
				if(s[i] == '-')
				{
					flag = 1;
					i++;
				}
				for(;s[i] <= '9'&&s[i] >= '0'; i++)
				{
					para[j] =para[j]*10 + (float)(s[i]-48);
				}
				if(s[i] == '.')
				{
					int c;
					i++;
					for(c = 1; s[i] <= '9'&&s[i] >= '0'; c++)
					{
						para[j] = para[j] + (float)(s[i] - 48) * pow(0.1,c);
						i++;
					}
				}
                                if(flag == 1) para[j] = -para[j];
			}
			matchsent(name,para,order);
		} 
}
