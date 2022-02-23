#include <stdio.h>
#include <math.h>
#include "DemoControl.h"
#include "../control.h"

int Demo_Get_ID_File(int Mode, int current_id)
{
	int file_id;
	printf("--%d",Mode);
	switch (Mode)
	{
	case HELLO: //0
		file_id = 0;
		break;
	case CRAWL: //1~3
		if (current_id == 2 || current_id == 3)
			file_id = current_id;
		else
			file_id = 1;
		break;
	case FALL_DL: //4~5
		if (current_id == 5)
			file_id = current_id;
		else
			file_id = 4;
		break;
	case ACTION: //6
		file_id = 6;
		break;
	default :
		file_id = current_id;
		break;
	}

	return file_id;
}

