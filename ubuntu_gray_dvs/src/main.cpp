#include "pipeline.h"

int main(int argc,char** argv)
{
	Pipeline po;
	bool vpOrNot = true;
	bool impOrNot = true;//false
	po.image_sequence(vpOrNot,impOrNot);
    return 0;
}

