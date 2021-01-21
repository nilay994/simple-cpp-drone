#ifndef INCLUDE_CTRL_OPT_H_
#define INCLUDE_CTRL_OPT_H_


#include <math.h>
#include "gains.h"

void* flush_job(void* data);
void* calc_job(void* data);




class Optimizer
{
public:
	Optimizer();
	~Optimizer();

	void solveQP(void);
};


#endif /* INCLUDE_CTRL_OPT_H_ */
