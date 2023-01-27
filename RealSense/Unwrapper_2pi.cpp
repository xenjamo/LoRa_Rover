/*  
*/

#include "Unwrapper_2pi.h"
#define   pi 3.1415927f
using namespace std;

Unwrapper_2pi::Unwrapper_2pi(void)
{   
    last_value = 0.0;
    turns = 0;
}

Unwrapper_2pi::~Unwrapper_2pi() {}

void Unwrapper_2pi::reset(void)
{
    last_value = 0.0;
    turns = 0;
}

float Unwrapper_2pi::doStep(float in)
{
    float temp = in + 2.0f*pi*(float)turns;
    if((temp - last_value) > (float)pi){
        temp -= 2.0f*pi;
        turns--;
        }
    else if((temp - last_value) < -(float)pi){
        temp += 2.0f*pi;
        turns++;
        }
    last_value = temp;
    return (temp);
}