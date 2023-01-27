#ifndef UNWRAPPER_2PI_H_
#define UNWRAPPER_2PI_H_


/*  
*/

using namespace std;

class Unwrapper_2pi
{
public:

    Unwrapper_2pi(void);
    
    float operator()(float in) {
        return doStep(in);
    }
    
    virtual     ~Unwrapper_2pi();
    
    void        reset(void);
    float       doStep(float inc);

private:

    long turns;
    float last_value;

};
#endif