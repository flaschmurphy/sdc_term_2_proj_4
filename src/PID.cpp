#include "PID.h"
#include <limits>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) 
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    p_error = 0;
    i_error = 0;
    d_error = 0;

    throttle = 0.0;

    best_err = std::numeric_limits<double>::max();

    is_initialized = true;

    previous_cte = 0;
}

void PID::UpdateError(double cte) 
{
    cte = std::abs(cte);

    // Update the proportional error
    p_error = Kp * cte;

    // Update the integral error
    i_error = Ki * (i_error + cte);

    // Update the differential error
    d_error = Kd * (std::abs(cte - previous_cte));
    previous_cte = cte;

}

double PID::TotalError() 
{
    return p_error + i_error + d_error;
}

void PID::SetParams(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}
