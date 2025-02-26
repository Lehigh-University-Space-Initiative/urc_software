/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki, double max_i )
    : pimpl(dt,max,min,Kp,Kd,Ki,max_i)
{
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl.calculate(setpoint,pv);
}
double PID::i_sum()
{
    return pimpl.i_sum();
}
PID::~PID()
{
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki, double max_i ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _max_i(max_i),
    _pre_error(0),
    _integral(0)
{
}

double PIDImpl::calculate( double setpoint, double pv )
{
    
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt * _Ki;
    //prevent windeup
    if( _integral > _max_i )
        _integral = _max_i;
    else if( _integral < -_max_i )
        _integral = -_max_i;

    double Iout =  _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

double PIDImpl::i_sum()
{
    return _integral;
}

PIDImpl::~PIDImpl()
{
}

#endif
