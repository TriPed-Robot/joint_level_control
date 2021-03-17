#include "joint_level_control/second_order_system.h"


SecondOrderSystem::SecondOrderSystem()
    :_input_z_1(0.0), _input_z_2(0.0), _output_z_1(0.0), _output_z_2(0.0)
{
}


SecondOrderSystem::~SecondOrderSystem()
{
}


double SecondOrderSystem::calculateOutput(double input)
{
    // Following transfer function:
    // G(s) = 2/(s^2 + 0.4s + 1)
    // is transformed with help of bilinear transformation to G[z].
    static const double sample_rate = 10.0; // [Hz]
    static const double sample_period = 1.0/sample_rate; // [s]
    static const double c_0 = 2.0/sample_period;
    static const double c_1 = 1.0/(c_0*c_0 + 0.4*c_0 + 1.0);
    static const double a_1 = (-2.0*c_0*c_0 + 2.0)*c_1;
    static const double a_2 = (c_0*c_0 - 0.4*c_0 + 1.0)*c_1;
    static const double b_0 = 1.0*c_1;
    static const double b_1 = 2.0*c_1;
    static const double b_2 = 1.0*c_1;
    
    double output = b_0*input + b_1*_input_z_1 + b_2*_input_z_2 - a_1*_output_z_1 - a_2*_output_z_2;
    
    _update(input, output);
    
    return output;
}


void SecondOrderSystem::_update(double input, double output)
{
    _input_z_2 = _input_z_1;
    _input_z_1 = input;
    _output_z_2 = _output_z_1;
    _output_z_1 = output;
}
