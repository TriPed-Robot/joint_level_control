#ifndef SYSTEM_SIMULATION_H
#define SYSTEM_SIMULATION_H


class SecondOrderSystem
{
public:
    SecondOrderSystem();
    ~SecondOrderSystem();
    
    double calculateOutput(double input);

private:
    void _update(double input, double output);
    
private:
    double _input_z_1;
    double _input_z_2;
    double _output_z_1;
    double _output_z_2;
};


#endif
