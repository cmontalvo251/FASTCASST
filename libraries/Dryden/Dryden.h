#ifndef DRYDEN_TURBULENCE_H
#define DRYDEN_TURBULENCE_H

#include <array>
#include <random>

// Parameters for MIL-F-8785C Dryden Model
struct DrydenParams {
    //Defaults but will be passed later
    double velocity = 15.0;     // Aircraft airspeed (m/s)
    double wing_span = 2.0;     // Wingspan, b (m)
    
    // Scale lengths (m)
    double L_u = 200.0;
    double L_v = 200.0;
    double L_w = 50.0;

    // Turbulence intensities (m/s)
    double sigma_u = 1.5;
    double sigma_v = 1.5;
    double sigma_w = 1.0;
};

// Struct holding full 6-DOF gust output
struct DrydenOutput {
    // Linear gust velocities (m/s)
    double u = 0.0; 
    double v = 0.0; 
    double w = 0.0; 

    // Angular gust rates (rad/s)
    double p = 0.0; // Roll rate
    double q = 0.0; // Pitch rate
    double r = 0.0; // Yaw rate
};

class DrydenTurbulence {
public:
    //Constructor
    DrydenTurbulence();

    void init(const DrydenParams& params, double dt, unsigned int seed);

    void reset();

    // Setters for dynamic runtime configuration
    void setVelocity(double velocity);
    void setWingSpan(double wing_span);
    void setVehicleParameters(double velocity, double wing_span);

    // Getters
    double getVelocity() const { return params_.velocity; }
    double getWingSpan() const { return params_.wing_span; }

    // Steps the filter forward by dt and returns full 6-DOF gust velocities and rates
    DrydenOutput update();

private:
    DrydenParams params_;
    double dt_;

    std::mt19937 rng_;
    std::normal_distribution<double> dist_;

    // Filter states for linear gusts
    double x_u_;
    std::array<double, 2> x_v_;
    std::array<double, 2> x_w_;

    // Filter state for roll gust rate
    double x_p_;

    double update2ndOrder(double L, double sigma, double noise, std::array<double, 2>& state);
};

#endif // DRYDEN_TURBULENCE_H