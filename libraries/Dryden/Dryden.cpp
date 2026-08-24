#include "Dryden.h"
#include <cmath>
#include <Mathp/mathp.h>

DrydenTurbulence::DrydenTurbulence()
{
    reset();
}

void DrydenTurbulence::init(const DrydenParams& params, double dt, unsigned int seed) {
    params_ = params;
    dt_ = dt;
    rng_.seed(seed);
    dist_ = std::normal_distribution<double>(0.0, 1.0);
}

void DrydenTurbulence::reset() {
    x_u_ = 0.0;
    x_v_ = {0.0, 0.0};
    x_w_ = {0.0, 0.0};
    x_p_ = 0.0;
}

void DrydenTurbulence::setVelocity(double velocity) {
    // Ensure velocity remains above a non-zero threshold to avoid division by zero
    params_.velocity = std::max(velocity, 0.1);
}

void DrydenTurbulence::setWingSpan(double wing_span) {
    params_.wing_span = std::max(wing_span, 0.01);
}

void DrydenTurbulence::setVehicleParameters(double velocity, double wing_span) {
    setVelocity(velocity);
    setWingSpan(wing_span);
}

DrydenOutput DrydenTurbulence::update() {
    // Continuous-time spectral density normalization scaling for Gaussian white noise
    double noise_scale = std::sqrt(1.0 / dt_);
    double n_u = dist_(rng_) * noise_scale;
    double n_v = dist_(rng_) * noise_scale;
    double n_w = dist_(rng_) * noise_scale;
    double n_p = dist_(rng_) * noise_scale;

    DrydenOutput out;

    // 1. Translational Gusts (Linear Velocities)
    
    // Longitudinal (u_g) - 1st order filter
    double tau_u = params_.L_u / params_.velocity;
    double K_u = params_.sigma_u * std::sqrt((2.0 * params_.L_u) / (M_PI * params_.velocity));
    double a_u = (2.0 * tau_u - dt_) / (2.0 * tau_u + dt_);
    double b_u = K_u * dt_ / (2.0 * tau_u + dt_);
    out.u = b_u * n_u + x_u_;
    x_u_ = out.u * a_u + b_u * n_u;

    // Lateral (v_g) & Vertical (w_g) - 2nd order filters
    out.v = update2ndOrder(params_.L_v, params_.sigma_v, n_v, x_v_);
    out.w = update2ndOrder(params_.L_w, params_.sigma_w, n_w, x_w_);

    // 2. Rotational Gusts (Angular Rates)
    double b = params_.wing_span;
    double V = params_.velocity;

    // Roll Rate (p_g): Driven by independent noise source n_p via MIL-F-8785C transfer function
    double sigma_p = (params_.sigma_w / b) * std::pow(0.8, 1.0 / 6.0) * 
                     std::pow((M_PI * b) / (4.0 * params_.L_w), 1.0 / 3.0);
    double tau_p = (4.0 * b) / (M_PI * V);
    double K_p = sigma_p * std::sqrt((0.8 * M_PI) / (2.0 * V));
    
    double a_p = (2.0 * tau_p - dt_) / (2.0 * tau_p + dt_);
    double b_p = K_p * dt_ / (2.0 * tau_p + dt_);
    out.p = b_p * n_p + x_p_;
    x_p_ = out.p * a_p + b_p * n_p;

    // Pitch Rate (q_g): Derived from spatial gradient of vertical turbulence (s*w_g / V)
    out.q = -(M_PI / (4.0 * b)) * out.w;

    // Yaw Rate (r_g): Derived from spatial gradient of lateral turbulence (s*v_g / V)
    out.r = (M_PI / (3.0 * b)) * out.v;

    return out;
}

double DrydenTurbulence::update2ndOrder(double L, double sigma, double noise, std::array<double, 2>& state) {
    double tau = L / params_.velocity;
    double K = sigma * std::sqrt(L / (M_PI * params_.velocity));

    double a0 = 1.0 / (tau * tau);
    double a1 = 2.0 / tau;
    double b0 = K * 1.0 / (tau * tau);
    double b1 = K * std::sqrt(3.0) / tau;

    double dx0 = state[1];
    double dx1 = -a0 * state[0] - a1 * state[1] + noise;

    state[0] += dx0 * dt_;
    state[1] += dx1 * dt_;

    return (b0 * state[0] + b1 * state[1]);
}