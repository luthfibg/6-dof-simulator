#include "Autopilot.hpp"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Autopilot::Autopilot()
    : rollIntegral(0.0)
    , pitchIntegral(0.0)
    , yawIntegral(0.0)
    , prevRollError(0.0)
    , prevPitchError(0.0)
    , prevYawError(0.0)
    , dt(0.01)
    , maxRollRate(1.0)    // rad/s
    , maxPitchRate(1.0)
    , maxYawRate(1.0)
{
    // Default gains (tuning awal)
    rollGain = {2.0, 0.1, 0.5};
    pitchGain = {3.0, 0.2, 0.8};
    yawGain = {2.0, 0.1, 0.5};
}

void Autopilot::setRollGains(double Kp, double Ki, double Kd) {
    rollGain = {Kp, Ki, Kd};
}

void Autopilot::setPitchGains(double Kp, double Ki, double Kd) {
    pitchGain = {Kp, Ki, Kd};
}

void Autopilot::setYawGains(double Kp, double Ki, double Kd) {
    yawGain = {Kp, Ki, Kd};
}

double Autopilot::getElevatorCommand(double pitchError, double pitchRate) {
    // Update integral
    pitchIntegral += pitchError * dt;
    
    // Anti-windup sederhana
    pitchIntegral = std::max(-1.0, std::min(1.0, pitchIntegral));
    
    // Derivative (gunakan pitch rate sebagai derivative approximation)
    double derivative = -pitchRate;  // Karena pitch rate = d(theta)/dt
    
    // PID output
    double command = pitchGain.Kp * pitchError + 
                     pitchGain.Ki * pitchIntegral + 
                     pitchGain.Kd * derivative;
    
    // Saturasi output
    return std::max(-0.35, std::min(0.35, command));  // ~20 derajat
}

double Autopilot::getRudderCommand(double yawError, double yawRate) {
    yawIntegral += yawError * dt;
    yawIntegral = std::max(-1.0, std::min(1.0, yawIntegral));
    
    double derivative = -yawRate;
    double command = yawGain.Kp * yawError + 
                     yawGain.Ki * yawIntegral + 
                     yawGain.Kd * derivative;
    
    return std::max(-0.35, std::min(0.35, command));
}

double Autopilot::getAileronCommand(double rollError, double rollRate) {
    rollIntegral += rollError * dt;
    rollIntegral = std::max(-1.0, std::min(1.0, rollIntegral));
    
    double derivative = -rollRate;
    double command = rollGain.Kp * rollError + 
                     rollGain.Ki * rollIntegral + 
                     rollGain.Kd * derivative;
    
    return std::max(-0.35, std::min(0.35, command));
}

std::vector<double> Autopilot::computeControlSurfaces(
    const Vector3D& targetOrientation,
    const Vector3D& currentOrientation,
    const Vector3D& currentAngularRate)
{
    // Hitung error (dengan wrap ke [-pi, pi])
    double rollError = targetOrientation.getX() - currentOrientation.getX();
    double pitchError = targetOrientation.getY() - currentOrientation.getY();
    double yawError = targetOrientation.getZ() - currentOrientation.getZ();
    
    // Normalisasi error sudut
    auto normalizeAngle = [](double angle) {
        while (angle > M_PI) angle -= 2*M_PI;
        while (angle < -M_PI) angle += 2*M_PI;
        return angle;
    };
    
    rollError = normalizeAngle(rollError);
    pitchError = normalizeAngle(pitchError);
    yawError = normalizeAngle(yawError);
    
    // Dapatkan defleksi dari masing-masing kontrol
    double elevator = getElevatorCommand(pitchError, currentAngularRate.getZ());
    double rudder = getRudderCommand(yawError, currentAngularRate.getY());
    double aileron = getAileronCommand(rollError, currentAngularRate.getX());
    
    return {elevator, rudder, aileron};
}

void Autopilot::reset() {
    rollIntegral = 0.0;
    pitchIntegral = 0.0;
    yawIntegral = 0.0;
    prevRollError = 0.0;
    prevPitchError = 0.0;
    prevYawError = 0.0;
}