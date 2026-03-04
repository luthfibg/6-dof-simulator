#ifndef AUTOPILOT_HPP
#define AUTOPILOT_HPP

#include "Vector3D.hpp"
#include "ControlSurface.hpp"
#include <vector>
#include <memory>

// Autopilot untuk menjaga attitude (sikap) rudal
// Menggunakan kontrol PID sederhana
class Autopilot {
private:
    // Gain PID untuk setiap sumbu
    struct PIDGains {
        double Kp;  // Proportional
        double Ki;  // Integral
        double Kd;  // Derivative
    };
    
    PIDGains rollGain;
    PIDGains pitchGain;
    PIDGains yawGain;
    
    // Integral terms (akumulasi error)
    double rollIntegral;
    double pitchIntegral;
    double yawIntegral;
    
    // Previous errors (untuk derivative)
    double prevRollError;
    double prevPitchError;
    double prevYawError;
    
    // Time step
    double dt;
    
    // Command limits
    double maxRollRate;
    double maxPitchRate;
    double maxYawRate;
    
public:
    Autopilot();
    
    // Set gain PID
    void setRollGains(double Kp, double Ki, double Kd);
    void setPitchGains(double Kp, double Ki, double Kd);
    void setYawGains(double Kp, double Ki, double Kd);
    
    // Set time step
    void setTimeStep(double timeStep) { dt = timeStep; }
    
    // Menghitung defleksi sirip berdasarkan error attitude
    // targetOrientation: orientasi yang diinginkan (roll, pitch, yaw)
    // currentOrientation: orientasi saat ini
    // currentAngularRate: kecepatan angular saat ini (opsional)
    std::vector<double> computeControlSurfaces(
        const Vector3D& targetOrientation,
        const Vector3D& currentOrientation,
        const Vector3D& currentAngularRate
    );
    
    // Reset integral terms
    void reset();
    
    // Mendapatkan rekomendasi defleksi untuk masing-masing sirip
    double getElevatorCommand(double pitchError, double pitchRate);
    double getRudderCommand(double yawError, double yawRate);
    double getAileronCommand(double rollError, double rollRate);
};

#endif