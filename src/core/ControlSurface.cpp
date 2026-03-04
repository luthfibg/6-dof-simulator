#include "ControlSurface.hpp"
#include <algorithm>
#include <cmath>

ControlSurface::ControlSurface(const std::string& name, 
                               double maxDef, 
                               double eff,
                               const Vector3D& pos)
    : name(name)
    , deflection(0.0)
    , maxDeflection(maxDef)
    , effectiveness(eff)
    , position(pos)
    , area(0.1)  // Default 0.1 m^2
{}

void ControlSurface::setDeflection(double def) {
    // Saturasi defleksi sesuai batas maksimum
    deflection = std::max(-maxDeflection, std::min(maxDeflection, def));
}

Vector3D ControlSurface::computeMoment(double dynamicPressure, 
                                        double referenceArea, 
                                        double referenceLength) const {
    // M = q * S_ref * Cm_delta * delta
    // Sederhana: asumsi kontribusi linear dengan defleksi
    double cmDelta = effectiveness * 2.0;  // Koefisien momen per radian
    double momentMagnitude = dynamicPressure * referenceArea * referenceLength * cmDelta * deflection;
    
    // Tentukan sumbu momen berdasarkan nama sirip
    // Konvensi: roll=X, pitch=Y, yaw=Z
    if (name == "elevator") {
        return Vector3D(0, -momentMagnitude, 0);   // Momen pitching (sumbu Y)
    } else if (name == "rudder") {
        return Vector3D(0, 0, momentMagnitude);    // Momen yaw (sumbu Z)
    } else if (name == "aileron") {
        return Vector3D(momentMagnitude, 0, 0);    // Momen roll (sumbu X)
    }
    
    return Vector3D(0, 0, 0);
}

Vector3D ControlSurface::computeForce(double dynamicPressure) const {
    // Gaya dari sirip biasanya kecil, diabaikan untuk simulasi awal
    return Vector3D(0, 0, 0);
}