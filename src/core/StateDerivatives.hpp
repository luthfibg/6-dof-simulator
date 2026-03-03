#ifndef STATE_DERIVATIVES_HPP
#define STATE_DERIVATIVES_HPP

#include "Vector3D.hpp"
#include <cmath>
#include <iostream>

// Struct untuk menyimpan turunan/derivatives dari semua state variables
// Ini adalah "dx/dt" untuk setiap state
struct StateDerivatives {
    // Turunan posisi = velocity (kinematika)
    Vector3D dPosition_dt;      // d(x,y,z)/dt = (vx, vy, vz)
    
    // Turunan velocity = acceleration (hukum Newton)
    Vector3D dVelocity_dt;      // d(vx,vy,vz)/dt = (ax, ay, az) = F/m
    
    // Turunan orientasi = fungsi angular velocity
    Vector3D dOrientation_dt;   // d(roll,pitch,yaw)/dt = f(p,q,r)
    
    // Turunan angular velocity = angular acceleration (hukum Euler)
    Vector3D dAngularVelocity_dt; // d(p,q,r)/dt = (alpha_x, alpha_y, alpha_z) = M/I
    
    // Turunan massa = -mass flow rate
    double dMass_dt;            // dm/dt = -ṁ
    
    // Validasi (cek NaN/inf)
    bool isValid() const {
        auto checkVector = [](const Vector3D& v) {
            return !std::isnan(v.getX()) && !std::isinf(v.getX()) &&
                   !std::isnan(v.getY()) && !std::isinf(v.getY()) &&
                   !std::isnan(v.getZ()) && !std::isinf(v.getZ());
        };
        
        return checkVector(dPosition_dt) &&
               checkVector(dVelocity_dt) &&
               checkVector(dOrientation_dt) &&
               checkVector(dAngularVelocity_dt) &&
               !std::isnan(dMass_dt) && !std::isinf(dMass_dt);
    }
    
    // Untuk debugging
    void print() const {
        std::cout << "Derivatives:\n"
                  << "  dPos/dt: " << dPosition_dt << "\n"
                  << "  dVel/dt: " << dVelocity_dt << "\n"
                  << "  dOri/dt: " << dOrientation_dt << "\n"
                  << "  dAngVel/dt: " << dAngularVelocity_dt << "\n"
                  << "  dm/dt: " << dMass_dt << std::endl;
    }
};

#endif