#include "MissileDynamics.hpp"
#include <iostream>
#include <cmath>

StateDerivatives MissileDynamics::computeDerivatives(
    const Vector3D& position,
    const Vector3D& velocity,
    const Vector3D& orientation,
    const Vector3D& angularVelocity,
    double mass,
    const ForcesAndMoments& fm)
{
    StateDerivatives derivs;
    
    // 1. Turunan posisi = kecepatan
    derivs.dPosition_dt = positionDerivative(velocity);
    
    // 2. Turunan kecepatan = akselerasi (F/m)
    derivs.dVelocity_dt = velocityDerivative(fm.totalForce, mass);
    
    // 3. Turunan orientasi (Euler angle rates)
    derivs.dOrientation_dt = orientationDerivative(angularVelocity, orientation);
    
    // 4. Turunan angular velocity (angular acceleration)
    derivs.dAngularVelocity_dt = angularVelocityDerivative(
        fm.totalMoment, angularVelocity, mass, 1.0  // referenceLength = 1.0 default
    );
    
    // 5. Turunan massa
    derivs.dMass_dt = massDerivative(fm.thrustMagnitude);
    
    return derivs;
}