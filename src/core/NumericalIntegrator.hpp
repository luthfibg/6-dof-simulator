#ifndef NUMERICAL_INTEGRATOR_HPP
#define NUMERICAL_INTEGRATOR_HPP

#include "StateDerivatives.hpp"
#include <functional>
#include <iostream>

// Kelas untuk integrasi numerik persamaan diferensial
class NumericalIntegrator {
public:
    // Tipe fungsi yang menghitung derivatives
    using DerivativeFunction = std::function<StateDerivatives(
        const Vector3D& pos,
        const Vector3D& vel,
        const Vector3D& ori,
        const Vector3D& angVel,
        double mass,
        double time
    )>;
    
    // ============ INTEGRASI RUNGE-KUTTA ORDE 4 ============
    
    // RK4 untuk satu langkah
    static void rk4Step(
        Vector3D& pos,           // state akan di-update
        Vector3D& vel,
        Vector3D& ori,
        Vector3D& angVel,
        double& mass,
        double& time,
        double dt,
        DerivativeFunction computeDerivs
    ) {
        // Simpan state awal
        Vector3D pos0 = pos;
        Vector3D vel0 = vel;
        Vector3D ori0 = ori;
        Vector3D angVel0 = angVel;
        double mass0 = mass;
        double t0 = time;
        
        // k1
        StateDerivatives k1 = computeDerivs(pos0, vel0, ori0, angVel0, mass0, t0);
        
        // k2 (state setengah langkah)
        Vector3D pos2 = pos0 + k1.dPosition_dt * (dt/2);
        Vector3D vel2 = vel0 + k1.dVelocity_dt * (dt/2);
        Vector3D ori2 = ori0 + k1.dOrientation_dt * (dt/2);
        Vector3D angVel2 = angVel0 + k1.dAngularVelocity_dt * (dt/2);
        double mass2 = mass0 + k1.dMass_dt * (dt/2);
        
        StateDerivatives k2 = computeDerivs(pos2, vel2, ori2, angVel2, mass2, t0 + dt/2);
        
        // k3
        Vector3D pos3 = pos0 + k2.dPosition_dt * (dt/2);
        Vector3D vel3 = vel0 + k2.dVelocity_dt * (dt/2);
        Vector3D ori3 = ori0 + k2.dOrientation_dt * (dt/2);
        Vector3D angVel3 = angVel0 + k2.dAngularVelocity_dt * (dt/2);
        double mass3 = mass0 + k2.dMass_dt * (dt/2);
        
        StateDerivatives k3 = computeDerivs(pos3, vel3, ori3, angVel3, mass3, t0 + dt/2);
        
        // k4
        Vector3D pos4 = pos0 + k3.dPosition_dt * dt;
        Vector3D vel4 = vel0 + k3.dVelocity_dt * dt;
        Vector3D ori4 = ori0 + k3.dOrientation_dt * dt;
        Vector3D angVel4 = angVel0 + k3.dAngularVelocity_dt * dt;
        double mass4 = mass0 + k3.dMass_dt * dt;
        
        StateDerivatives k4 = computeDerivs(pos4, vel4, ori4, angVel4, mass4, t0 + dt);
        
        // Kombinasi RK4: (k1 + 2k2 + 2k3 + k4) / 6
        StateDerivatives k;
        k.dPosition_dt = (k1.dPosition_dt + k2.dPosition_dt*2 + k3.dPosition_dt*2 + k4.dPosition_dt) / 6.0;
        k.dVelocity_dt = (k1.dVelocity_dt + k2.dVelocity_dt*2 + k3.dVelocity_dt*2 + k4.dVelocity_dt) / 6.0;
        k.dOrientation_dt = (k1.dOrientation_dt + k2.dOrientation_dt*2 + k3.dOrientation_dt*2 + k4.dOrientation_dt) / 6.0;
        k.dAngularVelocity_dt = (k1.dAngularVelocity_dt + k2.dAngularVelocity_dt*2 + k3.dAngularVelocity_dt*2 + k4.dAngularVelocity_dt) / 6.0;
        k.dMass_dt = (k1.dMass_dt + k2.dMass_dt*2 + k3.dMass_dt*2 + k4.dMass_dt) / 6.0;
        
        // Validasi
        if (!k.isValid()) {
            std::cerr << "Warning: Invalid RK4 combination, falling back to Euler" << std::endl;
            // Fallback ke Euler
            pos = pos0 + k1.dPosition_dt * dt;
            vel = vel0 + k1.dVelocity_dt * dt;
            ori = ori0 + k1.dOrientation_dt * dt;
            angVel = angVel0 + k1.dAngularVelocity_dt * dt;
            mass = mass0 + k1.dMass_dt * dt;
        } else {
            // Aplikasikan hasil RK4
            pos = pos0 + k.dPosition_dt * dt;
            vel = vel0 + k.dVelocity_dt * dt;
            ori = ori0 + k.dOrientation_dt * dt;
            angVel = angVel0 + k.dAngularVelocity_dt * dt;
            mass = mass0 + k.dMass_dt * dt;
        }
        
        time = t0 + dt;
        
        // Validasi akhir
        if (mass < 0) mass = 0;
    }
    
    // Euler sederhana (untuk fallback)
    static void eulerStep(
        Vector3D& pos,
        Vector3D& vel,
        Vector3D& ori,
        Vector3D& angVel,
        double& mass,
        double& time,
        double dt,
        DerivativeFunction computeDerivs
    ) {
        StateDerivatives k = computeDerivs(pos, vel, ori, angVel, mass, time);
        
        pos = pos + k.dPosition_dt * dt;
        vel = vel + k.dVelocity_dt * dt;
        ori = ori + k.dOrientation_dt * dt;
        angVel = angVel + k.dAngularVelocity_dt * dt;
        mass = mass + k.dMass_dt * dt;
        time = time + dt;
        
        if (mass < 0) mass = 0;
    }
};

#endif