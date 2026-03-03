#ifndef MISSILE_DYNAMICS_HPP
#define MISSILE_DYNAMICS_HPP

#include "Vector3D.hpp"
#include "StateDerivatives.hpp"

// Kelas ini berisi PERSAMAAN DIFERENSIAL yang mengatur gerak rudal
// Ini adalah implementasi dari Hukum Newton dan Euler
class MissileDynamics {
public:
    // ============ KONSTANTA FISIKA ============
    static constexpr double G0 = 9.80665;     // Gravitasi standar [m/s^2]
    static constexpr double RHO0 = 1.225;      // Density udara permukaan [kg/m^3]
    static constexpr double SPEED_OF_SOUND = 340.0; // [m/s]
    
    // ============ STRUKTUR DATA ============
    // Menampung semua gaya dan momen yang bekerja pada rudal
    struct ForcesAndMoments {
        Vector3D totalForce;      // Gaya total [N] dalam body frame
        Vector3D totalMoment;     // Momen total [Nm] dalam body frame
        double thrustMagnitude;   // Besar gaya dorong [N] (untuk mass flow)
        
        ForcesAndMoments() 
            : totalForce(0,0,0)
            , totalMoment(0,0,0)
            , thrustMagnitude(0.0) {}
    };
    
    // ============ METHOD UTAMA ============
    
    // Menghitung semua turunan berdasarkan state saat ini dan gaya-gaya
    static StateDerivatives computeDerivatives(
        const Vector3D& position,        // State saat ini
        const Vector3D& velocity,
        const Vector3D& orientation,
        const Vector3D& angularVelocity,
        double mass,
        const ForcesAndMoments& fm       // Gaya-gaya dari luar
    );
    
    // ============ PERSAMAAN DIFERENSIAL INDIVIDUAL ============
    
    // 1. Kinematika: posisi berubah karena kecepatan
    static Vector3D positionDerivative(const Vector3D& velocity) {
        return velocity;  // dx/dt = v
    }
    
    // 2. Dinamika translasi: kecepatan berubah karena gaya (Hukum Newton II)
    static Vector3D velocityDerivative(
        const Vector3D& totalForce,  // Dalam body frame
        double mass
    ) {
        if (mass < 1e-6) return Vector3D(0,0,0);
        return totalForce / mass;  // dv/dt = F/m
    }
    
    // 3. Kinematika rotasi: orientasi berubah karena kecepatan angular
    static Vector3D orientationDerivative(
        const Vector3D& angularVelocity,  // (p, q, r) dalam body frame
        const Vector3D& orientation       // (roll, pitch, yaw) saat ini
    ) {
        // Transformasi dari body rates ke Euler angle rates
        // Rumus: [dφ/dt, dθ/dt, dψ/dt] = T(φ,θ) * [p, q, r]
        double roll = orientation.getX();
        double pitch = orientation.getY();
        
        double sinRoll = std::sin(roll);
        double cosRoll = std::cos(roll);
        double tanPitch = std::tan(pitch);
        
        // Matriks transformasi (untuk urutan rotasi tertentu)
        double dRoll_dt = angularVelocity.getX() 
                        + sinRoll * tanPitch * angularVelocity.getY()
                        + cosRoll * tanPitch * angularVelocity.getZ();
        
        double dPitch_dt = cosRoll * angularVelocity.getY()
                         - sinRoll * angularVelocity.getZ();
        
        double dYaw_dt = (sinRoll / std::cos(pitch)) * angularVelocity.getY()
                       + (cosRoll / std::cos(pitch)) * angularVelocity.getZ();
        
        return Vector3D(dRoll_dt, dPitch_dt, dYaw_dt);
    }
    
    // 4. Dinamika rotasi: angular velocity berubah karena momen (Hukum Euler)
    static Vector3D angularVelocityDerivative(
        const Vector3D& totalMoment,      // Momen total dalam body frame
        const Vector3D& angularVelocity,  // (p, q, r) saat ini
        double mass,
        double referenceLength
    ) {
        // Sederhana: asumsi momen inersia skalar (bola homogen)
        // I = (2/5) * m * R^2 untuk bola, tapi kita pakai pendekatan silinder
        double Ixx = 0.5 * mass * referenceLength * referenceLength;  // Momen inersia utama
        double Iyy = Ixx;   // Sederhana, asumsi simetri
        double Izz = Ixx;
        
        if (Ixx < 1e-6) return Vector3D(0,0,0);
        
        // Untuk 6-DoF penuh, perlu tensor inersia dan suku gyroscopic
        // Sederhanakan: M = I * alpha
        return Vector3D(
            totalMoment.getX() / Ixx,
            totalMoment.getY() / Iyy,
            totalMoment.getZ() / Izz
        );
    }
    
    // 5. Massa berkurang karena bahan bakar terbakar
    static double massDerivative(
        double thrustMagnitude,
        double specificImpulse = 250.0  // Isp dalam detik
    ) {
        if (thrustMagnitude <= 0) return 0.0;
        // ṁ = T / (Isp * g0)
        return -thrustMagnitude / (specificImpulse * G0);
    }
};

#endif