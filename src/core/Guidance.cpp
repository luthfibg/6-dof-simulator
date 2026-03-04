#include "Guidance.hpp"
#include <cmath>

Guidance::Guidance() : navigationGain(3.0) {}  // Gain 3 adalah standar

void Guidance::setTarget(const Vector3D& targetPos) {
    target.position = targetPos;
    target.velocity = Vector3D(0,0,0);
    target.isMoving = false;
}

void Guidance::setMovingTarget(const Vector3D& targetPos, const Vector3D& targetVel) {
    target.position = targetPos;
    target.velocity = targetVel;
    target.isMoving = true;
}

Guidance::GuidanceCommand Guidance::purePursuit(const Vector3D& missilePos,
                                                 const Vector3D& missileVel) const {
    GuidanceCommand cmd;
    
    // Vektor dari rudal ke target
    Vector3D lineOfSight = target.position - missilePos;
    
    // Arah yang diinginkan adalah langsung ke target
    Vector3D desiredDir = lineOfSight.normalized();
    
    // Hitung pitch dan yaw yang diperlukan
    double desiredPitch = std::atan2(desiredDir.getY(), 
                                      std::sqrt(desiredDir.getX()*desiredDir.getX() + 
                                                desiredDir.getZ()*desiredDir.getZ()));
    double desiredYaw = std::atan2(desiredDir.getZ(), desiredDir.getX());
    
    cmd.desiredOrientation = Vector3D(0, desiredPitch, desiredYaw);  // Roll = 0
    cmd.timeToGo = lineOfSight.magnitude() / missileVel.magnitude();
    
    return cmd;
}

Guidance::GuidanceCommand Guidance::proportionalNavigation(const Vector3D& missilePos,
                                                            const Vector3D& missileVel,
                                                            double timeStep) const {
    GuidanceCommand cmd;
    
    // Line of sight vector
    Vector3D los = target.position - missilePos;
    double R = los.magnitude();  // Range to target
    
    if (R < 1.0) {
        // Already at target
        cmd.desiredOrientation = Vector3D(0,0,0);
        cmd.timeToGo = 0;
        return cmd;
    }
    
    // Line of sight rate (turunan LOS)
    static Vector3D prevLos = los;
    static bool firstCall = true;
    
    Vector3D losRate;
    if (!firstCall) {
        losRate = (los - prevLos) / timeStep;
    } else {
        losRate = Vector3D(0,0,0);
        firstCall = false;
    }
    prevLos = los;
    
    // Proportional navigation: commanded acceleration = N * Vc * LOS rate
    double Vc = -missileVel.dot(los.normalized());  // Closing velocity
    
    // Akselerasi yang diperintahkan (dalam koordinat inersia)
    // losRate * (N * Vc) karena operator* hanya mendukung Vector3D * double
    Vector3D commandedAccel = losRate * (navigationGain * Vc);
    
    // Konversi akselerasi ke orientasi yang diinginkan
    // Sederhana: asumsi akselerasi tegak lurus kecepatan
    Vector3D desiredDir = (missileVel + commandedAccel * timeStep).normalized();
    
    double desiredPitch = std::atan2(desiredDir.getY(), 
                                      std::sqrt(desiredDir.getX()*desiredDir.getX() + 
                                                desiredDir.getZ()*desiredDir.getZ()));
    double desiredYaw = std::atan2(desiredDir.getZ(), desiredDir.getX());
    
    cmd.desiredOrientation = Vector3D(0, desiredPitch, desiredYaw);
    cmd.timeToGo = R / Vc;
    
    return cmd;
}