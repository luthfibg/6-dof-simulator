#ifndef GUIDANCE_HPP
#define GUIDANCE_HPP

#include "Vector3D.hpp"
#include <vector>

// Berbagai algoritma guidance
class Guidance {
public:
    // Target
    struct Target {
        Vector3D position;      // Posisi target [m]
        Vector3D velocity;      // Kecepatan target [m/s] (untuk target bergerak)
        bool isMoving;          // Apakah target bergerak?
        
        Target() : position(0,0,0), velocity(0,0,0), isMoving(false) {}
        Target(const Vector3D& pos) : position(pos), velocity(0,0,0), isMoving(false) {}
    };
    
    // Output guidance: orientasi yang diinginkan
    struct GuidanceCommand {
        Vector3D desiredOrientation;  // (roll, pitch, yaw) yang diinginkan
        double timeToGo;               // Estimasi waktu menuju target [s]
    };
    
private:
    Target target;
    double navigationGain;  // N untuk proportional navigation
    
public:
    Guidance();
    
    // Set target
    void setTarget(const Vector3D& targetPos);
    void setMovingTarget(const Vector3D& targetPos, const Vector3D& targetVel);
    
    // Algoritma guidance
    
    // 1. Pure Pursuit (arahkan langsung ke target)
    GuidanceCommand purePursuit(const Vector3D& missilePos, 
                                 const Vector3D& missileVel) const;
    
    // 2. Proportional Navigation (PN) - standar untuk homing
    GuidanceCommand proportionalNavigation(const Vector3D& missilePos,
                                            const Vector3D& missileVel,
                                            double timeStep) const;
    
    // 3. Waypoint guidance (terbang melalui titik-titik)
    GuidanceCommand waypointGuidance(const Vector3D& missilePos,
                                      const std::vector<Vector3D>& waypoints,
                                      int& currentWaypoint) const;
    
    // Getter
    Target getTarget() const { return target; }
};

#endif