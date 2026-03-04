#include "Missile.hpp"
#include "MissileDynamics.hpp"
#include "Environment.hpp"
#include "ControlSurface.hpp"
#include <iostream>
#include <cmath>
#include <stdexcept>

// Konstanta fisika
const double GRAVITY = 9.81;         // Percepatan gravitasi [m/s^2]
const double SPEED_OF_SOUND = 340.0; // Kecepatan suara [m/s]

// Batasan numerik sederhana untuk menjaga stabilitas integrasi
const double MAX_SPEED = 1500.0;          // Batas kecepatan efektif [m/s]
const double MIN_SPEED_FOR_AERO = 1.0;    // Ambang gaya aero mulai bekerja [m/s]

// Konstruktor
Missile::Missile() : Missile("Generic Missile") {}

Missile::Missile(const std::string& name) 
    : missileName(name)
    , position(0, 0, 0)
    , velocity(0, 0, 0)
    , orientation(0, 0, 0)
    , angularVelocity(0, 0, 0)
    , mass(100.0)           // Default 100 kg
    , fuelMass(40.0)        // Default 40% dari massa total
    , dryMass(60.0)         // Massa struktur
    , referenceArea(0.1)     // Default 0.1 m^2
    , referenceLength(1.0)   // Default 1 m
    , currentMass(100.0)
    , thrust(0.0)
    , currentTime(0.0)
{
    // Inisialisasi koefisien aerodinamika dengan nilai default
    aeroCoeffs.Cd0 = 0.1;
    aeroCoeffs.CLa = 2.0;
    aeroCoeffs.Cm0 = 0.0;
    aeroCoeffs.Cma = -0.5;
    
    // Inisialisasi propulsi default
    propulsion.burnTime = 5.0;
    propulsion.totalImpulse = 5000.0;
    propulsion.thrustPeak = 2000.0;
}

// Setter
void Missile::setInitialPosition(const Vector3D& pos) {
    position = pos;
}

void Missile::setInitialVelocity(const Vector3D& vel) {
    velocity = vel;
}

void Missile::setInitialOrientation(const Vector3D& ori) {
    orientation = ori;
}

void Missile::setMass(double m) {
    mass = m;
    currentMass = m;
    // Default: 40% bahan bakar jika belum di-set secara eksplisit
    fuelMass = m * 0.4;
    dryMass = m - fuelMass;
}

void Missile::setReferenceArea(double area) {
    if (area <= 0) {
        throw std::runtime_error("Reference area must be positive");
    }
    referenceArea = area;
}

void Missile::setReferenceLength(double length) {
    if (length <= 0) {
        throw std::runtime_error("Reference length must be positive");
    }
    referenceLength = length;
}

void Missile::setAerodynamicCoeffs(double cd0, double cla, double cm0, double cma) {
    aeroCoeffs.Cd0 = cd0;
    aeroCoeffs.CLa = cla;
    aeroCoeffs.Cm0 = cm0;
    aeroCoeffs.Cma = cma;
}

void Missile::setFuelMass(double fuel) {
    if (fuel < 0 || fuel > mass) {
        throw std::runtime_error("Fuel mass must be between 0 and total mass");
    }
    fuelMass = fuel;
    dryMass = mass - fuelMass;
}

void Missile::setPropulsion(double burnTime, double totalImpulse, double thrustPeak) {
    if (burnTime <= 0) {
        throw std::runtime_error("Burn time must be positive");
    }
    propulsion.burnTime = burnTime;
    propulsion.totalImpulse = totalImpulse;
    propulsion.thrustPeak = thrustPeak;
}

// Method utama update
void Missile::update(double dt) {
    if (dt <= 0) return;
    // Logging untuk debug
    static int stepCount = 0;
    stepCount++;

    try
    {
        // Hitung gaya-gaya yang bekerja
        Vector3D totalForce = computeGravity() 
                            + computeThrust() 
                            + computeAerodynamicForce();

        // Validasi gaya total (hindari NaN/inf yang akan merusak simulasi)
        auto isInvalidComponent = [](double v) {
            return std::isnan(v) || std::isinf(v);
        };

        if (isInvalidComponent(totalForce.getX()) ||
            isInvalidComponent(totalForce.getY()) ||
            isInvalidComponent(totalForce.getZ())) {
            std::cerr << "WARNING: Invalid force at t=" << currentTime << std::endl;
            throw std::runtime_error("Invalid total force (NaN/inf)");
        }
        
        // Hitung momen (torsi)
        Vector3D totalMoment = computeAerodynamicMoment();
        
        // Update kecepatan linear: F = m*a -> a = F/m
        Vector3D acceleration = totalForce / currentMass;
        velocity += acceleration * dt;
        
        // Update posisi
        position += velocity * dt;
        
        // Update kecepatan angular: M = I*alpha (sederhana, asumsi inersia konstan)
        // Untuk 6-DoF penuh, ini perlu tensor inersia
        double momentOfInertia = 0.5 * currentMass * referenceLength * referenceLength;
        Vector3D angularAcceleration = totalMoment / momentOfInertia;
        angularVelocity += angularAcceleration * dt;
        
        // Update orientasi (sederhana, untuk 6-DoF penuh perlu quaternion)
        orientation += angularVelocity * dt;
        
        // Update waktu
        currentTime += dt;
        
        // Update massa (hanya bahan bakar yang berkurang, struktur tetap)
        if (currentTime <= propulsion.burnTime && currentMass > dryMass) {
            double massFlowRate = fuelMass / propulsion.burnTime;
            currentMass -= massFlowRate * dt;
            if (currentMass < dryMass) currentMass = dryMass;
        }
        
        // Update thrust berdasarkan waktu
        if (currentTime <= propulsion.burnTime) {
            // Model thrust sederhana: parabola
            double t = currentTime / propulsion.burnTime;
            thrust = 4 * propulsion.thrustPeak * t * (1 - t);  // Bentuk parabola
        } else {
            thrust = 0.0;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in Missile::update at t=" << currentTime 
                  << ": " << e.what() << std::endl;
        throw;
    }
    
}

// Perhitungan gaya-gaya
Vector3D Missile::computeGravity() const {
    // Gunakan model gravitasi yang bergantung ketinggian
    double altitude = position.getY();  // Asumsi Y adalah ketinggian
    return Environment::getGravityVector(altitude, currentMass);
}

Vector3D Missile::computeThrust() const {
    if (thrust > 0) {
        // Arah thrust mengikuti orientasi rudal (Euler angles)
        double pitch = orientation.getY();  // Sudut elevasi
        double yaw   = orientation.getZ();  // Sudut azimuth
        double tx = thrust * std::cos(pitch) * std::cos(yaw);
        double ty = thrust * std::sin(pitch);
        double tz = thrust * std::cos(pitch) * std::sin(yaw);
        return Vector3D(tx, ty, tz);
    }
    return Vector3D(0, 0, 0);
}

Vector3D Missile::computeAerodynamicForce() const {
    // Hitung kecepatan relatif terhadap udara (asumsi udara diam)
    double v = velocity.magnitude();
    if (v < MIN_SPEED_FOR_AERO) {  // Threshold untuk menghindari masalah numerik
        return Vector3D(0, 0, 0);  // Tidak ada gaya aerodinamika jika terlalu lambat
    }

    // Batasi kecepatan efektif agar dynamic pressure tidak meledak secara numerik
    double vEff = std::min(v, MAX_SPEED);

    // Dapatkan density berdasarkan ketinggian aktual
    double altitude = position.getY();
    double rho = Environment::getDensity(altitude);
    double speedOfSound = Environment::getSpeedOfSound(altitude);
    
    // Dynamic pressure: q = 0.5 * rho * v^2
    double q = 0.5 * rho * vEff * vEff;

    // Hitung Mach number untuk aerodinamika (nanti untuk Level 5)
    double mach = v / speedOfSound;
        
    // Hitung angle of attack (sudut serang)
    // Sederhana: asumsi angle of attack dari komponen vertikal kecepatan
    double alpha = std::atan2(velocity.getY(), velocity.getX());
    
    // Gaya drag (berlawanan arah kecepatan)
    double cd = aeroCoeffs.Cd0;  // + kontribusi induced drag nanti
    double dragMagnitude = q * referenceArea * cd;

    // Gaya drag - pastikan velocity tidak nol
    Vector3D drag = -velocity.normalized() * dragMagnitude;
    
    // Gaya lift (tegak lurus kecepatan)
    double cl = aeroCoeffs.CLa * alpha;
    double liftMagnitude = q * referenceArea * cl;
    
    // Lift tegak lurus kecepatan, kita asumsikan ke arah Y global
    // Untuk akurat, lift tegak lurus kecepatan dalam bidang simetri rudal
    Vector3D lift(0, liftMagnitude, 0);
    
    return drag + lift;
}

Vector3D Missile::computeAerodynamicMoment() const {
    // Momen pitching dasar (tanpa kontrol)
    double v = velocity.magnitude();
    if (v < MIN_SPEED_FOR_AERO) {
        return Vector3D(0, 0, 0);
    }
    
    double vEff = std::min(v, MAX_SPEED);
    double altitude = position.getY();
    double rho = Environment::getDensity(altitude);
    double q = 0.5 * rho * vEff * vEff;
    
    double alpha = std::atan2(velocity.getY(), velocity.getX());
    
    // Momen dasar (tanpa kontrol)
    double cm = aeroCoeffs.Cm0 + aeroCoeffs.Cma * alpha;
    double basePitchingMoment = q * referenceArea * referenceLength * cm;
    
    return Vector3D(0, 0, basePitchingMoment);
}

// Utility methods
void Missile::reset() {
    position = Vector3D(0, 0, 0);
    velocity = Vector3D(0, 0, 0);
    orientation = Vector3D(0, 0, 0);
    angularVelocity = Vector3D(0, 0, 0);
    currentMass = mass;
    thrust = 0.0;
    currentTime = 0.0;
}

void Missile::printStatus() const {
    std::cout << "===== " << missileName << " Status =====" << std::endl;
    std::cout << "Time: " << currentTime << " s" << std::endl;
    std::cout << "Position: " << position << " m" << std::endl;
    std::cout << "Velocity: " << velocity << " m/s" << std::endl;
    std::cout << "Speed: " << velocity.magnitude() << " m/s (" << getMachNumber() << " Mach)" << std::endl;
    std::cout << "Orientation (r,p,y): " << orientation << " rad" << std::endl;
    std::cout << "Mass: " << currentMass << " kg" << std::endl;
    std::cout << "Thrust: " << thrust << " N" << std::endl;
    std::cout << "=========================" << std::endl;
}

double Missile::getMachNumber() const {
    double altitude = position.getY();
    double speedOfSound = Environment::getSpeedOfSound(altitude);
    return velocity.magnitude() / speedOfSound;
}

// ============ METHOD BARU UNTUK LEVEL 3 ============

MissileDynamics::ForcesAndMoments Missile::computeForcesAndMoments() const {
    MissileDynamics::ForcesAndMoments fm;
    
    // Hitung semua gaya (dalam body frame)
    Vector3D gravity = computeGravity();        // Ini masih dalam inertial frame!
    Vector3D thrustForce = computeThrust();      // Seharusnya dalam body frame
    Vector3D aeroForce = computeAerodynamicForce(); // Seharusnya dalam body frame
    Vector3D aeroMoment = computeAerodynamicMoment(); // Dalam body frame
    
    // TODO: Transformasi gravity ke body frame
    // Untuk sementara, asumsi body frame sejajar inertial frame
    fm.totalForce = gravity + thrustForce + aeroForce;
    fm.totalMoment = aeroMoment;
    fm.thrustMagnitude = thrust;  // Simpan untuk mass flow
    
    return fm;
}

void Missile::applyDerivatives(const StateDerivatives& derivs, double dt) {
    // Update semua state berdasarkan derivatives
    position += derivs.dPosition_dt * dt;
    velocity += derivs.dVelocity_dt * dt;
    orientation += derivs.dOrientation_dt * dt;
    angularVelocity += derivs.dAngularVelocity_dt * dt;
    currentMass += derivs.dMass_dt * dt;
    
    // Validasi: massa tidak boleh di bawah massa struktur
    if (currentMass < dryMass) currentMass = dryMass;
    
    // Update thrust terpisah (berdasarkan waktu)
    if (currentTime <= propulsion.burnTime) {
        double t = currentTime / propulsion.burnTime;
        thrust = 4 * propulsion.thrustPeak * t * (1 - t);
    } else {
        thrust = 0.0;
    }
}

// ============ LEVEL 5: IMPLEMENTASI METHOD KONTROL ===========

void Missile::addControlSurface(const ControlSurface& surface) {
    controlSurfaces.push_back(surface);
}

ControlSurface& Missile::getControlSurface(const std::string& name) {
    for (auto& surface : controlSurfaces) {
        if (surface.getName() == name) {
            return surface;
        }
    }
    throw std::runtime_error("Control surface not found: " + name);
}

Vector3D Missile::computeTotalMoment(double dynamicPressure) const {
    // Mulai dengan momen aerodinamika dasar
    Vector3D totalMoment = computeAerodynamicMoment();
    
    // Tambahkan kontribusi dari semua sirip kendali
    for (const auto& surface : controlSurfaces) {
        totalMoment = totalMoment + surface.computeMoment(
            dynamicPressure, referenceArea, referenceLength
        );
    }
    
    return totalMoment;
}

void Missile::setControlDeflections(double elevator, double rudder, double aileron) {
    for (auto& surface : controlSurfaces) {
        if (surface.getName() == "elevator") {
            surface.setDeflection(elevator);
        } else if (surface.getName() == "rudder") {
            surface.setDeflection(rudder);
        } else if (surface.getName() == "aileron") {
            surface.setDeflection(aileron);
        }
    }
}

double Missile::getElevatorDeflection() const {
    for (const auto& surface : controlSurfaces) {
        if (surface.getName() == "elevator") {
            return surface.getDeflection();
        }
    }
    return 0.0;
}

double Missile::getRudderDeflection() const {
    for (const auto& surface : controlSurfaces) {
        if (surface.getName() == "rudder") {
            return surface.getDeflection();
        }
    }
    return 0.0;
}

double Missile::getAileronDeflection() const {
    for (const auto& surface : controlSurfaces) {
        if (surface.getName() == "aileron") {
            return surface.getDeflection();
        }
    }
    return 0.0;
}