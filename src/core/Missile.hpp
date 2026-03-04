#ifndef MISSILE_HPP
#define MISSILE_HPP

#include "Vector3D.hpp"
#include "Environment.hpp" 
#include "StateDerivatives.hpp"
#include "MissileDynamics.hpp"
#include <vector>
#include <string>

class Missile {
private:
    // ============ STATE VARIABLES (berubah setiap waktu) ============
    Vector3D position;      // Posisi [meter]
    Vector3D velocity;      // Kecepatan [m/s]
    Vector3D orientation;   // Orientasi (roll, pitch, yaw) [radian]
    Vector3D angularVelocity; // Kecepatan sudut (p, q, r) [rad/s]
    
    // ============ PROPERTIES KONSTAN ============
    double mass;            // Massa total [kg]
    double fuelMass;        // Massa bahan bakar [kg]
    double dryMass;         // Massa struktur tanpa bahan bakar [kg]
    double referenceArea;   // Luas penampang referensi [m^2]
    double referenceLength; // Panjang referensi (untuk momen) [m]
    
    // ============ PROPERTIES VARIABEL ============
    double currentMass;     // Massa saat ini (berkurang karena bahan bakar) [kg]
    double thrust;          // Gaya dorong saat ini [N]
    
    // ============ DATA AERODINAMIKA ============
    // Koefisien aerodinamika (sederhana, nanti bisa diperluas)
    struct AerodynamicCoeffs {
        double Cd0;     // Koefisien drag dasar (zero-lift drag)
        double CLa;     // Koefisien lift per sudut serang
        double Cm0;     // Koefisien momen pitching dasar
        double Cma;     // Koefisien momen pitching per sudut serang
    } aeroCoeffs;
    
    // ============ DATA PROPULSI ============
    struct PropulsionData {
        double burnTime;        // Lama pembakaran [s]
        double totalImpulse;    // Total impuls [N·s]
        double thrustPeak;      // Gaya dorong puncak [N]
        std::vector<double> thrustCurve; // Profil gaya dorong terhadap waktu
    } propulsion;
    
    // ============ PENDUKUNG SIMULASI ============
    double currentTime;     // Waktu simulasi [s]
    std::string missileName; // Nama/tipe rudal
    
public:
    // Konstruktor
    Missile();
    Missile(const std::string& name);
    
    // ============ SETTER UNTUK PROPERTI ============
    void setInitialPosition(const Vector3D& pos);
    void setInitialVelocity(const Vector3D& vel);
    void setInitialOrientation(const Vector3D& ori);
    void setMass(double m);
    void setReferenceArea(double area);
    void setReferenceLength(double length);
    void setAerodynamicCoeffs(double cd0, double cla, double cm0, double cma);
    void setFuelMass(double fuel);
    void setPropulsion(double burnTime, double totalImpulse, double thrustPeak);
    
    // ============ GETTER UNTUK STATE ============
    Vector3D getPosition() const { return position; }
    Vector3D getVelocity() const { return velocity; }
    Vector3D getOrientation() const { return orientation; }
    Vector3D getAngularVelocity() const { return angularVelocity; }
    double getCurrentMass() const { return currentMass; }
    double getCurrentTime() const { return currentTime; }
    double getCurrentThrust() const { return thrust; }
    
    // ============ METHOD UTAMA SIMULASI ============
    void update(double dt);  // Update state berdasarkan gaya-gaya yang bekerja
    
    // ============ METHOD PENDUKUNG ============
    void reset();  // Reset ke kondisi awal
    
    // ============ PERHITUNGAN GAYA ============
    Vector3D computeGravity() const;           // Gaya gravitasi
    Vector3D computeThrust() const;            // Gaya dorong
    Vector3D computeAerodynamicForce() const;  // Gaya aerodinamika (drag + lift)
    Vector3D computeAerodynamicMoment() const; // Momen aerodinamika
    
    // ============ UTILITAS ============
    void printStatus() const;  // Tampilkan status rudal
    double getMachNumber() const;  // Hitung bilangan Mach (asumsi kecepatan suara 340 m/s)

    // ============ METHOD BARU UNTUK LEVEL 3 ============
    
    // Mengumpulkan semua gaya dan momen yang bekerja
    MissileDynamics::ForcesAndMoments computeForcesAndMoments() const;
    
    // Meng-update state berdasarkan derivatives
    void applyDerivatives(const StateDerivatives& derivs, double dt);
};

#endif // MISSILE_HPP