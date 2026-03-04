#ifndef CONTROL_SURFACE_HPP
#define CONTROL_SURFACE_HPP

#include "Vector3D.hpp"
#include <string>

// Kelas untuk merepresentasikan sirip kendali (fin)
class ControlSurface {
private:
    std::string name;           // Nama sirip (elevator, rudder, aileron)
    double deflection;           // Defleksi saat ini [rad]
    double maxDeflection;        // Defleksi maksimum [rad]
    double effectiveness;        // Efektivitas (Cl_delta, Cm_delta)
    Vector3D position;           // Posisi sirip relatif terhadap CG [m]
    double area;                 // Luas sirip [m^2]
    
public:
    ControlSurface(const std::string& name, 
                   double maxDef = 0.35,    // ~20 derajat
                   double eff = 0.1,
                   const Vector3D& pos = Vector3D(-1.0, 0, 0));
    
    // Setter
    void setDeflection(double def);  // Dengan saturasi
    void setMaxDeflection(double maxDef) { maxDeflection = maxDef; }
    
    // Getter
    double getDeflection() const { return deflection; }
    double getMaxDeflection() const { return maxDeflection; }
    std::string getName() const { return name; }
    
    // Menghitung kontribusi momen dari sirip ini
    // M = q * S_fin * Cm_delta * delta * lengan
    Vector3D computeMoment(double dynamicPressure, double referenceArea, double referenceLength) const;
    
    // Menghitung gaya tambahan dari sirip (jika signifikan)
    Vector3D computeForce(double dynamicPressure) const;
    
    // Reset ke posisi netral
    void neutralize() { deflection = 0.0; }
};

#endif