#ifndef VECTOR3D_HPP
#define VECTOR3D_HPP

#include <cmath>
#include <iostream>

class Vector3D {
private:
    double x, y, z;  // Komponen vektor

public:
    // Konstruktor
    Vector3D(double x = 0.0, double y = 0.0, double z = 0.0);
    
    // Konstruktor salin (default sudah cukup, tapi kita tulis untuk kejelasan)
    Vector3D(const Vector3D& other) = default;
    
    // Destruktor
    ~Vector3D() = default;

    // Getter
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }
    
    // Setter
    void setX(double newX) { x = newX; }
    void setY(double newY) { y = newY; }
    void setZ(double newZ) { z = newZ; }
    
    // Operator assignment
    Vector3D& operator=(const Vector3D& other) = default;

    // Operasi dasar vektor
    Vector3D operator+(const Vector3D& other) const;
    Vector3D operator-(const Vector3D& other) const;
    Vector3D operator-() const;                     // Unary minus
    Vector3D operator*(double scalar) const;
    Vector3D operator/(double scalar) const;
    
    // Operator assignment dengan operasi
    Vector3D& operator+=(const Vector3D& other);
    Vector3D& operator-=(const Vector3D& other);
    Vector3D& operator*=(double scalar);
    Vector3D& operator/=(double scalar);
    
    // Operasi vektor khusus
    double dot(const Vector3D& other) const;      // Produk dot
    Vector3D cross(const Vector3D& other) const;  // Produk cross
    double magnitude() const;                      // Besar vektor
    double magnitudeSquared() const;               // Kuadrat besar vektor (lebih efisien)
    Vector3D normalized() const;                    // Vektor satuan
    void normalize();                               // Normalisasi in-place
    
    // Operasi utilitas
    double distanceTo(const Vector3D& other) const; // Jarak ke vektor lain
    Vector3D reflect(const Vector3D& normal) const; // Refleksi terhadap normal
    
    // Operator perbandingan
    bool operator==(const Vector3D& other) const;
    bool operator!=(const Vector3D& other) const;
    
    // Output stream
    friend std::ostream& operator<<(std::ostream& os, const Vector3D& vec);
};

#endif // VECTOR3D_HPP