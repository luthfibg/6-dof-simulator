#include "Vector3D.hpp"
#include <cmath>
#include <stdexcept>

// Konstruktor
Vector3D::Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}

// Operator aritmatika
Vector3D Vector3D::operator+(const Vector3D& other) const {
    return Vector3D(x + other.x, y + other.y, z + other.z);
}

Vector3D Vector3D::operator-(const Vector3D& other) const {
    return Vector3D(x - other.x, y - other.y, z - other.z);
}

Vector3D Vector3D::operator-() const {
    return Vector3D(-x, -y, -z);
}

Vector3D Vector3D::operator*(double scalar) const {
    return Vector3D(x * scalar, y * scalar, z * scalar);
}

Vector3D Vector3D::operator/(double scalar) const {
    if (std::abs(scalar) < 1e-10) {  // Gunakan toleransi, bukan == 0
        return Vector3D(0, 0, 0);     // Kembalikan vektor nol jika pembagi terlalu kecil
    }
    return Vector3D(x / scalar, y / scalar, z / scalar);
}

Vector3D& Vector3D::operator/=(double scalar) {
    if (std::abs(scalar) < 1e-10) {
        x = 0; y = 0; z = 0;  // Set ke nol jika pembagi terlalu kecil
        return *this;
    }
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}

// Operator assignment
Vector3D& Vector3D::operator+=(const Vector3D& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vector3D& Vector3D::operator-=(const Vector3D& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vector3D& Vector3D::operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

// Operasi vektor
double Vector3D::dot(const Vector3D& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vector3D Vector3D::cross(const Vector3D& other) const {
    return Vector3D(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

double Vector3D::magnitude() const {
    return std::sqrt(magnitudeSquared());
}

double Vector3D::magnitudeSquared() const {
    return x*x + y*y + z*z;
}

Vector3D Vector3D::normalized() const {
    double mag = magnitude();
    if (mag < 1e-10) {  // Jika magnitude sangat kecil
        return Vector3D(0, 0, 0);  // Kembalikan vektor nol
    }
    return *this / mag;
}

void Vector3D::normalize() {
    double mag = magnitude();
    if (mag > 1e-10) {  // Hanya normalisasi jika magnitude cukup besar
        x /= mag;
        y /= mag;
        z /= mag;
    }
    // Jika magnitude terlalu kecil, biarkan tetap (0,0,0)
}

// Operasi utilitas
double Vector3D::distanceTo(const Vector3D& other) const {
    return (*this - other).magnitude();
}

Vector3D Vector3D::reflect(const Vector3D& normal) const {
    // Rumus refleksi: v' = v - 2(v·n)n
    return *this - normal * (2.0 * this->dot(normal));
}

// Operator perbandingan
bool Vector3D::operator==(const Vector3D& other) const {
    const double EPSILON = 1e-10;
    return (std::abs(x - other.x) < EPSILON &&
            std::abs(y - other.y) < EPSILON &&
            std::abs(z - other.z) < EPSILON);
}

bool Vector3D::operator!=(const Vector3D& other) const {
    return !(*this == other);
}

// Output stream
std::ostream& operator<<(std::ostream& os, const Vector3D& vec) {
    os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
    return os;
}