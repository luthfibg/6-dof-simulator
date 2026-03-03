#include "../src/core/Vector3D.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

void testConstructor() {
    Vector3D v1;
    assert(v1.getX() == 0.0 && v1.getY() == 0.0 && v1.getZ() == 0.0);
    
    Vector3D v2(1.0, 2.0, 3.0);
    assert(v2.getX() == 1.0 && v2.getY() == 2.0 && v2.getZ() == 3.0);
    
    std::cout << "✓ Constructor test passed\n";
}

void testAddition() {
    Vector3D v1(1.0, 2.0, 3.0);
    Vector3D v2(4.0, 5.0, 6.0);
    
    Vector3D result = v1 + v2;
    assert(result == Vector3D(5.0, 7.0, 9.0));
    
    std::cout << "✓ Addition test passed\n";
}

void testSubtraction() {
    Vector3D v1(4.0, 5.0, 6.0);
    Vector3D v2(1.0, 2.0, 3.0);
    
    Vector3D result = v1 - v2;
    assert(result == Vector3D(3.0, 3.0, 3.0));
    
    std::cout << "✓ Subtraction test passed\n";
}

void testScalarMultiplication() {
    Vector3D v(1.0, 2.0, 3.0);
    
    Vector3D result = v * 2.0;
    assert(result == Vector3D(2.0, 4.0, 6.0));
    
    std::cout << "✓ Scalar multiplication test passed\n";
}

void testDotProduct() {
    Vector3D v1(1.0, 2.0, 3.0);
    Vector3D v2(4.0, 5.0, 6.0);
    
    double dot = v1.dot(v2);
    assert(std::abs(dot - 32.0) < 1e-10); // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    
    std::cout << "✓ Dot product test passed\n";
}

void testCrossProduct() {
    Vector3D v1(1.0, 0.0, 0.0);  // Vektor sumbu X
    Vector3D v2(0.0, 1.0, 0.0);  // Vektor sumbu Y
    
    Vector3D cross = v1.cross(v2);
    assert(cross == Vector3D(0.0, 0.0, 1.0)); // Hasilnya harus sumbu Z
    
    std::cout << "✓ Cross product test passed\n";
}

void testMagnitude() {
    Vector3D v(3.0, 4.0, 0.0);
    
    assert(std::abs(v.magnitude() - 5.0) < 1e-10);      // √(3²+4²+0²) = 5
    assert(std::abs(v.magnitudeSquared() - 25.0) < 1e-10); // 3²+4²+0² = 25
    
    std::cout << "✓ Magnitude test passed\n";
}

void testNormalization() {
    Vector3D v(3.0, 0.0, 0.0);
    Vector3D normalized = v.normalized();
    
    assert(normalized == Vector3D(1.0, 0.0, 0.0));
    assert(std::abs(normalized.magnitude() - 1.0) < 1e-10);
    
    std::cout << "✓ Normalization test passed\n";
}

int main() {
    std::cout << "Running Vector3D tests...\n\n";
    
    testConstructor();
    testAddition();
    testSubtraction();
    testScalarMultiplication();
    testDotProduct();
    testCrossProduct();
    testMagnitude();
    testNormalization();
    
    std::cout << "\n✅ All Vector3D tests passed!\n";
    
    return 0;
}