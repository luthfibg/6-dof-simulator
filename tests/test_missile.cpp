#include "../src/core/Missile.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

void testMissileInitialization() {
    Missile m("Test Missile");
    
    assert(m.getPosition() == Vector3D(0, 0, 0));
    assert(m.getVelocity() == Vector3D(0, 0, 0));
    assert(m.getCurrentMass() == 100.0);  // Default mass
    assert(m.getCurrentTime() == 0.0);
    
    std::cout << "✓ Missile initialization test passed\n";
}

void testMissileSetters() {
    Missile m;
    
    m.setInitialPosition(Vector3D(100, 200, 300));
    m.setInitialVelocity(Vector3D(10, 20, 30));
    m.setMass(500.0);
    m.setReferenceArea(0.5);
    
    assert(m.getPosition() == Vector3D(100, 200, 300));
    assert(m.getVelocity() == Vector3D(10, 20, 30));
    assert(m.getCurrentMass() == 500.0);
    
    std::cout << "✓ Missile setters test passed\n";
}

void testMissileGravity() {
    Missile m;
    m.setMass(100.0);
    
    Vector3D gravity = m.computeGravity();
    
    // Gravitasi harus ke bawah (negatif Y)
    assert(gravity.getX() == 0.0);
    assert(gravity.getY() < 0.0);
    assert(gravity.getZ() == 0.0);
    assert(std::abs(gravity.magnitude() - 981.0) < 1e-10);  // 100 kg * 9.81
    
    std::cout << "✓ Gravity computation test passed\n";
}

void testMissileUpdate() {
    Missile m;
    m.setMass(100.0);
    m.setInitialPosition(Vector3D(0, 1000, 0));  // Mulai dari ketinggian 1000m
    m.setInitialVelocity(Vector3D(100, 0, 0));   // Gerak horizontal
    
    // Simulasi 1 detik
    m.update(1.0);
    
    // Setelah 1 detik dengan gravitasi, posisi Y harus berkurang
    assert(m.getPosition().getY() < 1000.0);
    assert(m.getCurrentTime() == 1.0);
    
    std::cout << "✓ Missile update test passed\n";
}

void testMissilePropulsion() {
    Missile m;
    m.setMass(100.0);
    m.setPropulsion(5.0, 10000.0, 3000.0);
    
    // Simulasi selama waktu bakar
    for (int i = 0; i < 10; i++) {
        m.update(0.5);
    }
    
    // Setelah 5 detik, thrust harus 0
    assert(m.getCurrentThrust() == 0.0);
    assert(m.getCurrentTime() == 5.0);
    
    // Massa harus berkurang
    assert(m.getCurrentMass() < 100.0);
    
    std::cout << "✓ Missile propulsion test passed\n";
}

void testMissileReset() {
    Missile m;
    m.setInitialPosition(Vector3D(100, 200, 300));
    m.update(1.0);
    m.reset();
    
    assert(m.getPosition() == Vector3D(0, 0, 0));
    assert(m.getCurrentTime() == 0.0);
    
    std::cout << "✓ Missile reset test passed\n";
}

int main() {
    std::cout << "Running Missile class tests...\n\n";
    
    testMissileInitialization();
    testMissileSetters();
    testMissileGravity();
    testMissileUpdate();
    testMissilePropulsion();
    testMissileReset();
    
    std::cout << "\n✅ All Missile tests passed!\n";
    
    return 0;
}