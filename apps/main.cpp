#include "../src/core/Missile.hpp"
#include "../src/core/Simulation.hpp"
#include "../src/core/ControlSurface.hpp"
#include "../src/core/Autopilot.hpp"
#include "../src/core/Guidance.hpp"
#include <iostream>
#include <memory>
#include <chrono>
#include <iomanip>      // Untuk setprecision
#include <cmath>        // Untuk std::round

int main() {
    std::cout << "=== Missile 6-DoF Simulation ===\n\n";
    
    try {
        // Buat rudal
        Missile myMissile("Ballistic Missile Mk-1");
        
        // Set parameter rudal dengan validasi
        myMissile.setInitialPosition(Vector3D(0, 100, 0));      // Mulai dari ketinggian 100m
        myMissile.setInitialVelocity(Vector3D(200, 50, 0));    // Kecepatan awal
        myMissile.setInitialOrientation(Vector3D(0, 0.2, 0));   // Sedikit pitch up
        
        myMissile.setMass(500.0);                               // 500 kg total
        myMissile.setFuelMass(200.0);                           // 200 kg bahan bakar, 300 kg struktur
        myMissile.setReferenceArea(0.2);                        // 0.2 m^2
        myMissile.setReferenceLength(2.0);                      // 2 m panjang
        
        // Koefisien aerodinamika
        myMissile.setAerodynamicCoeffs(0.15,    // Cd0
                                       2.5,     // CLa
                                       0.0,     // Cm0
                                       -0.8);   // Cma
        
        // Propulsi - kurangi sedikit untuk menghindari edge case
        myMissile.setPropulsion(7.9,        // Burn time 7.9 detik (bukan 8.0)
                               25000.0,      // Total impulse 25,000 N·s
                               5000.0);      // Peak thrust 5000 N
        
        // ============ LEVEL 5: CONTROL SURFACES & GUIDANCE ============
        
        // Tambahkan sirip kendali (elevator, rudder, aileron)
        myMissile.addControlSurface(ControlSurface("elevator", 0.35, 0.15, Vector3D(-1.0, 0, 0)));
        myMissile.addControlSurface(ControlSurface("rudder", 0.35, 0.1, Vector3D(-1.0, 0, 0.2)));
        myMissile.addControlSurface(ControlSurface("aileron", 0.35, 0.1, Vector3D(-1.0, 0.2, 0)));
        
        // Buat autopilot dengan PID gains
        Autopilot autopilot;
        autopilot.setTimeStep(0.01);
        autopilot.setPitchGains(3.0, 0.2, 0.8);    // P, I, D untuk pitch
        autopilot.setRollGains(2.0, 0.1, 0.5);     // P, I, D untuk roll
        autopilot.setYawGains(2.5, 0.15, 0.6);     // P, I, D untuk yaw
        
        // Buat guidance system
        Guidance guidance;
        guidance.setTarget(Vector3D(5000, 0, 2000));  // Target: 5km range, 2km crossrange
        
        std::cout << "Autopilot and Guidance systems initialized\n";
        std::cout << "Target: (5000m, 0m, 2000m)\n\n";
        
        // Buat simulasi
        Simulation sim(0.01);  // Time step 10 ms
        
        // Set callback untuk monitoring dan kontrol
        sim.setStepCallback([&autopilot, &guidance, &myMissile](const Simulation& s, const TrajectoryData& data) {
            // Tampilkan setiap 1 detik
            if (std::abs(data.time - std::round(data.time)) < 0.005) {
                std::cout << "t = " << std::fixed << std::setprecision(2) << data.time 
                          << " s, altitude = " << std::fixed << std::setprecision(1) << data.altitude 
                          << " m, speed = " << std::fixed << std::setprecision(1) << data.velocity.magnitude() 
                          << " m/s, mass = " << std::fixed << std::setprecision(1) << data.mass << " kg\n";
            }
            
            // Jalankan guidance untuk mendapatkan orientasi target
            auto guidanceCmd = guidance.purePursuit(data.position, data.velocity);
            
            // Jalankan autopilot untuk menghitung defleksi sirip
            auto deflections = autopilot.computeControlSurfaces(
                guidanceCmd.desiredOrientation,
                data.orientation,
                myMissile.getAngularVelocity()
            );
            
            // Terapkan defleksi ke rudal (elevator, rudder, aileron)
            if (deflections.size() >= 3) {
                myMissile.setControlDeflections(deflections[0], deflections[1], deflections[2]);
            }
        });
        
        // Hubungkan rudal dengan simulasi
        sim.setMissile(&myMissile);
        
        // Set waktu simulasi
        sim.setMaxTime(60.0);  // Simulasi 60 detik
        
        // Catat waktu mulai
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // Jalankan simulasi
        if (sim.run()) {
            auto endTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
            
            std::cout << "\nSimulation completed in " << duration.count() / 1000.0 
                      << " seconds\n";
            
            // Ekspor data
            if (sim.exportToCSV("trajectory.csv")) {
                std::cout << "Data exported to trajectory.csv\n";
            }
            if (sim.exportToJSON("trajectory.json")) {
                std::cout << "Data exported to trajectory.json\n";
            }
            
            // Tampilkan summary
            sim.printSummary();
            
        } else {
            std::cerr << "Simulation failed!\n";
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}