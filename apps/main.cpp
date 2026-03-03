#include "../src/core/Missile.hpp"
#include "../src/core/Simulation.hpp"
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
        
        myMissile.setMass(500.0);                               // 500 kg
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
        
        // Buat simulasi
        Simulation sim(0.01);  // Time step 10 ms
        
        // Set callback untuk monitoring
        sim.setStepCallback([](const Simulation& s, const TrajectoryData& data) {
            // Tampilkan setiap 1 detik
            if (std::abs(data.time - std::round(data.time)) < 0.005) {
                std::cout << "t = " << std::fixed << std::setprecision(2) << data.time 
                          << " s, altitude = " << std::fixed << std::setprecision(1) << data.altitude 
                          << " m, speed = " << std::fixed << std::setprecision(1) << data.velocity.magnitude() 
                          << " m/s, mass = " << std::fixed << std::setprecision(1) << data.mass << " kg\n";
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