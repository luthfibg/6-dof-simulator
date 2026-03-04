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

enum class GuidanceMode {
    PurePursuit,
    ProportionalNavigation,
    BallisticBoost   // Profil pitch terprogram (boost) + free-flight ballistic (coast)
};

// Helper sederhana untuk membaca target dari argumen CLI
// Format: missile_sim.exe --target X Y Z
// Koordinat dalam meter, dengan konvensi X=range, Y=altitude, Z=crossrange.
static Vector3D parseTargetFromArgs(int argc, char* argv[], const Vector3D& defaultTarget) {
    for (int i = 1; i + 3 < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--target") {
            try {
                double x = std::stod(argv[i+1]);
                double y = std::stod(argv[i+2]);
                double z = std::stod(argv[i+3]);
                return Vector3D(x, y, z);
            } catch (...) {
                std::cerr << "Invalid --target arguments. Using default target.\n";
                return defaultTarget;
            }
        }
    }
    return defaultTarget;
}

// Parse --launch-angle <degrees>  (manual override sudut peluncuran)
static double parseLaunchAngleFromArgs(int argc, char* argv[]) {
    for (int i = 1; i + 1 < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--launch-angle") {
            try {
                double deg = std::stod(argv[i+1]);
                return deg;  // return degrees, negative means "use auto"
            } catch (...) {
                std::cerr << "Invalid --launch-angle value. Using auto.\n";
                return -1.0;
            }
        }
    }
    return -1.0;  // sentinel: use auto
}

static GuidanceMode parseGuidanceModeFromArgs(int argc, char* argv[], GuidanceMode defaultMode) {
    for (int i = 1; i + 1 < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--guidance") {
            std::string mode = argv[i+1];
            if (mode == "pure") {
                return GuidanceMode::PurePursuit;
            } else if (mode == "pn") {
                return GuidanceMode::ProportionalNavigation;
            } else if (mode == "ballistic") {
                return GuidanceMode::BallisticBoost;
            } else {
                std::cerr << "Unknown guidance mode '" << mode 
                          << "'. Use 'pure', 'pn', or 'ballistic'. Defaulting to 'ballistic'.\n";
                return defaultMode;
            }
        }
    }
    return defaultMode;
}

int main(int argc, char* argv[]) {
    std::cout << "=== Medium-Range Ballistic Missile 6-DoF Simulation ===\n\n";
    
    try {
        // ================================================================
        //  SPESIFIKASI RUDAL BALISTIK JARAK MENENGAH (MRBM)
        //  Kelas: Rajawali-2 (terinspirasi R-17 Elbrus / Scud-C / DF-15)
        //  Propelan: Cair (IRFNA / UDMH), satu tahap
        //  Jangkauan desain: 150 - 300 km
        // ================================================================
        
        Missile myMissile("Rajawali-2 MRBM");
        
        // --- Kondisi awal peluncuran dari TEL ---
        constexpr double V0 = 5.0;                     // Kecepatan awal 5 m/s
        
        // --- Spesifikasi massa ---
        myMissile.setMass(6500.0);                                 // 6.500 kg massa peluncuran
        myMissile.setFuelMass(4000.0);                             // 4.000 kg propelan cair
        //                                                          // 2.500 kg struktur + warhead
        
        // --- Geometri ---
        myMissile.setReferenceArea(0.608);                         // pi * (0.44)^2 m² (diameter 0.88m)
        myMissile.setReferenceLength(11.0);                        // Panjang total 11.0 m
        
        // --- Koefisien aerodinamika (body silinder-konis streamline) ---
        myMissile.setAerodynamicCoeffs(
            0.12,     // Cd0 — drag zero-lift (low-drag supersonic body)
            1.8,      // CLa — lift curve slope (moderat, lebih rendah dari rudal taktis)
            0.0,      // Cm0 — zero-alpha moment
            -0.3      // Cma — static pitch stiffness (stabil secara aerodinamis)
        );
        
        // --- Propulsi: motor roket cair satu tahap ---
        //   Isp ≈ 230 s → Total impulse = m_fuel × Isp × g0 ≈ 9.025 MN·s
        //   Peak thrust: T_peak = 1.5 × I_total / t_burn (profil parabola)
        constexpr double BURN_TIME      = 65.0;       // Waktu bakar [s]
        constexpr double TOTAL_IMPULSE  = 9100000.0;  // Total impulse [N·s]
        constexpr double PEAK_THRUST    = 210000.0;    // Peak thrust [N] (210 kN)
        myMissile.setPropulsion(BURN_TIME, TOTAL_IMPULSE, PEAK_THRUST);
        
        // --- Cetak spesifikasi rudal ---
        std::cout << "Missile: Rajawali-2 MRBM\n";
        std::cout << "+-- Mass: 6,500 kg (fuel: 4,000 kg / dry: 2,500 kg)\n";
        std::cout << "+-- Geometry: D = 0.88 m, L = 11.0 m\n";
        std::cout << "+-- Propulsion: Liquid (Isp ~ 230 s)\n";
        std::cout << "+-- Peak thrust: 210 kN, Burn time: 65 s\n";
        std::cout << "+-- Total impulse: 9.1 MN.s\n";
        std::cout << "+-- Aero: Cd0=0.12, CLa=1.8, Cma=-0.3\n\n";
        
        // ============ LEVEL 5: KENDALI & GUIDANCE ============
        
        // Sirip kendali (skala MRBM — lebih besar, defleksi sedikit lebih kecil)
        myMissile.addControlSurface(ControlSurface("elevator", 0.30, 0.20, Vector3D(-5.0, 0, 0)));
        myMissile.addControlSurface(ControlSurface("rudder",   0.30, 0.15, Vector3D(-5.0, 0, 0.4)));
        myMissile.addControlSurface(ControlSurface("aileron",  0.30, 0.15, Vector3D(-5.0, 0.4, 0)));
        
        // Autopilot PID (disesuaikan untuk MoI besar MRBM + TVC authority)
        Autopilot autopilot;
        autopilot.setTimeStep(0.01);
        autopilot.setPitchGains(5.0, 0.3, 1.5);     // P, I, D untuk pitch
        autopilot.setRollGains(3.0, 0.15, 0.8);     // P, I, D untuk roll
        autopilot.setYawGains(4.0, 0.25, 1.2);      // P, I, D untuk yaw
        
        // Guidance system
        Guidance guidance;
        Vector3D defaultTarget(200000, 0, 0);   // Default: 200 km downrange
        Vector3D target = parseTargetFromArgs(argc, argv, defaultTarget);
        guidance.setTarget(target);
        GuidanceMode defaultMode = GuidanceMode::BallisticBoost;  // Default MRBM: ballistic
        GuidanceMode guidanceMode = parseGuidanceModeFromArgs(argc, argv, defaultMode);
        
        // --- Hitung sudut peluncuran dari jarak target ---
        // Estimasi kecepatan burnout dari Tsiolkovsky
        double targetRange = std::sqrt(target.getX()*target.getX() + target.getZ()*target.getZ());
        double massRatio = 6500.0 / 2500.0;
        double isp = TOTAL_IMPULSE / (4000.0 * 9.80665);
        double idealDeltaV = isp * 9.80665 * std::log(massRatio);
        double estBurnoutVel = idealDeltaV * 0.72;  // ~72% efficiency (gravity + drag losses)
        
        // Estimasi jangkauan maksimum: R_max ≈ V²/g × bonus_ketinggian
        double maxRangeEst = estBurnoutVel * estBurnoutVel / 9.80665 * 1.3;
        bool outOfRange = targetRange > maxRangeEst;
        if (outOfRange) {
            std::cout << "WARNING: Target (" << targetRange/1000.0 
                      << " km) mungkin melebihi jangkauan maks rudal (~" 
                      << maxRangeEst/1000.0 << " km)!\n";
        }
        
        // Model empiris: sudut peluncuran = f(jarak/jarak_maks)
        // Dikalibrasi dari simulasi gravity turn MRBM Rajawali-2:
        //   ~75° @ 100 km, ~79° @ 200 km, ~84° @ 300 km
        double rangeFrac = std::max(0.05, std::min(1.0, targetRange / maxRangeEst));
        double launchPitchDeg = 71.0 + 14.0 * std::pow(rangeFrac, 1.1);
        
        // Override manual jika ada --launch-angle
        double manualAngle = parseLaunchAngleFromArgs(argc, argv);
        if (manualAngle > 0.0) {
            launchPitchDeg = manualAngle;
            std::cout << "Manual launch angle override: " << manualAngle << " deg\n";
        }
        double launchPitch = launchPitchDeg * 3.14159265 / 180.0;
        
        // Set kondisi awal dengan sudut peluncuran yang dihitung
        myMissile.setInitialPosition(Vector3D(0, 20, 0));
        myMissile.setInitialVelocity(Vector3D(
            V0 * std::cos(launchPitch),
            V0 * std::sin(launchPitch),
            0
        ));
        myMissile.setInitialOrientation(Vector3D(0, launchPitch, 0));
        
        std::cout << "Autopilot + TVC + Guidance initialized\n";
        std::cout << "Target: (" << target.getX()/1000.0 << " km, "
                  << target.getY()/1000.0 << " km, "
                  << target.getZ()/1000.0 << " km)\n";
        
        const char* modeName = "Ballistic (Gravity Turn)";
        if (guidanceMode == GuidanceMode::PurePursuit) modeName = "Pure Pursuit";
        else if (guidanceMode == GuidanceMode::ProportionalNavigation) modeName = "Proportional Navigation";
        std::cout << "Guidance mode: " << modeName << "\n";
        
        std::cout << "Launch pitch: " << std::fixed << std::setprecision(1) << launchPitchDeg << " deg\n";
        std::cout << "Est. burnout velocity: " << std::setprecision(0) << estBurnoutVel << " m/s\n";
        std::cout << "Est. max range: " << std::setprecision(0) << maxRangeEst/1000.0 << " km\n\n";
        
        // ============ SIMULASI ============
        
        Simulation sim(0.01);  // Time step 10 ms
        
        // Callback: monitoring + kendali per-fase
        sim.setStepCallback([&autopilot, &guidance, &myMissile, guidanceMode,
                             BURN_TIME, launchPitch, &target]
                            (const Simulation& s, const TrajectoryData& data) {
            // --- Fase penerbangan ---
            bool isBoosting   = data.time <= BURN_TIME;
            bool isDescending = !isBoosting && data.velocity.getY() < 0;
            
            // Interval cetak adaptif
            double interval = 30.0;
            if (isBoosting) interval = 5.0;
            else if (isDescending && data.altitude < 30000) interval = 5.0;
            
            bool shouldPrint = std::abs(data.time - std::round(data.time / interval) * interval) < 0.005;
            bool atBurnout   = std::abs(data.time - BURN_TIME) < 0.015;
            
            if (shouldPrint || atBurnout) {
                const char* phase = isBoosting ? "[BOOST] " : (isDescending ? "[REENTRY]" : "[COAST]  ");
                std::cout << phase
                          << " t=" << std::fixed << std::setprecision(1) << data.time << "s"
                          << "  alt=" << std::setprecision(2) << data.altitude / 1000.0 << " km"
                          << "  v=" << std::setprecision(0) << data.velocity.magnitude() << " m/s"
                          << " (M" << std::setprecision(2) << data.machNumber << ")"
                          << "  mass=" << std::setprecision(0) << data.mass << " kg"
                          << "\n";
            }
            
            // --- GUIDANCE & CONTROL ---
            if (guidanceMode == GuidanceMode::BallisticBoost) {
                // ===== MODE BALISTIK: GRAVITY TURN =====
                if (isBoosting) {
                    double desiredPitch;
                    double speed = data.velocity.magnitude();
                    
                    if (speed < 20.0 || data.time < 3.0) {
                        // Phase 1: Hold launch angle (kecepatan terlalu rendah untuk FPA)
                        desiredPitch = launchPitch;
                    } else {
                        // Phase 2: Gravity turn — ikuti arah kecepatan (α ≈ 0)
                        // Ini meminimalkan drag dan beban struktural
                        double vx = data.velocity.getX();
                        double vy = data.velocity.getY();
                        double vz = data.velocity.getZ();
                        double vHoriz = std::sqrt(vx*vx + vz*vz);
                        desiredPitch = std::atan2(vy, std::max(vHoriz, 1.0));
                    }
                    
                    double desiredYaw = std::atan2(target.getZ(), 
                                                    std::max(target.getX(), 1.0));
                    
                    auto deflections = autopilot.computeControlSurfaces(
                        Vector3D(0, desiredPitch, desiredYaw),
                        data.orientation,
                        myMissile.getAngularVelocity()
                    );
                    if (deflections.size() >= 3) {
                        myMissile.setControlDeflections(deflections[0], deflections[1], deflections[2]);
                    }
                } else {
                    // Ballistic free-flight: sirip netral
                    myMissile.setControlDeflections(0, 0, 0);
                }
            } else {
                // ===== MODE PURSUIT / PN (untuk rudal taktis) =====
                Guidance::GuidanceCommand guidanceCmd;
                if (guidanceMode == GuidanceMode::PurePursuit) {
                    guidanceCmd = guidance.purePursuit(data.position, data.velocity);
                } else {
                    guidanceCmd = guidance.proportionalNavigation(
                        data.position, data.velocity, s.getTimeStep()
                    );
                }
                auto deflections = autopilot.computeControlSurfaces(
                    guidanceCmd.desiredOrientation,
                    data.orientation,
                    myMissile.getAngularVelocity()
                );
                if (deflections.size() >= 3) {
                    myMissile.setControlDeflections(deflections[0], deflections[1], deflections[2]);
                }
            }
        });
        
        sim.setMissile(&myMissile);
        sim.setMaxTime(600.0);  // 10 menit — cukup untuk MRBM 200-300 km
        
        auto startTime = std::chrono::high_resolution_clock::now();
        
        if (sim.run()) {
            auto endTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
            
            std::cout << "\nSimulation completed in " << duration.count() / 1000.0 
                      << " seconds\n";
            
            if (sim.exportToCSV("trajectory.csv")) {
                std::cout << "Data exported to trajectory.csv\n";
            }
            if (sim.exportToJSON("trajectory.json")) {
                std::cout << "Data exported to trajectory.json\n";
            }
            
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