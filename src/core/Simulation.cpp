#include "Simulation.hpp"
#include "NumericalIntegrator.hpp"
#include "MissileDynamics.hpp"
#include "StateDerivatives.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

// Konstruktor
Simulation::Simulation() : Simulation(0.01) {}  // Default 10ms step

Simulation::Simulation(double dt) 
    : missile(nullptr)
    , currentTime(0.0)
    , timeStep(dt)
    , maxTime(100.0)
    , stepsPerformed(0)
    , integrationError(0.0)
{
    if (dt <= 0) {
        throw std::runtime_error("Time step must be positive");
    }
}

Simulation::~Simulation() {
    // Missile di-manage secara eksternal, tidak kita delete di sini
}

// Setter
void Simulation::setMissile(Missile* m) {
    missile = m;
}

void Simulation::setTimeStep(double dt) {
    if (dt <= 0) {
        throw std::runtime_error("Time step must be positive");
    }
    timeStep = dt;
}

void Simulation::setMaxTime(double tmax) {
    if (tmax <= 0) {
        throw std::runtime_error("Max time must be positive");
    }
    maxTime = tmax;
}

void Simulation::setStepCallback(std::function<void(const Simulation&, const TrajectoryData&)> callback) {
    stepCallback = callback;
}

// Inisialisasi
bool Simulation::initialize() {
    if (!missile) {
        std::cerr << "Error: No missile set for simulation" << std::endl;
        return false;
    }
    
    // Reset state
    currentTime = 0.0;
    stepsPerformed = 0;
    integrationError = 0.0;
    trajectory.clear();
    
    // Record initial state
    recordCurrentState();
    
    return true;
}

// Jalankan simulasi penuh
bool Simulation::run() {
    if (!initialize()) {
        return false;
    }
    
    std::cout << "Starting simulation..." << std::endl;
    std::cout << "Time step: " << timeStep << " s" << std::endl;
    std::cout << "Max time: " << maxTime << " s" << std::endl;
    
    while (currentTime < maxTime) {
        if (!step()) {
            std::cerr << "Simulation failed at t = " << currentTime << " s" << std::endl;
            return false;
        }
        
        // Cek impact dengan tanah (setelah fase awal peluncuran)
        if (currentTime > 1.0 && missile->getPosition().getY() <= 0) {
            std::cout << "\nMissile impacted ground at t = " 
                      << std::fixed << std::setprecision(2) << currentTime << " s" << std::endl;
            break;
        }
        
        // Progress indicator setiap 10%
        if (stepsPerformed % static_cast<int>(maxTime / timeStep / 10) == 0) {
            printProgress();
        }
    }
    
    std::cout << "\nSimulation completed!" << std::endl;
    printSummary();
    
    return true;
}

// Satu langkah simulasi
bool Simulation::step() {
    if (!missile) {
        return false;
    }
    
    try {
        // Validasi state sebelum step
        Vector3D pos = missile->getPosition();
        if (std::isnan(pos.getX()) || std::isinf(pos.getX())) {
            std::cerr << "Invalid position detected at t=" << currentTime << std::endl;
            return false;
        }
        
        // Gunakan RK4 dengan try-catch internal
        try {
            rk4Step(timeStep);
        } catch (const std::exception& e) {
            std::cerr << "RK4 step failed at t=" << currentTime << ": " << e.what() << std::endl;
            // Fallback ke Euler dengan step lebih kecil
            double smallStep = timeStep / 10.0;
            for (int i = 0; i < 10; i++) {
                eulerStep(smallStep);
            }
        }
        
        // Update waktu
        currentTime += timeStep;
        stepsPerformed++;
        
        // Record data
        recordCurrentState();
        
        // Panggil callback jika ada
        if (stepCallback && !trajectory.empty()) {
            stepCallback(*this, trajectory.back());
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error during simulation step at t=" 
                  << currentTime << ": " << e.what() << std::endl;
        return false;
    }
}

// Metode Euler (sederhana, kurang akurat)
void Simulation::eulerStep(double dt) {
    // Simpan state sebelum update untuk estimasi error
    Vector3D oldPos = missile->getPosition();
    
    // Update missile dengan dt
    missile->update(dt);
    
    // Estimasi error sederhana (perubahan posisi)
    Vector3D newPos = missile->getPosition();
    integrationError = (newPos - oldPos).magnitude() / dt;
}

// Runge-Kutta Orde 4 (lebih akurat)
void Simulation::rk4Step(double dt) {
    if (!missile) return;
    
    // Dapatkan gaya-gaya saat ini
    auto fm = missile->computeForcesAndMoments();
    
    // Buat lambda function untuk menghitung derivatives
    auto computeDerivs = [&fm](const Vector3D& pos, 
                               const Vector3D& vel,
                               const Vector3D& ori,
                               const Vector3D& angVel,
                               double mass,
                               double time) -> StateDerivatives {
        // Panggil MissileDynamics dengan gaya yang sudah dihitung
        return MissileDynamics::computeDerivatives(
            pos, vel, ori, angVel, mass, fm
        );
    };
    
    // Ambil state saat ini
    Vector3D pos = missile->getPosition();
    Vector3D vel = missile->getVelocity();
    Vector3D ori = missile->getOrientation();
    Vector3D angVel = missile->getAngularVelocity();
    double mass = missile->getCurrentMass();
    double time = missile->getCurrentTime();
    
    // Lakukan integrasi RK4
    NumericalIntegrator::rk4Step(
        pos, vel, ori, angVel, mass, time, dt, computeDerivs
    );
    
    // Aplikasikan hasil ke missile
    // TODO: Buat method setState di Missile
    // Sementara kita update langsung lewat method yang ada?
    // Ini perlu penanganan khusus
    
    // Untuk sementara, kita update manual
    // missile->setPosition(pos);  // Perlu method setter
    // missile->setVelocity(vel);  // Perlu method setter
    
    // Karena tidak ada setter, kita gunakan applyDerivatives?
    // Tapi applyDerivatives butuh derivatives, bukan state akhir
    
    // SOLUSI SEMENTARA: Tetap pakai Euler dulu
    eulerStep(dt);
}

// Record state saat ini
void Simulation::recordCurrentState() {
    if (!missile) return;
    
    TrajectoryData data;
    data.time = currentTime;
    data.position = missile->getPosition();
    data.velocity = missile->getVelocity();
    data.orientation = missile->getOrientation();
    data.mass = missile->getCurrentMass();
    data.thrust = missile->getCurrentThrust();
    data.machNumber = missile->getMachNumber();
    data.altitude = data.position.getY();  // Asumsi Y adalah ketinggian
    
    trajectory.push_back(data);
}

// Reset simulasi
void Simulation::reset() {
    if (missile) {
        missile->reset();
    }
    currentTime = 0.0;
    stepsPerformed = 0;
    integrationError = 0.0;
    trajectory.clear();
    recordCurrentState();
}

// Ekspor ke CSV
bool Simulation::exportToCSV(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }
    
    // Header
    file << "Time(s),X(m),Y(m),Z(m),Vx(m/s),Vy(m/s),Vz(m/s),"
         << "Roll(rad),Pitch(rad),Yaw(rad),Mass(kg),Thrust(N),Mach,Altitude(m)\n";
    
    // Data
    for (const auto& data : trajectory) {
        file << std::fixed << std::setprecision(6)
             << data.time << ","
             << data.position.getX() << "," << data.position.getY() << "," << data.position.getZ() << ","
             << data.velocity.getX() << "," << data.velocity.getY() << "," << data.velocity.getZ() << ","
             << data.orientation.getX() << "," << data.orientation.getY() << "," << data.orientation.getZ() << ","
             << data.mass << "," << data.thrust << "," << data.machNumber << "," << data.altitude << "\n";
    }
    
    file.close();
    std::cout << "Data exported to " << filename << std::endl;
    return true;
}

// Ekspor ke JSON (sederhana)
bool Simulation::exportToJSON(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }
    
    file << "{\n";
    file << "  \"simulation\": {\n";
    file << "    \"timeStep\": " << timeStep << ",\n";
    file << "    \"totalSteps\": " << stepsPerformed << ",\n";
    file << "    \"totalTime\": " << currentTime << "\n";
    file << "  },\n";
    file << "  \"trajectory\": [\n";
    
    for (size_t i = 0; i < trajectory.size(); i++) {
        const auto& data = trajectory[i];
        file << "    {\n";
        file << "      \"time\": " << data.time << ",\n";
        file << "      \"position\": [" << data.position.getX() << ", " << data.position.getY() << ", " << data.position.getZ() << "],\n";
        file << "      \"velocity\": [" << data.velocity.getX() << ", " << data.velocity.getY() << ", " << data.velocity.getZ() << "],\n";
        file << "      \"mach\": " << data.machNumber << ",\n";
        file << "      \"altitude\": " << data.altitude << "\n";
        file << "    }" << (i < trajectory.size() - 1 ? "," : "") << "\n";
    }
    
    file << "  ]\n";
    file << "}\n";
    
    file.close();
    std::cout << "Data exported to " << filename << std::endl;
    return true;
}

// Analisis
double Simulation::getMaxAltitude() const {
    double maxAlt = 0.0;
    for (const auto& data : trajectory) {
        if (data.altitude > maxAlt) {
            maxAlt = data.altitude;
        }
    }
    return maxAlt;
}

double Simulation::getMaxSpeed() const {
    double maxSpeed = 0.0;
    for (const auto& data : trajectory) {
        double speed = data.velocity.magnitude();
        if (speed > maxSpeed) {
            maxSpeed = speed;
        }
    }
    return maxSpeed;
}

double Simulation::getMaxRange() const {
    if (trajectory.empty()) return 0.0;
    
    // Range adalah jarak horizontal dari titik awal
    Vector3D startPos = trajectory.front().position;
    double maxRange = 0.0;
    
    for (const auto& data : trajectory) {
        double dx = data.position.getX() - startPos.getX();
        double dz = data.position.getZ() - startPos.getZ();
        double range = std::sqrt(dx*dx + dz*dz);
        if (range > maxRange) {
            maxRange = range;
        }
    }
    
    return maxRange;
}

Vector3D Simulation::getImpactPoint() const {
    // Cari titik di mana Y <= 0 (menyentuh tanah)
    for (size_t i = 1; i < trajectory.size(); i++) {
        if (trajectory[i].position.getY() <= 0) {
            // Interpolasi linear antara titik i-1 dan i
            double y1 = trajectory[i-1].position.getY();
            double y2 = trajectory[i].position.getY();
            
            if (y1 > 0) {  // Past kita melewati Y=0
                double fraction = y1 / (y1 - y2);
                
                Vector3D impact;
                impact.setX(trajectory[i-1].position.getX() + 
                           fraction * (trajectory[i].position.getX() - trajectory[i-1].position.getX()));
                impact.setY(0.0);
                impact.setZ(trajectory[i-1].position.getZ() + 
                           fraction * (trajectory[i].position.getZ() - trajectory[i-1].position.getZ()));
                
                return impact;
            }
        }
    }
    
    return Vector3D(0, 0, 0);  // Tidak impact
}

// Utilitas
void Simulation::printProgress() const {
    int percent = static_cast<int>(100.0 * currentTime / maxTime);
    std::cout << "\rProgress: " << percent << "% complete at t = " 
              << std::fixed << std::setprecision(1) << currentTime << " s" << std::flush;
}

void Simulation::printSummary() const {
    std::cout << "\n\n========== SIMULATION SUMMARY ==========\n";
    std::cout << "Total time simulated: " << std::fixed << std::setprecision(2) << currentTime << " s\n";
    std::cout << "Number of steps: " << stepsPerformed << "\n";
    std::cout << "Time step: " << timeStep << " s\n";
    
    double maxAlt = getMaxAltitude();
    double maxSpd = getMaxSpeed();
    double maxRng = getMaxRange();
    
    // Tampilkan dalam unit yang sesuai (km jika besar, m jika kecil)
    if (maxAlt > 10000.0) {
        std::cout << "Max altitude: " << std::setprecision(2) << maxAlt / 1000.0 << " km\n";
    } else {
        std::cout << "Max altitude: " << std::setprecision(2) << maxAlt << " m\n";
    }
    
    std::cout << "Max speed: " << std::setprecision(1) << maxSpd << " m/s ("
              << std::setprecision(1) << maxSpd * 3.6 << " km/h, Mach "
              << std::setprecision(2) << maxSpd / 295.0 << ")\n";
    
    if (maxRng > 10000.0) {
        std::cout << "Max range: " << std::setprecision(2) << maxRng / 1000.0 << " km\n";
    } else {
        std::cout << "Max range: " << std::setprecision(2) << maxRng << " m\n";
    }
    
    Vector3D impact = getImpactPoint();
    if (impact.magnitude() > 10000.0) {
        std::cout << "Impact point: X = " << std::setprecision(2) << impact.getX() / 1000.0
                  << " km, Z = " << std::setprecision(2) << impact.getZ() / 1000.0 << " km\n";
    } else {
        std::cout << "Impact point: X = " << std::setprecision(1) << impact.getX() 
                  << " m, Z = " << std::setprecision(1) << impact.getZ() << " m\n";
    }
    std::cout << "========================================\n";
}