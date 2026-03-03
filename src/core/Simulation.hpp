#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "Missile.hpp"
#include <vector>
#include <string>
#include <functional>

// Struct untuk menyimpan data trajectory
struct TrajectoryData {
    double time;
    Vector3D position;
    Vector3D velocity;
    Vector3D orientation;
    double mass;
    double thrust;
    double machNumber;
    double altitude;
};

class Simulation {
private:
    Missile* missile;              // Pointer ke rudal yang disimulasi
    double currentTime;             // Waktu saat ini [s]
    double timeStep;                // Langkah waktu integrasi [s]
    double maxTime;                 // Maksimum waktu simulasi [s]
    
    // Data trajectory
    std::vector<TrajectoryData> trajectory;
    
    // Callback untuk monitoring
    std::function<void(const Simulation&, const TrajectoryData&)> stepCallback;
    
    // Statistik
    int stepsPerformed;
    double integrationError;
    
public:
    // Konstruktor
    Simulation();
    explicit Simulation(double dt);
    
    // Destruktor
    ~Simulation();
    
    // Setter
    void setMissile(Missile* m);
    void setTimeStep(double dt);
    void setMaxTime(double tmax);
    void setStepCallback(std::function<void(const Simulation&, const TrajectoryData&)> callback);
    
    // Getter
    double getCurrentTime() const { return currentTime; }
    double getTimeStep() const { return timeStep; }
    int getStepsPerformed() const { return stepsPerformed; }
    const std::vector<TrajectoryData>& getTrajectory() const { return trajectory; }
    
    // Method utama simulasi
    bool initialize();              // Persiapan sebelum run
    bool run();                     // Jalankan simulasi penuh
    bool step();                    // Satu langkah simulasi
    void reset();                   // Reset simulasi
    
    // Method integrasi numerik
    void eulerStep(double dt);      // Metode Euler (sederhana)
    void rk4Step(double dt);        // Runge-Kutta orde 4 (akurat)
    
    // Method pencatatan data
    void recordCurrentState();
    
    // Method ekspor data
    bool exportToCSV(const std::string& filename) const;
    bool exportToJSON(const std::string& filename) const;
    
    // Method analisis
    double getMaxAltitude() const;
    double getMaxSpeed() const;
    double getMaxRange() const;
    Vector3D getImpactPoint() const;  // Titik jatuh (asumsi y=0)
    
    // Method utilitas
    void printProgress() const;
    void printSummary() const;
};

#endif // SIMULATION_HPP