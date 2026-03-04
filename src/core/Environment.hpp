#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include "Vector3D.hpp"
#include <cmath>

// Kelas untuk memodelkan lingkungan di mana rudal terbang
// Mengacu pada International Standard Atmosphere (ISA)
class Environment {
public:
    // ============ KONSTANTA FISIKA ============
    static constexpr double R = 287.05287;      // Konstanta gas udara [J/(kg·K)]
    static constexpr double GAMMA = 1.4;         // Rasio panas spesifik
    static constexpr double G0 = 9.80665;        // Gravitasi permukaan [m/s²]
    static constexpr double R_EARTH = 6371000.0; // Radius bumi [m]
    
    // Kondisi permukaan laut (SL - Sea Level)
    static constexpr double T0_SL = 288.15;      // Temperatur [K] (15°C)
    static constexpr double P0_SL = 101325.0;    // Tekanan [Pa]
    static constexpr double RHO0_SL = 1.225;     // Density [kg/m³]
    
    // Lapisan atmosfer
    static constexpr double TROPOPAUSE = 11000.0;  // Ketinggian tropopause [m]
    static constexpr double STRATOPAUSE = 20000.0; // Ketinggian stratopause [m]
    static constexpr double LAPSE_RATE = 0.0065;   // Lapse rate troposfer [K/m]
    
public:
    // ============ GRAVITASI ============
    
    // Gravitasi sebagai fungsi ketinggian (inverse square law)
    static double getGravity(double altitude) {
        // g(h) = g0 * (R / (R + h))^2
        double factor = R_EARTH / (R_EARTH + altitude);
        return G0 * factor * factor;
    }
    
    // Vektor gravitasi (selalu mengarah ke pusat bumi, asumsi -Y)
    static Vector3D getGravityVector(double altitude, double mass) {
        double g = getGravity(altitude);
        return Vector3D(0, -mass * g, 0);
    }
    
    // ============ ATMOSFER ============
    
    // Temperatur sebagai fungsi ketinggian (model ISA 7-lapisan)
    // Referensi: US Standard Atmosphere 1976
    static double getTemperature(double altitude) {
        if (altitude < 0) altitude = 0;  // Clamp negatif
        
        if (altitude <= TROPOPAUSE) {
            // 0-11 km: Troposfer (L = -6.5 K/km)
            return T0_SL - LAPSE_RATE * altitude;
        }
        else if (altitude <= 20000.0) {
            // 11-20 km: Tropopause (isotermal)
            return 216.65;
        }
        else if (altitude <= 32000.0) {
            // 20-32 km: Stratosfer bawah (L = +1.0 K/km)
            return 216.65 + 0.001 * (altitude - 20000.0);
        }
        else if (altitude <= 47000.0) {
            // 32-47 km: Stratosfer atas (L = +2.8 K/km)
            return 228.65 + 0.0028 * (altitude - 32000.0);
        }
        else if (altitude <= 51000.0) {
            // 47-51 km: Stratopause (isotermal)
            return 270.65;
        }
        else if (altitude <= 71000.0) {
            // 51-71 km: Mesosfer (L = -2.8 K/km)
            return 270.65 - 0.0028 * (altitude - 51000.0);
        }
        else if (altitude <= 86000.0) {
            // 71-86 km: Mesosfer atas (L = -2.0 K/km)
            return 214.65 - 0.002 * (altitude - 71000.0);
        }
        else {
            // >86 km: Basis termosfer (konstan untuk simulasi)
            return 186.65;
        }
    }
    
    // Tekanan sebagai fungsi ketinggian (barometric formula berlapis)
    static double getPressure(double altitude) {
        if (altitude < 0) altitude = 0;
        
        if (altitude <= TROPOPAUSE) {
            // 0-11 km: Troposfer (gradient layer)
            double T = getTemperature(altitude);
            double exponent = G0 / (LAPSE_RATE * R);
            return P0_SL * std::pow(T / T0_SL, exponent);
        }
        
        // Hitung P di batas tropopause (11 km)
        double T_11 = 216.65;
        double P_11 = P0_SL * std::pow(T_11 / T0_SL, G0 / (LAPSE_RATE * R));
        
        if (altitude <= 20000.0) {
            // 11-20 km: Isotermal (216.65 K)
            double H = R * 216.65 / G0;
            return P_11 * std::exp(-(altitude - TROPOPAUSE) / H);
        }
        
        // P di 20 km
        double H_tropo = R * 216.65 / G0;
        double P_20 = P_11 * std::exp(-(20000.0 - TROPOPAUSE) / H_tropo);
        
        if (altitude <= 32000.0) {
            // 20-32 km: Gradient layer (L = +0.001 K/m)
            double L = 0.001;
            double T = getTemperature(altitude);
            return P_20 * std::pow(T / 216.65, -G0 / (L * R));
        }
        
        // P di 32 km
        double P_32 = P_20 * std::pow(228.65 / 216.65, -G0 / (0.001 * R));
        
        if (altitude <= 47000.0) {
            // 32-47 km: Gradient layer (L = +0.0028 K/m)
            double L = 0.0028;
            double T = getTemperature(altitude);
            return P_32 * std::pow(T / 228.65, -G0 / (L * R));
        }
        
        // P di 47 km
        double P_47 = P_32 * std::pow(270.65 / 228.65, -G0 / (0.0028 * R));
        
        if (altitude <= 51000.0) {
            // 47-51 km: Isotermal (270.65 K)
            double H = R * 270.65 / G0;
            return P_47 * std::exp(-(altitude - 47000.0) / H);
        }
        
        // P di 51 km
        double H_strato = R * 270.65 / G0;
        double P_51 = P_47 * std::exp(-(51000.0 - 47000.0) / H_strato);
        
        if (altitude <= 71000.0) {
            // 51-71 km: Gradient layer (L = -0.0028 K/m)
            double L = -0.0028;
            double T = getTemperature(altitude);
            return P_51 * std::pow(T / 270.65, -G0 / (L * R));
        }
        
        // P di 71 km
        double P_71 = P_51 * std::pow(214.65 / 270.65, -G0 / (-0.0028 * R));
        
        if (altitude <= 86000.0) {
            // 71-86 km: Gradient layer (L = -0.002 K/m)
            double L = -0.002;
            double T = getTemperature(altitude);
            return P_71 * std::pow(T / 214.65, -G0 / (L * R));
        }
        
        // >86 km: Exponential decay dari 86 km
        double P_86 = P_71 * std::pow(186.65 / 214.65, -G0 / (-0.002 * R));
        double H_thermo = R * 186.65 / G0;
        return P_86 * std::exp(-(altitude - 86000.0) / H_thermo);
    }
    
    // Density sebagai fungsi ketinggian (dari persamaan gas ideal)
    static double getDensity(double altitude) {
        if (altitude < 0) altitude = 0;
        double P = getPressure(altitude);
        double T = getTemperature(altitude);
        double rho = P / (R * T);
        // Clamp density minimum (near-vacuum di ketinggian sangat tinggi)
        return (rho > 1e-12) ? rho : 0.0;
    }
    
    // Density dengan model eksponensial sederhana (aproksimasi cepat)
    static double getDensitySimple(double altitude) {
        double H = 8400.0;  // Scale height [m]
        return RHO0_SL * std::exp(-altitude / H);
    }
    
    // Kecepatan suara sebagai fungsi temperatur
    static double getSpeedOfSound(double altitude) {
        double T = getTemperature(altitude);
        return std::sqrt(GAMMA * R * T);
    }
    
    // ============ ANGIN (SEDERHANA) ============
    
    // Model angin sederhana (bisa dikembangkan)
    static Vector3D getWindVelocity(double altitude) {
        // Angin baratan (westerlies) di lintang menengah
        double windSpeed = 10.0;  // m/s, sederhana
        
        // Angin bertambah dengan ketinggian sampai jet stream
        if (altitude < 11000) {
            windSpeed = 5.0 + altitude / 2000.0;  // Naik linear
        } else if (altitude < 20000) {
            windSpeed = 30.0;  // Jet stream
        } else {
            windSpeed = 20.0;  // Menurun lagi
        }
        
        return Vector3D(windSpeed, 0, 0);  // Angin ke arah X positif
    }
    
    // ============ UTILITAS ============
    
    // Mendapatkan semua data lingkungan sekaligus
    struct EnvironmentData {
        double altitude;
        double gravity;
        double temperature;
        double pressure;
        double density;
        double speedOfSound;
        Vector3D wind;
    };
    
    static EnvironmentData getEnvironmentData(double altitude) {
        EnvironmentData data;
        data.altitude = altitude;
        data.gravity = getGravity(altitude);
        data.temperature = getTemperature(altitude);
        data.pressure = getPressure(altitude);
        data.density = getDensity(altitude);
        data.speedOfSound = getSpeedOfSound(altitude);
        data.wind = getWindVelocity(altitude);
        return data;
    }
    
    // Untuk debugging
    static void printEnvironmentData(double altitude) {
        auto data = getEnvironmentData(altitude);
        std::cout << "===== ENVIRONMENT at " << altitude << " m =====\n";
        std::cout << "Gravity: " << data.gravity << " m/s²\n";
        std::cout << "Temperature: " << data.temperature - 273.15 << " °C\n";
        std::cout << "Pressure: " << data.pressure / 100.0 << " hPa\n";
        std::cout << "Density: " << data.density << " kg/m³\n";
        std::cout << "Speed of sound: " << data.speedOfSound << " m/s\n";
        std::cout << "Wind: " << data.wind << " m/s\n";
        std::cout << "================================\n";
    }
};

#endif