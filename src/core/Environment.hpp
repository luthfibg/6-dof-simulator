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
    
    // Temperatur sebagai fungsi ketinggian (model ISA berlapis)
    static double getTemperature(double altitude) {
        if (altitude <= TROPOPAUSE) {
            // Troposfer: T = T0 - L * h
            return T0_SL - LAPSE_RATE * altitude;
        } 
        else if (altitude <= STRATOPAUSE) {
            // Tropopause (isotermal)
            return T0_SL - LAPSE_RATE * TROPOPAUSE;  // ~216.65K
        }
        else {
            // Stratosfer atas (sederhana, bisa diperluas)
            return 216.65;  // Konstan untuk simulasi awal
        }
    }
    
    // Tekanan sebagai fungsi ketinggian
    static double getPressure(double altitude) {
        if (altitude <= TROPOPAUSE) {
            // Troposfer: P = P0 * (T/T0)^(g/(L*R))
            double T = getTemperature(altitude);
            double exponent = G0 / (LAPSE_RATE * R);
            return P0_SL * std::pow(T / T0_SL, exponent);
        }
        else {
            // Tropopause/Stratosfer: model eksponensial
            double T_tropo = getTemperature(TROPOPAUSE);
            double P_tropo = getPressure(TROPOPAUSE);
            double scaleHeight = R * T_tropo / G0;
            return P_tropo * std::exp(-(altitude - TROPOPAUSE) / scaleHeight);
        }
    }
    
    // Density sebagai fungsi ketinggian (dari persamaan gas ideal)
    static double getDensity(double altitude) {
        double P = getPressure(altitude);
        double T = getTemperature(altitude);
        return P / (R * T);
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