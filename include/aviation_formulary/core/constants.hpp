#ifndef AVIATION_FORMULARY_CORE_CONSTANTS_HPP
#define AVIATION_FORMULARY_CORE_CONSTANTS_HPP

/**
 * @file constants.hpp
 * @brief Mathematical and physical constants for aviation calculations
 * 
 * Based on Ed Williams' Aviation Formulary V1.47
 * https://edwilliams.org/avform.htm
 */

namespace aviation {
namespace constants {

// =============================================================================
// Mathematical Constants
// =============================================================================

/// Pi with extended precision
constexpr double PI = 3.141592653589793238462643383279502884;

/// 2 * Pi
constexpr double TWO_PI = 2.0 * PI;

/// Pi / 2
constexpr double HALF_PI = PI / 2.0;

/// Degrees to radians conversion factor
constexpr double DEG_TO_RAD = PI / 180.0;

/// Radians to degrees conversion factor
constexpr double RAD_TO_DEG = 180.0 / PI;

/// Nautical miles to radians conversion factor (1 nm = 1 minute of arc)
constexpr double NM_TO_RAD = PI / (180.0 * 60.0);

/// Radians to nautical miles conversion factor
constexpr double RAD_TO_NM = (180.0 * 60.0) / PI;

/// Small number for floating point comparisons
constexpr double EPS = 1e-10;

/// Machine epsilon for double precision
constexpr double DOUBLE_EPS = 2.220446049250313e-16;

// =============================================================================
// Earth Constants
// =============================================================================

/// Earth's mean radius in nautical miles (derived from 1 nm = 1 minute of arc)
constexpr double EARTH_RADIUS_NM = 3440.065;

/// Earth's mean radius in kilometers (FAI sphere)
constexpr double EARTH_RADIUS_KM = 6371.0;

/// Earth's mean radius in meters
constexpr double EARTH_RADIUS_M = 6371000.0;

/// Earth's mean radius in statute miles
constexpr double EARTH_RADIUS_SM = 3958.8;

/// Earth's mean radius in feet
constexpr double EARTH_RADIUS_FT = 20902231.0;

// =============================================================================
// Standard Atmosphere Constants (ISA - International Standard Atmosphere)
// =============================================================================

/// Sea level standard pressure in millibars (hPa)
constexpr double ISA_P0_MB = 1013.25;

/// Sea level standard pressure in inches of mercury
constexpr double ISA_P0_INHG = 29.92126;

/// Sea level standard temperature in Celsius
constexpr double ISA_T0_C = 15.0;

/// Sea level standard temperature in Kelvin
constexpr double ISA_T0_K = 288.15;

/// Sea level standard temperature in Fahrenheit
constexpr double ISA_T0_F = 59.0;

/// Sea level standard density in kg/m³
constexpr double ISA_RHO0 = 1.225;

/// Temperature lapse rate in troposphere (°C per 1000 feet)
constexpr double ISA_LAPSE_RATE_C_PER_1000FT = 1.98;

/// Temperature lapse rate in troposphere (°C per meter)
constexpr double ISA_LAPSE_RATE_C_PER_M = 0.0065;

/// Temperature lapse rate in troposphere (K per meter)
constexpr double ISA_LAPSE_RATE_K_PER_M = 0.0065;

/// Tropopause altitude in feet
constexpr double TROPOPAUSE_FT = 36089.24;

/// Tropopause altitude in meters
constexpr double TROPOPAUSE_M = 11000.0;

/// Tropopause temperature in Celsius
constexpr double TROPOPAUSE_TEMP_C = -56.5;

/// Tropopause temperature in Kelvin
constexpr double TROPOPAUSE_TEMP_K = 216.65;

/// Specific gas constant for dry air (J/(kg·K))
constexpr double R_DRY_AIR = 287.05287;

/// Gravitational acceleration (m/s²)
constexpr double G = 9.80665;

/// Pressure exponent for troposphere: g/(R*L) where L is lapse rate
constexpr double PRESSURE_EXPONENT = 5.255876;

/// Density exponent for troposphere
constexpr double DENSITY_EXPONENT = 4.255876;

/// Coefficient for pressure altitude formula (per foot)
constexpr double PRESSURE_ALT_COEFF = 6.8756e-6;

// =============================================================================
// Speed of Sound Constants
// =============================================================================

/// Speed of sound at sea level ISA in knots
constexpr double SPEED_OF_SOUND_SL_KT = 661.4788;

/// Speed of sound at sea level ISA in m/s
constexpr double SPEED_OF_SOUND_SL_MPS = 340.294;

/// Speed of sound coefficient: a = 38.967854 * sqrt(T_kelvin)
constexpr double SPEED_OF_SOUND_COEFF = 38.967854;

// =============================================================================
// Airspeed Constants
// =============================================================================

/// Standard sea level pressure for airspeed calculations (lb/ft²)
constexpr double P0_LBF_SQFT = 2116.22;

/// Ratio of specific heats for air (gamma)
constexpr double GAMMA = 1.4;

/// (gamma - 1) / gamma
constexpr double GAMMA_MINUS_1_OVER_GAMMA = 0.2857142857;

/// gamma / (gamma - 1)
constexpr double GAMMA_OVER_GAMMA_MINUS_1 = 3.5;

/// 2 / (gamma - 1)
constexpr double TWO_OVER_GAMMA_MINUS_1 = 5.0;

/// (gamma - 1) / 2
constexpr double GAMMA_MINUS_1_OVER_2 = 0.2;

/// Constant for CAS/Mach relationship at sea level
constexpr double A0_SQUARED = 661.4788 * 661.4788;

// =============================================================================
// Humidity Constants
// =============================================================================

/// Saturation vapor pressure coefficient A (for Magnus formula)
constexpr double VAPOR_PRESSURE_A = 6.1121;

/// Saturation vapor pressure coefficient B (for Magnus formula over water)
constexpr double VAPOR_PRESSURE_B_WATER = 17.502;

/// Saturation vapor pressure coefficient C (for Magnus formula over water)
constexpr double VAPOR_PRESSURE_C_WATER = 240.97;

/// Saturation vapor pressure coefficient B (for Magnus formula over ice)
constexpr double VAPOR_PRESSURE_B_ICE = 22.587;

/// Saturation vapor pressure coefficient C (for Magnus formula over ice)
constexpr double VAPOR_PRESSURE_C_ICE = 273.86;

// =============================================================================
// Unit Conversion Factors
// =============================================================================

/// Nautical miles to kilometers
constexpr double NM_TO_KM = 1.852;

/// Kilometers to nautical miles
constexpr double KM_TO_NM = 1.0 / 1.852;

/// Nautical miles to statute miles
constexpr double NM_TO_SM = 1.15078;

/// Statute miles to nautical miles
constexpr double SM_TO_NM = 1.0 / 1.15078;

/// Feet to meters
constexpr double FT_TO_M = 0.3048;

/// Meters to feet
constexpr double M_TO_FT = 1.0 / 0.3048;

/// Knots to km/h
constexpr double KT_TO_KPH = 1.852;

/// km/h to knots
constexpr double KPH_TO_KT = 1.0 / 1.852;

/// Knots to mph
constexpr double KT_TO_MPH = 1.15078;

/// mph to knots
constexpr double MPH_TO_KT = 1.0 / 1.15078;

/// Knots to m/s
constexpr double KT_TO_MPS = 0.514444;

/// m/s to knots
constexpr double MPS_TO_KT = 1.0 / 0.514444;

/// Millibars to inches of mercury
constexpr double MB_TO_INHG = 0.02953;

/// Inches of mercury to millibars
constexpr double INHG_TO_MB = 33.8639;

/// US gallons to liters
constexpr double USGAL_TO_L = 3.78541;

/// Liters to US gallons
constexpr double L_TO_USGAL = 1.0 / 3.78541;

/// Imperial gallons to liters
constexpr double IMPGAL_TO_L = 4.54609;

/// Liters to Imperial gallons
constexpr double L_TO_IMPGAL = 1.0 / 4.54609;

/// Pounds to kilograms
constexpr double LB_TO_KG = 0.453592;

/// Kilograms to pounds
constexpr double KG_TO_LB = 1.0 / 0.453592;

// =============================================================================
// Turn Calculation Constants
// =============================================================================

/// Constant for turn radius calculation: radius = V²/(g * tan(bank))
/// In feet and knots: radius_ft = V_kt² * 0.0514444² / (g * tan(bank))
constexpr double TURN_RADIUS_COEFF_FT = 11.26;

/// Rate of turn coefficient: rate = 1091 * tan(bank) / V_kt
constexpr double TURN_RATE_COEFF = 1091.0;

/// Standard rate turn (3 degrees per second)
constexpr double STANDARD_RATE_DEG_PER_SEC = 3.0;

} // namespace constants
} // namespace aviation

#endif // AVIATION_FORMULARY_CORE_CONSTANTS_HPP
