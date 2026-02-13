#ifndef AVIATION_FORMULARY_CORE_TYPES_HPP
#define AVIATION_FORMULARY_CORE_TYPES_HPP

/**
 * @file types.hpp
 * @brief Core types for aviation calculations
 * 
 * SIGN CONVENTION: Following Ed Williams' Aviation Formulary V1.47
 * - North latitudes are POSITIVE
 * - West longitudes are POSITIVE
 * - South latitudes are NEGATIVE
 * - East longitudes are NEGATIVE
 * 
 * This differs from standard geographic convention (N/E positive).
 * 
 * Based on Ed Williams' Aviation Formulary V1.47
 * https://edwilliams.org/avform.htm
 */

#include "constants.hpp"
#include <cmath>
#include <type_traits>

namespace aviation {

// =============================================================================
// Error Handling Types
// =============================================================================

/**
 * @brief Error codes for aviation calculations
 * 
 * Used with Result<T> for zero-overhead error handling without exceptions.
 */
enum class ErrorCode {
    Success = 0,
    
    // Input validation errors
    InvalidLatitude,        ///< Latitude out of range [-90, 90]
    InvalidLongitude,       ///< Longitude could not be normalized
    InvalidFraction,        ///< Fraction not in range [0, 1]
    InvalidBearing,         ///< Bearing could not be normalized
    InvalidDistance,        ///< Distance is negative
    InvalidSpeed,           ///< Speed is negative or zero when required positive
    InvalidAltitude,        ///< Altitude out of valid range
    InvalidTemperature,     ///< Temperature out of valid range
    InvalidPressure,        ///< Pressure out of valid range
    InvalidAngle,           ///< Angle out of valid range
    InvalidMachNumber,      ///< Mach number out of valid range
    InvalidBankAngle,       ///< Bank angle out of valid range (0-90)
    InvalidHumidity,        ///< Relative humidity not in [0, 1] or [0, 100]
    
    // Geometric edge cases
    AntipodalPoints,        ///< Points are on opposite sides of earth
    IdenticalPoints,        ///< Points are the same (bearing undefined)
    PolePoint,              ///< Point is at a pole (longitude undefined)
    ParallelPaths,          ///< Great circles are parallel (no intersection)
    CoincidentPaths,        ///< Great circles are the same (infinite intersections)
    
    // Calculation errors
    NoSolution,             ///< No mathematical solution exists
    AmbiguousSolution,      ///< Multiple solutions exist
    DivisionByZero,         ///< Would require division by zero
    OutOfRange,             ///< Result outside valid range
    NumericalInstability,   ///< Calculation would be numerically unstable
    
    // Wind-specific errors
    WindExceedsTAS,         ///< Wind speed exceeds true airspeed
    NoWindSolution,         ///< Wind triangle has no solution
    
    // Atmosphere-specific errors
    AboveCeiling,           ///< Altitude above model ceiling
    BelowFloor,             ///< Altitude below model floor
    
    // General errors
    NotImplemented,         ///< Function not yet implemented
    Unknown                 ///< Unknown error
};

/**
 * @brief Get human-readable description of an error code
 */
[[nodiscard]] constexpr const char* error_message(ErrorCode code) noexcept {
    switch (code) {
        case ErrorCode::Success:              return "Success";
        case ErrorCode::InvalidLatitude:      return "Latitude out of range [-90, 90] degrees";
        case ErrorCode::InvalidLongitude:     return "Longitude could not be normalized";
        case ErrorCode::InvalidFraction:      return "Fraction must be in range [0, 1]";
        case ErrorCode::InvalidBearing:       return "Bearing could not be normalized";
        case ErrorCode::InvalidDistance:      return "Distance cannot be negative";
        case ErrorCode::InvalidSpeed:         return "Speed must be positive";
        case ErrorCode::InvalidAltitude:      return "Altitude out of valid range";
        case ErrorCode::InvalidTemperature:   return "Temperature out of valid range";
        case ErrorCode::InvalidPressure:      return "Pressure out of valid range";
        case ErrorCode::InvalidAngle:         return "Angle out of valid range";
        case ErrorCode::InvalidMachNumber:    return "Mach number out of valid range";
        case ErrorCode::InvalidBankAngle:     return "Bank angle must be in range (0, 90) degrees";
        case ErrorCode::InvalidHumidity:      return "Relative humidity out of valid range";
        case ErrorCode::AntipodalPoints:      return "Points are antipodal - route undefined";
        case ErrorCode::IdenticalPoints:      return "Points are identical - bearing undefined";
        case ErrorCode::PolePoint:            return "Point is at a pole - longitude undefined";
        case ErrorCode::ParallelPaths:        return "Great circles are parallel - no intersection";
        case ErrorCode::CoincidentPaths:      return "Great circles coincide - infinite intersections";
        case ErrorCode::NoSolution:           return "No mathematical solution exists";
        case ErrorCode::AmbiguousSolution:    return "Multiple solutions exist";
        case ErrorCode::DivisionByZero:       return "Division by zero";
        case ErrorCode::OutOfRange:           return "Result outside valid range";
        case ErrorCode::NumericalInstability: return "Calculation numerically unstable";
        case ErrorCode::WindExceedsTAS:       return "Wind speed exceeds true airspeed";
        case ErrorCode::NoWindSolution:       return "Wind triangle has no solution";
        case ErrorCode::AboveCeiling:         return "Altitude above atmosphere model ceiling";
        case ErrorCode::BelowFloor:           return "Altitude below atmosphere model floor";
        case ErrorCode::NotImplemented:       return "Function not implemented";
        case ErrorCode::Unknown:              return "Unknown error";
        default:                              return "Unrecognized error code";
    }
}

/**
 * @brief Result type for zero-overhead error handling
 * 
 * Replaces exceptions with a value-or-error pattern.
 * 
 * Usage:
 * @code
 *   Result<double> result = calculate_something(input);
 *   if (result) {
 *       double value = result.value;
 *       // use value
 *   } else {
 *       ErrorCode err = result.error;
 *       const char* msg = result.message();
 *       // handle error
 *   }
 * @endcode
 * 
 * @tparam T The value type on success
 */
template<typename T>
struct Result {
    T value;                ///< The result value (valid only if error == Success)
    ErrorCode error;        ///< Error code (Success if operation succeeded)
    
    /// Default constructor - creates a success result with default value
    constexpr Result() noexcept : value{}, error{ErrorCode::Success} {}
    
    /// Construct a success result with a value
    constexpr Result(T val) noexcept : value{val}, error{ErrorCode::Success} {}
    
    /// Construct an error result
    constexpr Result(ErrorCode err) noexcept : value{}, error{err} {}
    
    /// Construct an error result with a default value
    constexpr Result(T val, ErrorCode err) noexcept : value{val}, error{err} {}
    
    /// Check if the result is successful
    [[nodiscard]] constexpr bool is_ok() const noexcept { 
        return error == ErrorCode::Success; 
    }
    
    /// Check if the result is an error
    [[nodiscard]] constexpr bool is_error() const noexcept { 
        return error != ErrorCode::Success; 
    }
    
    /// Implicit conversion to bool (true if success)
    [[nodiscard]] constexpr explicit operator bool() const noexcept { 
        return is_ok(); 
    }
    
    /// Get error message
    [[nodiscard]] constexpr const char* message() const noexcept {
        return error_message(error);
    }
    
    /// Get value or default if error
    [[nodiscard]] constexpr T value_or(T default_value) const noexcept {
        return is_ok() ? value : default_value;
    }
    
    // Factory methods
    
    /// Create a success result
    [[nodiscard]] static constexpr Result ok(T val) noexcept {
        return Result{val, ErrorCode::Success};
    }
    
    /// Create an error result
    [[nodiscard]] static constexpr Result err(ErrorCode code) noexcept {
        return Result{T{}, code};
    }
    
    /// Create an error result with a fallback value
    [[nodiscard]] static constexpr Result err(ErrorCode code, T fallback) noexcept {
        return Result{fallback, code};
    }
};

// Specialization for void (operations that don't return a value)
template<>
struct Result<void> {
    ErrorCode error;
    
    constexpr Result() noexcept : error{ErrorCode::Success} {}
    constexpr Result(ErrorCode err) noexcept : error{err} {}
    
    [[nodiscard]] constexpr bool is_ok() const noexcept { 
        return error == ErrorCode::Success; 
    }
    
    [[nodiscard]] constexpr bool is_error() const noexcept { 
        return error != ErrorCode::Success; 
    }
    
    [[nodiscard]] constexpr explicit operator bool() const noexcept { 
        return is_ok(); 
    }
    
    [[nodiscard]] constexpr const char* message() const noexcept {
        return error_message(error);
    }
    
    [[nodiscard]] static constexpr Result ok() noexcept {
        return Result{ErrorCode::Success};
    }
    
    [[nodiscard]] static constexpr Result err(ErrorCode code) noexcept {
        return Result{code};
    }
};

// =============================================================================
// Geographic Types
// =============================================================================

/**
 * @brief Latitude/Longitude point
 * 
 * SIGN CONVENTION: Following Ed Williams' Aviation Formulary V1.47
 * - North latitudes are POSITIVE
 * - West longitudes are POSITIVE
 * - South latitudes are NEGATIVE
 * - East longitudes are NEGATIVE
 * 
 * This differs from standard geographic convention (N/E positive).
 * 
 * Internal storage is in RADIANS.
 * 
 * Examples:
 * - New York (40.7°N, 74.0°W): LatLon::from_degrees(40.7, 74.0)
 * - Tokyo (35.7°N, 139.7°E): LatLon::from_degrees(35.7, -139.7)
 * - Sydney (33.9°S, 151.2°E): LatLon::from_degrees(-33.9, -151.2)
 */
struct LatLon {
    double lat;  ///< Latitude in radians, [-PI/2, PI/2]
    double lon;  ///< Longitude in radians, [-PI, PI]
    
    /// Default constructor - creates point at 0,0
    constexpr LatLon() noexcept : lat(0.0), lon(0.0) {}
    
    /// Construct from radians (private use - prefer from_degrees or from_radians)
    constexpr LatLon(double lat_rad, double lon_rad) noexcept 
        : lat(lat_rad), lon(lon_rad) {}
    
    /**
     * @brief Create from degrees with validation
     * 
     * SIGN CONVENTION: North/West positive, South/East negative
     * 
     * @param lat_deg Latitude in degrees [-90, 90]
     * @param lon_deg Longitude in degrees (will be normalized to [-180, 180])
     * @return Result containing LatLon or error
     */
    [[nodiscard]] static Result<LatLon> from_degrees(double lat_deg, double lon_deg) noexcept {
        // Validate latitude
        if (lat_deg < -90.0 - constants::EPS || lat_deg > 90.0 + constants::EPS) {
            return Result<LatLon>::err(ErrorCode::InvalidLatitude);
        }
        
        // Clamp latitude to valid range (handle floating point edge cases)
        if (lat_deg < -90.0) lat_deg = -90.0;
        if (lat_deg > 90.0) lat_deg = 90.0;
        
        double lat_rad = lat_deg * constants::DEG_TO_RAD;
        double lon_rad = lon_deg * constants::DEG_TO_RAD;
        
        // Normalize longitude to [-PI, PI]
        lon_rad = std::fmod(lon_rad + constants::PI, constants::TWO_PI);
        if (lon_rad < 0) lon_rad += constants::TWO_PI;
        lon_rad -= constants::PI;
        
        return Result<LatLon>::ok(LatLon{lat_rad, lon_rad});
    }
    
    /**
     * @brief Create from degrees without validation (unchecked)
     * 
     * Use when you know the values are valid. Slightly faster.
     * 
     * SIGN CONVENTION: North/West positive, South/East negative
     */
    [[nodiscard]] static constexpr LatLon from_degrees_unchecked(double lat_deg, double lon_deg) noexcept {
        return LatLon{lat_deg * constants::DEG_TO_RAD, lon_deg * constants::DEG_TO_RAD};
    }
    
    /**
     * @brief Create from radians with validation
     */
    [[nodiscard]] static Result<LatLon> from_radians(double lat_rad, double lon_rad) noexcept {
        if (lat_rad < -constants::HALF_PI - constants::EPS || 
            lat_rad > constants::HALF_PI + constants::EPS) {
            return Result<LatLon>::err(ErrorCode::InvalidLatitude);
        }
        
        // Clamp latitude
        if (lat_rad < -constants::HALF_PI) lat_rad = -constants::HALF_PI;
        if (lat_rad > constants::HALF_PI) lat_rad = constants::HALF_PI;
        
        // Normalize longitude
        lon_rad = std::fmod(lon_rad + constants::PI, constants::TWO_PI);
        if (lon_rad < 0) lon_rad += constants::TWO_PI;
        lon_rad -= constants::PI;
        
        return Result<LatLon>::ok(LatLon{lat_rad, lon_rad});
    }
    
    /**
     * @brief Create from radians without validation (unchecked)
     */
    [[nodiscard]] static constexpr LatLon from_radians_unchecked(double lat_rad, double lon_rad) noexcept {
        return LatLon{lat_rad, lon_rad};
    }
    
    /// Get latitude in degrees
    [[nodiscard]] constexpr double lat_degrees() const noexcept {
        return lat * constants::RAD_TO_DEG;
    }
    
    /// Get longitude in degrees  
    [[nodiscard]] constexpr double lon_degrees() const noexcept {
        return lon * constants::RAD_TO_DEG;
    }
    
    /// Check if point is at or very close to a pole
    [[nodiscard]] bool is_pole() const noexcept {
        return std::abs(std::cos(lat)) < constants::EPS;
    }
    
    /// Check if point is at north pole
    [[nodiscard]] bool is_north_pole() const noexcept {
        return lat > constants::HALF_PI - constants::EPS;
    }
    
    /// Check if point is at south pole
    [[nodiscard]] bool is_south_pole() const noexcept {
        return lat < -constants::HALF_PI + constants::EPS;
    }
    
    /// Equality comparison (within epsilon)
    [[nodiscard]] bool equals(const LatLon& other, double eps = constants::EPS) const noexcept {
        return std::abs(lat - other.lat) < eps && std::abs(lon - other.lon) < eps;
    }
};

// =============================================================================
// Wind Types
// =============================================================================

/**
 * @brief Wind vector
 * 
 * Represents wind as speed and direction.
 * Direction is the direction the wind is coming FROM (meteorological convention).
 */
struct Wind {
    double speed;      ///< Wind speed (typically in knots)
    double direction;  ///< Wind FROM direction in radians [0, 2*PI)
    
    constexpr Wind() noexcept : speed(0.0), direction(0.0) {}
    constexpr Wind(double spd, double dir) noexcept : speed(spd), direction(dir) {}
    
    /// Create from speed and direction in degrees
    [[nodiscard]] static constexpr Wind from_degrees(double speed, double dir_deg) noexcept {
        return Wind{speed, dir_deg * constants::DEG_TO_RAD};
    }
    
    /// Get direction in degrees
    [[nodiscard]] constexpr double direction_degrees() const noexcept {
        return direction * constants::RAD_TO_DEG;
    }
    
    /// Check if wind is calm (speed below threshold)
    [[nodiscard]] bool is_calm(double threshold = 0.5) const noexcept {
        return speed < threshold;
    }
};

// =============================================================================
// Result Types for Complex Calculations
// =============================================================================

/**
 * @brief Result of intersection calculation
 */
struct IntersectionResult {
    LatLon point;         ///< Intersection point (valid only if exists && !ambiguous)
    ErrorCode error;      ///< Error code
    bool exists;          ///< True if intersection exists
    bool ambiguous;       ///< True if multiple intersections exist
    
    constexpr IntersectionResult() noexcept 
        : point{}, error{ErrorCode::NoSolution}, exists{false}, ambiguous{false} {}
    
    [[nodiscard]] constexpr bool is_ok() const noexcept {
        return error == ErrorCode::Success && exists && !ambiguous;
    }
    
    [[nodiscard]] constexpr explicit operator bool() const noexcept {
        return is_ok();
    }
    
    [[nodiscard]] constexpr const char* message() const noexcept {
        return error_message(error);
    }
};

/**
 * @brief Result of parallel crossing calculation
 */
struct ParallelCrossingResult {
    double lon1;          ///< First crossing longitude (radians)
    double lon2;          ///< Second crossing longitude (radians)
    ErrorCode error;      ///< Error code
    int num_crossings;    ///< Number of crossings (0, 1, or 2)
    
    constexpr ParallelCrossingResult() noexcept 
        : lon1{0.0}, lon2{0.0}, error{ErrorCode::NoSolution}, num_crossings{0} {}
    
    [[nodiscard]] constexpr bool is_ok() const noexcept {
        return error == ErrorCode::Success && num_crossings > 0;
    }
    
    [[nodiscard]] constexpr explicit operator bool() const noexcept {
        return is_ok();
    }
};

/**
 * @brief Result of wind triangle calculation
 */
struct WindTriangleResult {
    double heading;       ///< Heading to fly (radians)
    double ground_speed;  ///< Resulting ground speed
    double wca;           ///< Wind correction angle (radians), positive = right
    ErrorCode error;      ///< Error code
    
    constexpr WindTriangleResult() noexcept 
        : heading{0.0}, ground_speed{0.0}, wca{0.0}, error{ErrorCode::Success} {}
    
    [[nodiscard]] constexpr bool is_ok() const noexcept {
        return error == ErrorCode::Success;
    }
    
    [[nodiscard]] constexpr explicit operator bool() const noexcept {
        return is_ok();
    }
};

/**
 * @brief Result of atmospheric calculation
 */
struct AtmosphereResult {
    double temperature;   ///< Temperature (Kelvin)
    double pressure;      ///< Pressure (Pa or mb depending on context)
    double density;       ///< Density (kg/m³)
    ErrorCode error;      ///< Error code
    
    constexpr AtmosphereResult() noexcept 
        : temperature{0.0}, pressure{0.0}, density{0.0}, error{ErrorCode::Success} {}
    
    [[nodiscard]] constexpr bool is_ok() const noexcept {
        return error == ErrorCode::Success;
    }
    
    [[nodiscard]] constexpr explicit operator bool() const noexcept {
        return is_ok();
    }
};

} // namespace aviation

#endif // AVIATION_FORMULARY_CORE_TYPES_HPP
