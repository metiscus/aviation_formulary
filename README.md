# Aviation Formulary C++ Library

A modern, robust C++17 header-only library implementing Ed Williams' Aviation Formulary V1.47 for aviation calculations on a spherical earth model.

## Features

### Navigation
- **Great Circle Navigation**
  - Distance calculations using haversine formula
  - Initial bearing/course between points
  - Destination point from start, bearing, and distance
  - Intermediate points along great circle routes
  - Cross-track and along-track distance
  - Intersection of two great circle radials
  - Great circle crossing of parallels
  - Maximum latitude (Clairaut's formula)

- **Rhumb Line (Loxodrome) Navigation**
  - Distance along rhumb lines
  - Constant course calculations
  - Destination point along rhumb line

- **Local Flat Earth Approximation**
  - Fast calculations for short distances
  - Local Cartesian coordinate conversions

### Atmosphere & Altimetry
- **Standard Atmosphere (ISA)**
  - Temperature, pressure, density at altitude
  - Temperature/pressure/density ratios
  - Speed of sound calculations

- **Altitude Calculations**
  - Pressure altitude
  - Density altitude
  - True altitude corrections

- **Humidity**
  - Dewpoint from temperature and RH
  - Relative humidity calculations
  - Vapor pressure

### Performance
- **Airspeed Conversions**
  - TAS/CAS/EAS/IAS conversions
  - Mach number calculations
  - Compressibility corrections

- **Wind Triangles**
  - Wind correction angle
  - Ground speed calculations
  - Headwind/crosswind components

- **Turn Calculations**
  - Turn radius and rate
  - Bank angle for desired rate
  - Load factor
  - Pivotal altitude

### Utilities
- **Comprehensive Unit Conversions**
  - Distance (nm, km, sm, ft, m)
  - Speed (kts, mph, kph, m/s)
  - Temperature (°C, °F, K)
  - Pressure (mb, inHg, hPa)
  - Volume and mass

- **Miscellaneous**
  - Distance to horizon
  - Bellamy's drift formula

### Robustness
- **Result<T> Error Handling** (exception-free)
- Numerically stable across all edge cases
- Comprehensive stability testing (73 tests)
- Safe trigonometric functions with clamping
- Handles edge cases (poles, antipodal points, etc.)

## Building

### Requirements
- CMake 3.14 or higher
- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- Internet connection (for downloading Google Test)

### Build Instructions

```bash
mkdir build
cd build
cmake ..
cmake --build .
ctest  # Run tests
```

### Windows (PowerShell)
```powershell
mkdir build
cd build
cmake ..
cmake --build . --config Release
ctest -C Release
```

## Usage

### Basic Example

```cpp
#include "aviation_formulary/aviation_formulary.hpp"
#include <iostream>

using namespace aviation;

int main() {
    // Define two points: LAX and JFK
    auto lax_result = LatLon::from_degrees(33.95, 118.4);  // N/W positive
    auto jfk_result = LatLon::from_degrees(40.633, 73.783);
    
    if (!lax_result.is_ok() || !jfk_result.is_ok()) {
        std::cerr << "Invalid coordinates\n";
        return 1;
    }
    
    LatLon lax = lax_result.value;
    LatLon jfk = jfk_result.value;
    
    // Calculate distance
    double dist_rad = distance(lax, jfk);
    double dist_nm = rad_to_nm(dist_rad);
    std::cout << "Distance: " << dist_nm << " nautical miles\n";
    
    // Calculate initial bearing
    auto bearing_result = initial_bearing(lax, jfk);
    if (bearing_result.is_ok()) {
        double bearing_deg = rad_to_deg(bearing_result.value);
        std::cout << "Initial bearing: " << bearing_deg << " degrees\n";
    }
    
    // Find point 40% along the route
    auto midpoint_result = intermediate_point(lax, jfk, 0.4);
    if (midpoint_result.is_ok()) {
        std::cout << "Midpoint: " << midpoint_result.value.lat_degrees() << "°N, "
                  << midpoint_result.value.lon_degrees() << "°W\n";
    }
    
    return 0;
}
```

### Atmosphere & Altimetry

```cpp
// Standard atmosphere
double temp_c = isa_temperature_c(10000.0);  // ISA temp at 10,000 ft
double pressure = isa_pressure_mb(18000.0);  // Pressure at FL180

// Density altitude (important for performance)
double da = density_altitude(5000.0, 30.0);  // PA 5000 ft, 30°C
std::cout << "Density altitude: " << da << " ft\n";
```

### Airspeed Conversions

```cpp
// TAS from CAS at altitude
double tas = tas_from_cas_isa(200.0, 25000.0);  // 200 CAS at FL250

// Mach number
double temp_k = isa_temperature_k(35000.0);
double mach = mach_from_tas(450.0, temp_k);
```

### Wind Triangle

```cpp
// Calculate wind correction angle
double track = deg_to_rad(270.0);   // Westbound
double wind_dir = deg_to_rad(300.0);  // Wind from 300°
double wind_speed = 35.0;  // 35 knots
double tas = 150.0;

auto wca_result = wind_correction_angle(wind_speed, wind_dir, track, tas);
if (wca_result.is_ok()) {
    double heading = track + wca_result.value;
    std::cout << "Heading to fly: " << rad_to_deg(heading) << "°\n";
}

// Headwind/crosswind for runway
double hw = headwind_component(20.0, deg_to_rad(330.0), deg_to_rad(270.0));
double xw = crosswind_component(20.0, deg_to_rad(330.0), deg_to_rad(270.0));
```

### Turn Calculations

```cpp
// Bank angle for standard rate turn
auto bank_result = standard_rate_bank_angle(120.0);  // 120 kts

// Turn radius
auto radius_result = turn_radius_ft(150.0, deg_to_rad(30.0));

// Pivotal altitude for eights-on-pylons
double pa = pivotal_altitude(100.0);  // 100 kts ground speed
```

## Sign Convention

Following the formulary's convention:
- **North latitudes** and **West longitudes** are **positive**
- **South latitudes** and **East longitudes** are **negative**

Example:
- New York (40.7°N, 74.0°W): `LatLon::from_degrees(40.7, 74.0)`
- Tokyo (35.7°N, 139.7°E): `LatLon::from_degrees(35.7, -139.7)`
- Sydney (33.9°S, 151.2°E): `LatLon::from_degrees(-33.9, -151.2)`

## Units

- All angles internally use **radians**
- Convenient conversion functions provided:
  - `deg_to_rad()` / `rad_to_deg()`
  - `nm_to_rad()` / `rad_to_nm()`
- `LatLon::from_degrees()` accepts degrees
- `lat_degrees()` / `lon_degrees()` return degrees

## Testing

The library includes comprehensive tests based on worked examples from the Aviation Formulary:

```bash
cd build
ctest --output-on-failure
```

Test coverage includes:
- **Great Circle Navigation tests (21 tests)**: All navigation functions
- **New Module tests (32 tests)**: Atmosphere, airspeed, wind, turns, etc.
- **Numerical stability tests (20 tests)**: Edge cases, poles, antipodal points
- **Total: 73 tests, 100% passing**

### Numerical Stability
The library has been extensively tested for numerical stability. See [NUMERICAL_STABILITY.md](NUMERICAL_STABILITY.md) for details.

## API Reference

### Error Handling

The library uses a `Result<T>` pattern for error handling (no exceptions):

```cpp
template<typename T>
struct Result {
    T value;
    ErrorCode error;
    
    bool is_ok() const noexcept;       // Check if successful
    T value_or(T default_val) const;   // Get value or default
    operator bool() const noexcept;    // Implicit bool conversion
};
```

For performance-critical code where you know inputs are valid, use `_unchecked` variants:
```cpp
LatLon p = LatLon::from_degrees_unchecked(45.0, 90.0);  // No validation
double b = initial_bearing_unchecked(p1, p2);  // Returns double directly
```

### Header Organization

```
include/aviation_formulary/
├── aviation_formulary.hpp        # Main umbrella header
├── core/
│   ├── constants.hpp             # Mathematical/physical constants
│   ├── types.hpp                 # Result<T>, ErrorCode, LatLon, etc.
│   └── math.hpp                  # Safe trig, unit conversions
├── navigation/
│   ├── great_circle.hpp          # Great circle navigation
│   ├── rhumb_line.hpp            # Constant course navigation
│   └── local_flat.hpp            # Flat earth approximations
├── atmosphere/
│   ├── standard_atmosphere.hpp   # ISA calculations
│   ├── altimetry.hpp             # Altitude calculations
│   └── humidity.hpp              # Dewpoint, RH, vapor pressure
├── performance/
│   ├── airspeed.hpp              # TAS/CAS/EAS/Mach
│   ├── wind.hpp                  # Wind triangles
│   └── turns.hpp                 # Turn calculations
└── utilities/
    ├── conversions.hpp           # Unit conversions
    └── miscellaneous.hpp         # Horizon, Bellamy, etc.
```

### Types

#### `LatLon`
Represents a latitude/longitude point in radians.

**Constructors:**
```cpp
LatLon(double lat_rad, double lon_rad)
static LatLon from_degrees(double lat_deg, double lon_deg)
```

**Methods:**
- `double lat_degrees() const` - Get latitude in degrees
- `double lon_degrees() const` - Get longitude in degrees
- `bool is_pole() const` - Check if point is at a pole

### Functions

#### Distance Calculations
```cpp
double distance(const LatLon& p1, const LatLon& p2)
double distance_cosine(const LatLon& p1, const LatLon& p2)  // Alternative
```

#### Bearing
```cpp
double initial_bearing(const LatLon& p1, const LatLon& p2)
```

#### Navigation
```cpp
LatLon destination_point(const LatLon& start, double bearing, double distance)
LatLon intermediate_point(const LatLon& p1, const LatLon& p2, double fraction)
```

#### Cross-Track
```cpp
double cross_track_distance(const LatLon& a, const LatLon& b, const LatLon& d)
double along_track_distance(const LatLon& a, const LatLon& b, const LatLon& d)
```

#### Advanced
```cpp
double max_latitude(const LatLon& point, double bearing)
IntersectionResult intersection(const LatLon& p1, double brng1, 
                                const LatLon& p2, double brng2)
ParallelCrossingResult crossing_parallels(const LatLon& p1, const LatLon& p2, 
                                         double lat3)
```

## Implementation Notes

- Earth is modeled as a sphere with radius 3440.065 nm (6371 km)
- Haversine formula used for better numerical stability at short distances
- Safe trigonometric functions prevent domain errors from rounding
- Proper modulo implementation ensures correct sign convention
- Special handling for edge cases (poles, meridians, antipodal points)

## References

- [Ed Williams' Aviation Formulary V1.47](http://edwilliams.org/avform.htm)

## License

This implementation is based on publicly available mathematical formulas from Ed Williams' Aviation Formulary. See [LICENSE](LICENSE) for details.

## Version History

- **v2.0.0** - Complete implementation of Aviation Formulary V1.47
  - Added rhumb line navigation
  - Added standard atmosphere & altimetry
  - Added airspeed conversions (TAS/CAS/EAS/Mach)
  - Added wind triangle calculations
  - Added humidity functions
  - Added turn calculations
  - Added comprehensive unit conversions
  - Implemented Result<T> error handling (exception-free)
  - Reorganized into modular header structure
  - 73 tests (up from 41)

- **v1.0.0** - Initial version (Unreleased)
  - Great circle navigation functions
  - Basic numerical stability
