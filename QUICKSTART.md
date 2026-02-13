# Quick Start Guide

## Installation

### Option 1: Header-Only Library
Simply copy `include/aviation_formulary/aviation_formulary.hpp` to your project and include it:

```cpp
#include "aviation_formulary/aviation_formulary.hpp"
using namespace aviation;
```

### Option 2: Build with CMake
```bash
# Clone or extract the library
cd av_form

# Build
mkdir build && cd build
cmake ..
cmake --build . --config Release

# Run tests
ctest -C Release

# Run example
./examples/Release/aviation_example  # Windows
./examples/aviation_example          # Linux/Mac
```

## 5-Minute Tutorial

### 1. Calculate Distance
```cpp
#include "aviation_formulary/aviation_formulary.hpp"
using namespace aviation;

// Create two points (degrees, West/North positive)
LatLon lax = LatLon::from_degrees(33.95, 118.4);   // Los Angeles
LatLon jfk = LatLon::from_degrees(40.633, 73.783); // New York

// Calculate distance
double dist_rad = distance(lax, jfk);
double dist_nm = rad_to_nm(dist_rad);
std::cout << "Distance: " << dist_nm << " nautical miles\n";
// Output: Distance: 2143.726 nautical miles
```

### 2. Calculate Bearing
```cpp
// Get initial bearing from LAX to JFK
double bearing_rad = initial_bearing(lax, jfk);
double bearing_deg = rad_to_deg(bearing_rad);
std::cout << "Bearing: " << bearing_deg << " degrees\n";
// Output: Bearing: 65.892 degrees
```

### 3. Find Destination Point
```cpp
// Start at a point, fly 500 nm on bearing 060°
LatLon start = LatLon::from_degrees(45.0, 10.0);
double bearing = deg_to_rad(60.0);
double distance = nm_to_rad(500.0);

LatLon dest = destination_point(start, bearing, distance);
std::cout << "Destination: " << dest.lat_degrees() << "°N, "
          << dest.lon_degrees() << "°W\n";
```

### 4. Check If Off Course
```cpp
LatLon planned_start = LatLon::from_degrees(34.0, 120.0);
LatLon planned_end = LatLon::from_degrees(40.0, 75.0);
LatLon current_pos = LatLon::from_degrees(35.0, 118.0);

double xtd = cross_track_distance(planned_start, planned_end, current_pos);
double xtd_nm = rad_to_nm(xtd);

if (xtd > 0) {
    std::cout << "Right of course by " << xtd_nm << " nm\n";
} else {
    std::cout << "Left of course by " << -xtd_nm << " nm\n";
}
```

### 5. Find Intersection Point
```cpp
// Two VOR radials intersecting
LatLon vor1 = LatLon::from_degrees(42.6, 117.866);
LatLon vor2 = LatLon::from_degrees(44.84, 117.806);

double radial1 = deg_to_rad(51.0);
double radial2 = deg_to_rad(137.0);

IntersectionResult result = intersection(vor1, radial1, vor2, radial2);

if (result.exists && !result.ambiguous) {
    std::cout << "Fix at: " << result.point.lat_degrees() << "°N, "
              << result.point.lon_degrees() << "°W\n";
}
```

## Important: Sign Convention

**North latitudes and West longitudes are POSITIVE**
**South latitudes and East longitudes are NEGATIVE**

### Examples
```cpp
// New York (40.7°N, 74.0°W)
LatLon nyc = LatLon::from_degrees(40.7, 74.0);

// Tokyo (35.7°N, 139.7°E) - Note: East is negative!
LatLon tokyo = LatLon::from_degrees(35.7, -139.7);

// Sydney (33.9°S, 151.2°E) - Both negative!
LatLon sydney = LatLon::from_degrees(-33.9, -151.2);

// São Paulo (23.5°S, 46.6°W) - South negative, West positive
LatLon saopaulo = LatLon::from_degrees(-23.5, 46.6);
```

## Common Pitfalls

### 1. Unit Confusion
```cpp
// ❌ WRONG: Mixing degrees and radians
double lat = 45.0;  // This is 45 radians, not 45 degrees!
LatLon p(lat, lon);

// ✅ CORRECT: Use from_degrees
LatLon p = LatLon::from_degrees(45.0, 10.0);

// ✅ OR: Convert explicitly
LatLon p(deg_to_rad(45.0), deg_to_rad(10.0));
```

### 2. Sign Convention
```cpp
// ❌ WRONG: Tokyo (35.7°N, 139.7°E)
LatLon tokyo = LatLon::from_degrees(35.7, 139.7);  // Treats as West!

// ✅ CORRECT: East is negative
LatLon tokyo = LatLon::from_degrees(35.7, -139.7);
```

### 3. Distance Units
```cpp
// ❌ WRONG: distance() returns radians, not nautical miles
double dist = distance(p1, p2);
std::cout << dist << " nm";  // Wrong! This is radians

// ✅ CORRECT: Convert to desired units
double dist_rad = distance(p1, p2);
double dist_nm = rad_to_nm(dist_rad);
std::cout << dist_nm << " nm";
```

## Complete Example

```cpp
#include "aviation_formulary/aviation_formulary.hpp"
#include <iostream>
#include <iomanip>

using namespace aviation;

int main() {
    std::cout << std::fixed << std::setprecision(2);
    
    // Define waypoints
    LatLon waypoint1 = LatLon::from_degrees(34.0, 120.0);
    LatLon waypoint2 = LatLon::from_degrees(40.0, 75.0);
    
    // Calculate leg information
    double dist = rad_to_nm(distance(waypoint1, waypoint2));
    double bearing = rad_to_deg(initial_bearing(waypoint1, waypoint2));
    
    std::cout << "Flight Plan:\n";
    std::cout << "From: " << waypoint1.lat_degrees() << "°N, "
              << waypoint1.lon_degrees() << "°W\n";
    std::cout << "To:   " << waypoint2.lat_degrees() << "°N, "
              << waypoint2.lon_degrees() << "°W\n";
    std::cout << "Distance: " << dist << " nm\n";
    std::cout << "Initial bearing: " << bearing << "°\n\n";
    
    // Calculate 25%, 50%, 75% waypoints
    std::cout << "Waypoints:\n";
    for (double f : {0.25, 0.50, 0.75}) {
        LatLon wp = intermediate_point(waypoint1, waypoint2, f);
        std::cout << (int)(f*100) << "% : "
                  << wp.lat_degrees() << "°N, "
                  << wp.lon_degrees() << "°W\n";
    }
    
    return 0;
}
```

## Next Steps

1. **Read the full documentation**: See `README.md` for complete API reference
2. **Run the tests**: `ctest -C Release` to see all test cases
3. **Study the examples**: `examples/example.cpp` shows 8 different use cases
4. **Explore the formulary**: Read Ed Williams' original documentation for formulas

## Getting Help

- Check the test cases in `tests/test_aviation_formulary.cpp` for usage examples
- Review the inline documentation in `aviation_formulary.hpp`
- Compare with worked examples from Ed Williams' formulary

## License

This implementation is based on publicly available mathematical formulas from Ed Williams' Aviation Formulary. Use freely for educational and practical applications.
