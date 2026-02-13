#include "aviation_formulary/aviation_formulary.hpp"
#include <iostream>
#include <iomanip>

using namespace aviation;

void print_separator() {
    std::cout << "\n" << std::string(70, '=') << "\n\n";
}

int main() {
    std::cout << std::fixed << std::setprecision(3);
    
    std::cout << "Aviation Formulary C++ Library - Example\n";
    std::cout << "Based on Ed Williams' Aviation Formulary V1.47\n";
    
    print_separator();
    
    // Example 1: Distance and bearing
    std::cout << "Example 1: Distance and Bearing (LAX to JFK)\n";
    std::cout << "--------------------------------------------\n";
    
    // Using from_degrees_unchecked for known-valid coordinates
    LatLon lax = LatLon::from_degrees_unchecked(33.0 + 57.0/60.0, 118.0 + 24.0/60.0);
    LatLon jfk = LatLon::from_degrees_unchecked(40.0 + 38.0/60.0, 73.0 + 47.0/60.0);
    
    std::cout << "LAX: " << lax.lat_degrees() << "°N, " 
              << lax.lon_degrees() << "°W\n";
    std::cout << "JFK: " << jfk.lat_degrees() << "°N, " 
              << jfk.lon_degrees() << "°W\n\n";
    
    double dist = distance(lax, jfk);
    auto bearing_result = initial_bearing(lax, jfk);
    double bearing = bearing_result.value_or(0.0);
    
    std::cout << "Distance: " << rad_to_nm(dist) << " nm\n";
    std::cout << "Initial bearing: " << rad_to_deg(bearing) << "°\n";
    
    print_separator();
    
    // Example 2: Waypoint calculation
    std::cout << "Example 2: Waypoint 100nm from LAX on Great Circle to JFK\n";
    std::cout << "-----------------------------------------------------------\n";
    
    LatLon waypoint = destination_point_unchecked(lax, bearing, nm_to_rad(100.0));
    
    std::cout << "Waypoint at 100nm: " << waypoint.lat_degrees() << "°N, "
              << waypoint.lon_degrees() << "°W\n";
    
    print_separator();
    
    // Example 3: Intermediate point
    std::cout << "Example 3: Point 40% along the route\n";
    std::cout << "------------------------------------\n";
    
    auto mid_result = intermediate_point(lax, jfk, 0.4);
    if (mid_result.is_ok()) {
        LatLon mid = mid_result.value;
        std::cout << "40% point: " << mid.lat_degrees() << "°N, "
                  << mid.lon_degrees() << "°W\n";
    } else {
        std::cout << "Could not calculate intermediate point\n";
    }
    
    print_separator();
    
    // Example 4: Cross-track error
    std::cout << "Example 4: Cross-Track Error\n";
    std::cout << "----------------------------\n";
    
    LatLon current = LatLon::from_degrees_unchecked(34.5, 116.5);
    
    std::cout << "Current position: " << current.lat_degrees() << "°N, "
              << current.lon_degrees() << "°W\n";
    
    double xtd = cross_track_distance(lax, jfk, current);
    double atd = along_track_distance(lax, jfk, current);
    
    std::cout << "Cross-track distance: " << rad_to_nm(xtd) << " nm ";
    std::cout << (xtd > 0 ? "(right of course)" : "(left of course)") << "\n";
    std::cout << "Along-track distance: " << rad_to_nm(atd) << " nm\n";
    
    print_separator();
    
    // Example 5: Radial intersection
    std::cout << "Example 5: Radial Intersection\n";
    std::cout << "------------------------------\n";
    
    LatLon reo = LatLon::from_degrees_unchecked(42.600, 117.866);
    LatLon bke = LatLon::from_degrees_unchecked(44.840, 117.806);
    
    std::cout << "REO: " << reo.lat_degrees() << "°N, "
              << reo.lon_degrees() << "°W\n";
    std::cout << "BKE: " << bke.lat_degrees() << "°N, "
              << bke.lon_degrees() << "°W\n\n";
    
    double brng1 = deg_to_rad(51.0);
    double brng2 = deg_to_rad(137.0);
    
    std::cout << "REO radial: " << rad_to_deg(brng1) << "°\n";
    std::cout << "BKE radial: " << rad_to_deg(brng2) << "°\n\n";
    
    IntersectionResult result = intersection(reo, brng1, bke, brng2);
    
    if (result.exists && !result.ambiguous) {
        std::cout << "Intersection: " << result.point.lat_degrees() << "°N, "
                  << result.point.lon_degrees() << "°W\n";
    } else if (result.ambiguous) {
        std::cout << "Intersection is ambiguous\n";
    } else {
        std::cout << "No intersection: " << result.message() << "\n";
    }
    
    print_separator();
    
    // Example 6: Parallel crossing
    std::cout << "Example 6: Great Circle Crossing Parallel\n";
    std::cout << "-----------------------------------------\n";
    
    double parallel = deg_to_rad(36.0);
    
    std::cout << "Finding where LAX-JFK route crosses " 
              << rad_to_deg(parallel) << "°N\n\n";
    
    ParallelCrossingResult crossing = crossing_parallels(lax, jfk, parallel);
    
    if (crossing.num_crossings > 0) {
        std::cout << "Number of crossings: " << crossing.num_crossings << "\n";
        std::cout << "Crossing longitude 1: " << rad_to_deg(crossing.lon1) << "°W\n";
        if (crossing.num_crossings > 1) {
            std::cout << "Crossing longitude 2: " << rad_to_deg(crossing.lon2) << "°W\n";
        }
    } else {
        std::cout << "Route does not cross this parallel\n";
    }
    
    print_separator();
    
    // Example 7: Maximum latitude
    std::cout << "Example 7: Maximum Latitude on Great Circle Route\n";
    std::cout << "-------------------------------------------------\n";
    
    double max_lat = max_latitude(lax, bearing);
    
    std::cout << "LAX latitude: " << lax.lat_degrees() << "°N\n";
    std::cout << "JFK latitude: " << jfk.lat_degrees() << "°N\n";
    std::cout << "Maximum latitude: " << rad_to_deg(max_lat) << "°N\n";
    
    print_separator();
    
    // Example 8: Round trip consistency
    std::cout << "Example 8: Round Trip Verification\n";
    std::cout << "----------------------------------\n";
    
    LatLon start = LatLon::from_degrees_unchecked(45.0, 10.0);
    double out_bearing = deg_to_rad(60.0);
    double out_dist = nm_to_rad(500.0);
    
    LatLon destination = destination_point_unchecked(start, out_bearing, out_dist);
    auto back_bearing_result = initial_bearing(destination, start);
    double back_bearing = back_bearing_result.value_or(0.0);
    double back_dist = distance(destination, start);
    
    std::cout << "Start: " << start.lat_degrees() << "°N, "
              << start.lon_degrees() << "°W\n";
    std::cout << "Outbound: " << rad_to_deg(out_bearing) 
              << "° for " << rad_to_nm(out_dist) << " nm\n";
    std::cout << "Destination: " << destination.lat_degrees() << "°N, "
              << destination.lon_degrees() << "°W\n";
    std::cout << "Return bearing: " << rad_to_deg(back_bearing) << "°\n";
    std::cout << "Return distance: " << rad_to_nm(back_dist) << " nm\n";
    
    print_separator();
    
    std::cout << "All examples completed successfully!\n";
    
    return 0;
}
