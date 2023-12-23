import math
import matplotlib.pyplot as plt
from geopy.distance import geodesic

def calculate_vector(wp1, wp2):
    return [wp2[0] - wp1[0], wp2[1] - wp1[1]]

def normalize_vector(vector):
    magnitude = math.sqrt(vector[0]**2 + vector[1]**2)
    return [vector[0] / magnitude, vector[1] / magnitude]

def calculate_perpendicular(vector):
    return [-vector[1], vector[0]]

def shift_waypoint(waypoint, perpendicular, distance_meters):
    # Convert latitude and longitude from degrees to meters
    lat_meters = distance_meters / 111000
    lon_meters = distance_meters / (111000 * math.cos(math.radians(waypoint[0])))

    # Calculate the shift
    shifted = [waypoint[0] + lat_meters * perpendicular[0], waypoint[1] + lon_meters * perpendicular[1]]
    return shifted

# Example Usage:


# Specify the index of the waypoint from where you want to shift all subsequent waypoints
start_idx_to_shift = 49  # Change this to the desired index

shift_distance_meters = 4.0  # Change this to the desired shift distance in meters

shifted_waypoints = []

for i in range(len(waypoints) - 1):
    vector = calculate_vector(waypoints[i], waypoints[i + 1])
    normalized_vector = normalize_vector(vector)
    perpendicular = calculate_perpendicular(normalized_vector)

    if i >= start_idx_to_shift:
        # Shifting waypoints starting from the specified index by 4 meters perpendicular to the path
        shifted = shift_waypoint(waypoints[i], perpendicular, shift_distance_meters)
    else:
        shifted = waypoints[i]

    shifted_waypoints.append(shifted)

# Add the last waypoint
shifted_waypoints.append(waypoints[-1])

# Plotting
original_lats, original_lons = zip(*waypoints)
shifted_lats, shifted_lons = zip(*shifted_waypoints)

plt.figure(figsize=(12, 8))

# Plot original waypoints
plt.plot(original_lons, original_lats, marker='o', label='Original Waypoints', linestyle='-', color='b')

# Plot shifted waypoints
plt.plot(shifted_lons, shifted_lats, marker='x', label=f'Shifted Waypoints from Index {start_idx_to_shift}', linestyle='--', color='r')

plt.title('Original Waypoints and Shifted Waypoints')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.legend()
plt.grid(True)
plt.show()
