def normalize_vector_(vector):
    #print("\n Waypoint vector : ", vector )
    magnitude = math.sqrt(vector[0]**2 + vector[1]**2)
    return [vector[0] / magnitude, vector[1] / magnitude]

def normalize_vector(vector):
    print("\n Waypoint vector : ", vector)
    magnitude = math.sqrt(vector[0]**2 + vector[1]**2)
    
    # Check for zero magnitude
    if magnitude == 0:
        return vector
