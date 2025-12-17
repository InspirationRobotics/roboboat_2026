from GNC.Nav_Core import map

test_map = map.Map()
# test_result = test_map.load_map_config("Test_Scripts/GNC_Tests/Nav_Tests/test_map_config_file.json")
# print(test_result)

point1 = (40.748817, -73.985428)  # New York City
point2 = (34.052235, -118.243683)  # Los Angeles

test_heading = test_map.calculate_heading_to_waypoint(point1, point2)
test_midpoint = test_map.calculate_midpoint(point1, point2)
print(f"[TEST RESULTS] Heading : {test_heading}, Midpoint : {test_midpoint}")
