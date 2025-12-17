# Define the input and output file paths
input_file = r'Test_Scripts/API_Tests/GPS_Tests/Missions/large_waypoint_test.txt'
output_file = r'Test_Scripts/API_Tests/GPS_Tests/Missions/large_gps_parser_test.txt'

# Open the input file for reading and the output file for writing
with open(input_file, "r") as infile, open(output_file, "w") as outfile:
    # Iterate over each line in the input file with an enumerated loop
    for idx, line in enumerate(infile, start=1):
        # Add the sequential number and "% " before each line
        new_line = f"{idx} % {line}"
        # Write the modified line to the output file
        outfile.write(new_line)

print(f"Lines from {input_file} have been processed and saved to {output_file}.")