import os

# Get the current directory
current_directory = os.path.dirname(os.path.realpath(__file__))

# Define the path to the results folder
results_path = os.path.join(current_directory, "..", "results/validation_seam_computetime/internal_length")

# Define the subfolder names
subfolder_names = ["internal_length_10", "internal_length_20", "internal_length_30", "internal_length_40", "internal_length_50", "internal_length_60", "internal_length_70", "internal_length_80", "internal_length_90", "internal_length_100"]

# Create a list to store average values
averages = []

# Function to calculate average from a list of valid numbers
def calculate_average(numbers):
    if not numbers:
        return 0.0
    return sum(numbers) / len(numbers)

# Loop through each subfolder
for subfolder in subfolder_names:
    subfolder_path = os.path.join(results_path, subfolder)
    
    # Initialize lists to store valid numbers
    valid_numbers_apr = []
    valid_numbers_seam = []
    
    # Loop through each txt file in the subfolder
    for filename in ["compute_time_apr.txt", "compute_time_seam.txt"]:
        file_path = os.path.join(subfolder_path, filename)
        if os.path.exists(file_path):
            with open(file_path, "r") as file:
                lines = file.readlines()
                for line in lines:
                    num_str = line.strip()
                    try:
                        num = float(num_str)
                        # Check if the number has 9 decimal places
                        if round(num, 9) == num:
                            if filename == "compute_time_apr.txt":
                                valid_numbers_apr.append(num)
                            else:
                                valid_numbers_seam.append(num)
                    except ValueError:
                        pass  # Skip invalid numbers
    
    # Calculate the average for this subfolder
    avg_apr = calculate_average(valid_numbers_apr)
    avg_seam = calculate_average(valid_numbers_seam)
    averages.append((avg_seam * 1000000, avg_apr * 1000000))

# Write the averages to data_ct_length.txt
output_path = os.path.join(results_path, "data_ct_length.txt")
with open(output_path, "w") as output_file:
    for avg_apr, avg_seam in averages:
        output_file.write(f"{avg_apr:.9f} {avg_seam:.9f}\n")
