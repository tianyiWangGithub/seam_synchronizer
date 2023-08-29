import os

# Get the current directory
current_directory = os.path.dirname(os.path.realpath(__file__))

# Define the path to the results folder
results_path = os.path.join(current_directory, "..", "results/validation_seam_success_rate/B")

# Define the subfolder names
subfolder_names = ["B_100", "B_105", "B_110", "B_115", "B_120", "B_125", "B_130", "B_135", "B_140", "B_145"]

# Read success rates from subfolders
success_rates = []
for subfolder in subfolder_names:
    subfolder_path = os.path.join(results_path, subfolder)
    success_rate_file_path = os.path.join(subfolder_path, "success_rate.txt")
    if os.path.exists(success_rate_file_path):
        with open(success_rate_file_path, "r") as success_rate_file:
            line = success_rate_file.readline().strip()
            success_rates.append(line)

# Write success rates to success_rate.txt in B folder
output_path = os.path.join(results_path, "success_rate.txt")
with open(output_path, "w") as output_file:
    for success_rate in success_rates:
        output_file.write(f"{success_rate}\n")
