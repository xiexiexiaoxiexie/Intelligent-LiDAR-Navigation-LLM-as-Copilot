import json

def clear_non_empty_lists(json_file):
    # Read the JSON file
    with open(json_file, 'r') as file:
        data = json.load(file)
    
    # Clear non-empty lists
    for key, value in data.items():
        if value:  # Check if the list is not empty
            data[key] = []  # Clear the list
    
    # Write the modified data back to the JSON file
    with open(json_file, 'w') as file:
        json.dump(data, file, indent=4)

def clean_up_experience(json_file):
    # Read the JSON file
    with open(json_file, 'r') as file:
        data = json.load(file)
    
    # Iterate over items and clean up "experience" list
    for key in data:
        if "experience" in data[key] and data[key]["experience"]:
            data[key]["experience"] = []  # Clear the "experience" list

    # Write the modified data back to the JSON file
    with open(json_file, 'w') as file:
        json.dump(data, file, indent=4)

# Specify the paths to your JSON files
json_file_path1 = '../case1/experience_2floor.json'  # Replace with the path to your list file
json_file_path2 = '../case1/experience_2_openai.json'  # Replace with the path to your "experience" file

# Call functions
clear_non_empty_lists(json_file_path1)
clean_up_experience(json_file_path2)
