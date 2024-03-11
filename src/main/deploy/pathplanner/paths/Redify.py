# -*- coding: utf-8 -*-
"""
Created on Mon Mar 11 13:19:01 2024

@author: ChatGPT
"""

import os
import json

def replace_values_and_rename(json_file_path):
    # Load JSON file
    print(json_file_path)
    with open(json_file_path, 'r') as file:
        data = json.load(file)
    
    # Replace 'x', 'rotation', and 'rotationDegrees' values
    for waypoint in data['waypoints']:
        if 'linkedName' in waypoint and waypoint['linkedName'] is not None:
            waypoint['linkedName'] = waypoint['linkedName'].replace('BLUE', 'RED')
        if 'x' in waypoint['anchor']:
            print(waypoint['anchor']['x'])
            waypoint['anchor']['x'] = 16.54 - waypoint['anchor']['x']
        if  waypoint['nextControl'] is not None and 'x' in waypoint['nextControl']:
            waypoint['nextControl']['x'] = 16.54 - waypoint['nextControl']['x']
        if  waypoint['prevControl'] is not None and 'x' in waypoint['prevControl']:
            waypoint['prevControl']['x'] = 16.54 - waypoint['prevControl']['x']
        if 'rotation' in waypoint and waypoint['rotation'] is not None:
            waypoint['rotation'] = 180 - waypoint['rotation']
            
    for rotationTarget in data['rotationTargets']:
        rotationTarget['rotationDegrees'] = 180 - rotationTarget['rotationDegrees']
    if 'folder' in data and data['folder'] is not None:
        data['folder'] = data['folder'].replace('BLUE', 'RED')
    if 'goalEndState' in data:
        data['goalEndState']['rotation'] = 180- data['goalEndState']['rotation']
    if 'previewStartingState' in data and data['previewStartingState'] is not None:
        if 'rotation' in data['previewStartingState']:
            data['previewStartingState']['rotation'] = 180 - data['previewStartingState']['rotation']
        else:
            data['previewStartingState']['rotation'] = 180
            data['previewStartingState']['velocity'] = 0
    
    # Update filename
    filename = os.path.basename(json_file_path)
    new_filename = filename.replace('BLUE', 'RED')
    new_file_path = os.path.join(os.path.dirname(json_file_path), new_filename)
    
    # Export modified JSON to new file
    with open(new_file_path, 'w') as file:
        json.dump(data, file, indent=4)
    
    print(f"Modified JSON file exported to '{new_filename}'")

def process_files_in_folder(folder_path):
    
    # Iterate over files in the folder
    for filename in os.listdir(folder_path):
        if filename.startswith("BLUE") and filename.endswith(".path"):
            json_file_path = os.path.join(folder_path, filename)
            replace_values_and_rename(json_file_path)

# Example usage
if __name__ == "__main__":
    folder_path = "./"  # Path to the folder containing JSON files
    process_files_in_folder(folder_path)
