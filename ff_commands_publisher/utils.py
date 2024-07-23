import os
from ament_index_python.packages import get_package_share_directory


def get_list_exp_filepaths(exp_name: str) -> list[str]:
    """Return a list of full path to the bag files containing the input bag_name."""
    
    workspace_path = f"{get_package_share_directory('ff_commands_publisher')}/../../../../"
    bags_directory = f"{workspace_path}/bags"

    folders = [f for f in os.listdir(bags_directory) if exp_name in f]
    exp_filepaths = []
    
    for folder in folders:
        folder = f"{bags_directory}/{folder}"
        files = [f for f in os.listdir(folder) if f.endswith(".mcap")]
        if len(files) == 1:
            exp_filepaths.append(f"{folder}/{files[0]}")
        if len(files) > 1:
            raise ValueError(f"Multiple bag files found in {folder}.")
        
    return exp_filepaths

def get_bag_filepath(bag_name: str) -> str:
    """Return the full path to the bag file."""
    
    package_share_directory = get_package_share_directory('ff_commands_publisher')
    bags_directory = f"{package_share_directory}/bags"
    
    if os.path.exists(f"{bags_directory}/{bag_name}/{bag_name}.mcap"):
        return f"{bags_directory}/{bag_name}/{bag_name}.mcap"


    folders = [f for f in os.listdir(bags_directory) if bag_name in f]
    if len(folders) == 1:
        folder = f"{bags_directory}/{folders[0]}"
        files = [f for f in os.listdir(folder) if f.endswith(".mcap")]
        if len(files) == 1:
            return f"{folder}/{files[0]}"
        if len(files) > 1:
            raise ValueError(f"Multiple bag files found in {folder}.")
            
    if len(folders) > 1:
        raise ValueError(f"Multiple folders found with the name {bag_name}.")
        
    raise FileNotFoundError(f"No bag file found for the name {bag_name}.")


def get_full_bag_filename(bag_name: str) -> str:
    """Return the full name of the bag file."""
    
    bag_filepath = get_bag_filepath(bag_name)
    print(bag_filepath)
    
    bag_full_filename = bag_filepath.split('/')[-2].split('/')[-1]
    print(bag_full_filename)
    
    return bag_full_filename
