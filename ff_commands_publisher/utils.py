import os
from ament_index_python.packages import get_package_share_directory


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
