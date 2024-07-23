import os
from setuptools import find_packages, setup

package_name = 'ff_commands_publisher'

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['bags/', 'launch']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Davide De Benedittis',
    maintainer_email='davide.debenedittis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_inspector = ff_commands_publisher.bag_inspector:main',
            'bag_recorder = ff_commands_publisher.bag_recorder:main',
            'ff_commands_publisher_node = ff_commands_publisher.ff_commands_publisher_node:main',
        ],
    },
)
