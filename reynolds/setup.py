import os

from setuptools import find_packages, setup

package_name = 'reynolds'


def generate_data_files(src_dir, dest_prefix):
    """Recursively find all files in src_dir and create data_files entries."""
    data_files = []
    if os.path.exists(src_dir):
        for root, dirs, files in os.walk(src_dir):
            if files:  # Only add directories that contain files
                # Convert source path to destination path
                rel_path = os.path.relpath(root, '.')
                dest_path = os.path.join(dest_prefix, rel_path)
                file_paths = [os.path.join(root, f) for f in files]
                data_files.append((dest_path, file_paths))
    return data_files


launch_data_files = generate_data_files('launch', f'share/{package_name}')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + launch_data_files, # type: ignore
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='148395425+vexlexroy@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'boid_calculator = reynolds.boid_calculator:main',
            'boid_calculator_decent = reynolds.boid_calculator_decent:main',
            'neighbor_alocator = reynolds.neighbor_alocator:main',
            'goal_point_selector = reynolds.goal_point_selector:main'
        ],
    },
)
