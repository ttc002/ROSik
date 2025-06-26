from setuptools import find_packages, setup

package_name = 'esp_miniBot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='petr',
    maintainer_email='shokalPetr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': ['read_mp3 = esp_miniBot.mp3_file_read:main', 'send_music = esp_miniBot.music_stream:main', 'send_speed = esp_miniBot.espSpeedStreamer:main', 'speaker = esp_miniBot.virtual_speaker:main'
        ],
    },
)
