[![Ardupilot Cloud EcoSystem](https://cloud.ardupilot.org/_static/ardupilot_logo.png "Ardupilot Cloud EcoSystem")](https://cloud.ardupilot.org "Ardupilot Cloud EcoSystem") **Drone Engage** is part of Ardupilot Cloud Eco System

------------

![Drone Engage Mavlink Module](resources/de_logo_title.png) 

#Drone Engage Mavlink Module

# What is Drone-Mavlink ?

Drone Mavlink is the most important [Drone-Engage](https://cloud.ardupilot.org "Drone-Engage") plugin.

[Raspberry-pi boards to run Drone-Engage](https://cloud.ardupilot.org/_images/setup1.png "Raspberry-pi boards to run Drone-Engage")


[![https://cloud.ardupilot.org/_images/setup1.png](https://cloud.ardupilot.org/_images/setup1.png "https://cloud.ardupilot.org/_images/setup1.png")](https://cloud.ardupilot.org/_images/setup1.png "https://cloud.ardupilot.org/_images/setup1.png")

------------

# Build Instructions

## Simple Build (Quick Start)

For a quick build with default settings:

```bash
# Clone the repository
git clone https://github.com/DroneEngage/droneengage_mavlink.git
cd droneengage_mavlink

# Build using the provided script
./build.sh

# The executable will be created in bin/de_ardupilot
```

## Advanced Build Options

### Manual Build with Custom Configuration

```bash
# Clone the repository
git clone https://github.com/DroneEngage/droneengage_mavlink.git
cd droneengage_mavlink

# Create build directory
mkdir build && cd build

# Configure with CMake (Debug build - default)
cmake ..

# Or configure for Release build
cmake -DCMAKE_BUILD_TYPE=RELEASE ..

# Build the project
make -j$(nproc)

# Generate packages (optional)
cpack
```

### Build Options

- **Debug Build**: Includes debugging symbols, optimized for development
- **Release Build**: Optimized for production, includes version increment

### Package Generation

The build system can generate multiple package formats:

```bash
cd build

# Generate all package types (.deb, .tar.gz, .sh)
cpack

# Generate specific package type
cpack -G DEB        # Debian package
cpack -G TGZ        # Tar.gz archive  
cpack -G STGZ       # Self-extracting shell script
```

### Package Contents

Generated packages install to:
- `/home/$USER/drone_engage/de_mavlink/de_ardupilot` - Main executable
- `/home/$USER/drone_engage/de_mavlink/de_mavlink.config.module.json` - Configuration file
- `/home/$USER/drone_engage/de_mavlink/template.json` - Template file

### Installing .deb Package

```bash
sudo dpkg -i build/packages/de-mavlink-plugin-*.deb
```

### Build Dependencies

- GCC/G++ compiler
- CMake 3.1+
- pthreads library
- libstdc++6

### Version Management

The project uses automatic version management:
- Major.Minor.Bugfix.Build format
- Build number auto-increments on RELEASE builds
- Version defined in CMakeLists.txt (MAJOR_VERSION, MINOR_VERSION, BUGFIX_VERSION)

------------

[![Ardupilot Cloud EcoSystem](https://cloud.ardupilot.org/_static/ardupilot_logo.png "Ardupilot Cloud EcoSystem")](https://cloud.ardupilot.org "Ardupilot Cloud EcoSystem") **Drone Engage** is part of Ardupilot Cloud Eco System

