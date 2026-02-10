# CMake Debian Package Generation with Auto-Versioning

## Overview
Instructions for adding Debian package generation (.deb) with automatic version increment to CMake projects.

## Step 1: Add CPack Configuration to CMakeLists.txt

Add this at the end of your CMakeLists.txt:

```cmake
# CPack configuration for Debian package generation
set(CPACK_PACKAGE_NAME "YOUR_PROJECT_NAME")
set(CPACK_PACKAGE_VERSION "${__APP__VERSION__}")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libstdc++6") # Add your dependencies
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Your Team Name")
include(CPack)

# Installation configuration
install(TARGETS your_executable_name
    RUNTIME DESTINATION $ENV{HOME}/your_folder/your_subfolder
    COMPONENT runtime
)

install(FILES 
    ${PROJECT_SOURCE_DIR}/config_file1.json
    ${PROJECT_SOURCE_DIR}/config_file2.json
    DESTINATION $ENV{HOME}/your_folder/your_subfolder
    COMPONENT runtime
)
```

## Step 2: Add Auto-Versioning System

Add this after the `cmake_minimum_required` section in CMakeLists.txt:

```cmake
# Auto-increment version on each build
set(VERSION_FILE "${CMAKE_SOURCE_DIR}/.version")

# Manually set major and minor versions (you can change these)
set(MAJOR_VERSION 1)
set(MINOR_VERSION 0)

if(EXISTS ${VERSION_FILE})
    file(READ ${VERSION_FILE} CURRENT_VERSION)
    string(STRIP "${CURRENT_VERSION}" CURRENT_VERSION)
    # Extract current version components
    string(REPLACE "." ";" VERSION_LIST ${CURRENT_VERSION})
    list(LENGTH VERSION_LIST LIST_LENGTH)
    if(LIST_LENGTH EQUAL 3)
        list(GET VERSION_LIST 0 STORED_MAJOR)
        list(GET VERSION_LIST 1 STORED_MINOR)
        list(GET VERSION_LIST 2 STORED_PATCH)
        
        # Check if major or minor version changed
        if(NOT (${STORED_MAJOR} EQUAL ${MAJOR_VERSION} AND ${STORED_MINOR} EQUAL ${MINOR_VERSION}))
            set(PATCH_VERSION 0)  # Reset patch when major/minor changes
            message(STATUS "Version changed from ${STORED_MAJOR}.${STORED_MINOR}.${STORED_PATCH} to ${MAJOR_VERSION}.${MINOR_VERSION}, resetting patch to 0")
        else()
            set(PATCH_VERSION ${STORED_PATCH})  # Continue with existing patch
        endif()
    else()
        set(PATCH_VERSION 0)  # Default patch version
    endif()
else()
    set(PATCH_VERSION 0)  # Default patch version
endif()

# Increment patch version
math(EXPR PATCH_VERSION "${PATCH_VERSION} + 1")

# Create new version string
set(__APP__VERSION__ "${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}")

# Write new version back to file
file(WRITE ${VERSION_FILE} ${__APP__VERSION__})

message(STATUS "Building version: ${__APP__VERSION__}")
message(STATUS "To change major/minor versions, edit MAJOR_VERSION and MINOR_VERSION above")

SET (__APP__VERSION__ "${__APP__VERSION__}")
add_definitions( -D__APP__VERSION__="${__APP__VERSION__}")
project(YOUR_PROJECT_NAME VERSION "${__APP__VERSION__}")
```

## Step 3: Update .gitignore

Add `.version` to your .gitignore file:

```
.version
```

## Step 4: Build and Package

1. **Build the project:**
   ```bash
   ./build.sh  # or your build command
   ```

2. **Generate Debian package:**
   ```bash
   cd build
   cpack -G DEB
   ```

3. **Install the package:**
   ```bash
   sudo dpkg -i YOUR_PROJECT_NAME-x.y.z-Linux.deb
   ```

## Step 5: Customize for Your Project

Replace these placeholders:
- `YOUR_PROJECT_NAME` → your actual project name
- `your_executable_name` → name of your executable target
- `your_folder/your_subfolder` → desired installation path under $HOME
- `config_file1.json`, `config_file2.json` → your actual config files
- `libstdc++6` → your actual dependencies
- `Your Team Name` → your maintainer name

## Version Control

- **Major/Minor versions**: Edit `MAJOR_VERSION` and `MINOR_VERSION` manually
- **Patch version**: Auto-increments on each build
- **Version format**: `x.y.z` where you control x & y, z is automatic
- **Reset behavior**: Patch resets to 0 when major/minor versions change

## Installation Behavior

- Files are installed to `$HOME/your_folder/your_subfolder/`
- Binary and config files are placed in the specified location
- Reinstallation overwrites existing files
- Use `--force-confold` to preserve modified config files during reinstall

## Example Output

```
-- Building version: 1.0.5
-- To change major/minor versions, edit MAJOR_VERSION and MINOR_VERSION above
...
-- package: /path/to/build/YOUR_PROJECT_NAME-1.0.5-Linux.deb generated
```

## Quick Copy-Paste Summary

### CMakeLists.txt additions:
1. **Auto-versioning** (after cmake_minimum_required): Lines 30-67 from Step 2
2. **CPack config** (at end): Lines 1-20 from Step 1

### .gitignore addition:
```
.version
```

### Build commands:
```bash
./build.sh
cd build
cpack -G DEB
sudo dpkg -i *.deb
```
