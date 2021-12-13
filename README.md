# DS18B20
*STM32* LL(Low Layer) library. DS18B20 is 1-Wire digital temperature sensor from Maxim IC. 
Celsius temperature measurement with 9 to 12-bit precision, from -55 to 125 (+/-0.5). 
Each sensor has a unique 64-Bit Serial number etched into it - allows for a huge number of sensors to be used on one data bus.

<img src="https://github.com/ximtech/DS18B20/blob/main/example/ds18b20.PNG" alt="image" width="300"/>

### Features
- Multiple sensor 1-Wire scan on single port
- Single sensor on port support
- Sensor auto address resolving
- CRC data validation

### Add as CPM project dependency

How to add CPM to the project, check the [link](https://github.com/cpm-cmake/CPM.cmake)
```cmake
CPMAddPackage(
        NAME DS18B20
        GITHUB_REPOSITORY ximtech/DS18B20
        GIT_TAG origin/main)
```

### Project configuration

1. Start project with STM32CubeMX:
    * [GPIO configuration](https://github.com/ximtech/DS18B20/blob/main/example/config.PNG)
2. Select: Project Manager -> Advanced Settings -> GPIO -> LL
3. Generate Code
4. Add sources to project:

```cmake
include_directories(${includes} 
        ${DS18B20_DIRECTORY})   # source directories

file(GLOB_RECURSE SOURCES ${sources} 
        ${DS18B20_SOURCES})    # source files

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT}) # executable declaration should be before libraries

target_link_libraries(${PROJECT_NAME}.elf Vector)   # add library dependencies to project
target_link_libraries(${PROJECT_NAME}.elf CRC)
```

3. Then Build -> Clean -> Rebuild Project

### Wiring

- <img src="https://github.com/ximtech/DS18B20/blob/main/example/wiring.PNG" alt="image" width="300"/>
- <img src="https://github.com/ximtech/DS18B20/blob/main/example/multiple_wiring.PNG" alt="image" width="300"/>

## Usage

- Usage example: [link](https://github.com/ximtech/DS18B20/blob/main/example/example.c)