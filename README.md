| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

# _Sample project_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This is the simplest buildable example. The example is used by command `idf.py create-project`
that copies the project to user specified path and set it's name. For more information follow the [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project)



## How to use example
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── components
    ├──ttn-esp32
├── dependencires.lock
├── main
│   ├── CMakeLists.txt
|   ├── idf_component.yml
|   ├──Kconfig.projbuild
|   ├── main.c
|   ├── nmea_parser.c
│   └── nmea_parser.h
├──  managed_components
|   ├── espressif_cmake_utilities
|   ├── espressif_esp_lcd_ili9341
|   └── lvgl_lvgl 
└── README.md                  This is the file you are currently reading
└── sdkconfig
└── sdkconfig.old
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.

For the configuration of utilizing this firmware. User should make a [Pin Connections](#pin-connections) according to the following table.

## Pin Connections

| Components          | Components Pin | ESP Pin     |
|---------------------|----------------|-------------|
| **ST7735s TFT Display** |                |             |
|                     | SCL            | GPIO 13     |
|                     | SDA            | GPIO 15     |
|                     | RST            | GPIO 4      |
|                     | DC             | GPIO 2      |
|                     | CS             | GPIO 14     |
|                     | BLK            | NC          |
| **RA-01H**          |                |             |
|                     | SCK            | GPIO 5      |
|                     | MOSI           | GPIO 27     |
|                     | MISO           | GPIO 19     |
|                     | NSS            | GPIO 18     |
|                     | DIO0           | GPIO 26     |
|                     | DIO1           | GPIO 35     |
| **GPS01-TD**        |                |             |
|                     | Tx             | GPIO 16     |
| **Button**          |                |             |
|                     | Button MSG     | GPIO 33     |
|                     | Button RST     | GPIO 25     |

## Menu-configuration

Menuconfiguration of this LoRaWAN Tester, since it is utilizing 3 library differently, make user need to configure each parameter to suit the library accordingly.
| Library          | Parameter | Info     |
|---------------------|----------------|-------------|
| **LMIC**                |                |                                    |
|                         | LoRa Frequency | Any Frequency between 803-920 MHz  |
|                         | Radio Chip     | SX1276                             |
| **NMEA Configuration**  |                          |                     |
|                     | RxD Pin                      | GPIO 16             |
|                     | NMEA Statement Support       | GGA, GSAA, GSV, RMC |
| **lvgl**            |                           |                             |
|                     | Color Setting             | Swap 2 bytes of RGB565 Color|
|                     | LCD Controller Model      |   ILI9341                   |

The above configuration is based on the module that use in the tester:
- LoRa Module
  - RA-01H
- GNSS
  - GPS01-TD
- LVGL
  - ST7735s
