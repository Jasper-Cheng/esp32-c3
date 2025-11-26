# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/942.AD/esp/v5.2.2/esp-idf/components/bootloader/subproject"
  "C:/Users/942.AD/Desktop/esp_study_project/esp32-c3-light/build/bootloader"
  "C:/Users/942.AD/Desktop/esp_study_project/esp32-c3-light/build/bootloader-prefix"
  "C:/Users/942.AD/Desktop/esp_study_project/esp32-c3-light/build/bootloader-prefix/tmp"
  "C:/Users/942.AD/Desktop/esp_study_project/esp32-c3-light/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/942.AD/Desktop/esp_study_project/esp32-c3-light/build/bootloader-prefix/src"
  "C:/Users/942.AD/Desktop/esp_study_project/esp32-c3-light/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/942.AD/Desktop/esp_study_project/esp32-c3-light/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/942.AD/Desktop/esp_study_project/esp32-c3-light/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
