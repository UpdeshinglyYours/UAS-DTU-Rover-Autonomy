# Default to Release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )
list(APPEND CMAKE_MODULE_PATH "${openvdb_vendor_DIR}/../../../opt/openvdb_vendor/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)
