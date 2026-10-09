#ifndef GROUND_SEGMENTATION__VISIBILITY_CONTROL_H_
#define GROUND_SEGMENTATION__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GROUND_SEGMENTATION_EXPORT __attribute__ ((dllexport))
    #define GROUND_SEGMENTATION_IMPORT __attribute__ ((dllimport))
  #else
    #define GROUND_SEGMENTATION_EXPORT __declspec(dllexport)
    #define GROUND_SEGMENTATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef GROUND_SEGMENTATION_BUILDING_LIBRARY
    #define GROUND_SEGMENTATION_PUBLIC GROUND_SEGMENTATION_EXPORT
  #else
    #define GROUND_SEGMENTATION_PUBLIC GROUND_SEGMENTATION_IMPORT
  #endif
  #define GROUND_SEGMENTATION_PUBLIC_TYPE GROUND_SEGMENTATION_PUBLIC
  #define GROUND_SEGMENTATION_LOCAL
#else
  #define GROUND_SEGMENTATION_EXPORT __attribute__ ((visibility("default")))
  #define GROUND_SEGMENTATION_IMPORT
  #if __GNUC__ >= 4
    #define GROUND_SEGMENTATION_PUBLIC __attribute__ ((visibility("default")))
    #define GROUND_SEGMENTATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GROUND_SEGMENTATION_PUBLIC
    #define GROUND_SEGMENTATION_LOCAL
  #endif
  #define GROUND_SEGMENTATION_PUBLIC_TYPE
#endif

#endif  // GROUND_SEGMENTATION__VISIBILITY_CONTROL_H_
