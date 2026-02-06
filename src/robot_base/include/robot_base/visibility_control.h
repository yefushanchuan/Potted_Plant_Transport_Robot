#ifndef ROBOT_BASE__VISIBILITY_CONTROL_H_
#define ROBOT_BASE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_BASE_EXPORT __attribute__ ((dllexport))
    #define ROBOT_BASE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_BASE_EXPORT __declspec(dllexport)
    #define ROBOT_BASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_BASE_BUILDING_DLL
    #define ROBOT_BASE_PUBLIC ROBOT_BASE_EXPORT
  #else
    #define ROBOT_BASE_PUBLIC ROBOT_BASE_IMPORT
  #endif
  #define ROBOT_BASE_PUBLIC_TYPE ROBOT_BASE_PUBLIC
  #define ROBOT_BASE_LOCAL
#else
  #define ROBOT_BASE_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_BASE_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_BASE_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_BASE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_BASE_PUBLIC
    #define ROBOT_BASE_LOCAL
  #endif
  #define ROBOT_BASE_PUBLIC_TYPE
#endif

#endif  // ROBOT_BASE__VISIBILITY_CONTROL_H_
