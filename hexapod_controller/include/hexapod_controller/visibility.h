#ifndef HEXAPOD_CONTROLLER__VISIBILITY_H_
#define HEXAPOD_CONTROLLER__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define HEXAPOD_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define HEXAPOD_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define HEXAPOD_CONTROLLER_EXPORT __declspec(dllexport)
    #define HEXAPOD_CONTROLLER_IMPORT __declspec(dllimport)
  #endif

  #ifdef HEXAPOD_CONTROLLER_DLL
    #define HEXAPOD_CONTROLLER_PUBLIC HEXAPOD_CONTROLLER_EXPORT
  #else
    #define HEXAPOD_CONTROLLER_PUBLIC HEXAPOD_CONTROLLER_IMPORT
  #endif

  #define HEXAPOD_CONTROLLER_PUBLIC_TYPE HEXAPOD_CONTROLLER_PUBLIC

  #define HEXAPOD_CONTROLLER_LOCAL

#else

  #define HEXAPOD_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define HEXAPOD_CONTROLLER_IMPORT

  #if __GNUC__ >= 4
    #define HEXAPOD_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define HEXAPOD_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HEXAPOD_CONTROLLER_PUBLIC
    #define HEXAPOD_CONTROLLER_LOCAL
  #endif

  #define HEXAPOD_CONTROLLER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // HEXAPOD_CONTROLLER__VISIBILITY_H_