#ifndef BBOT_HARDWARE__VISIBILITY_CONTROL_H_
#define BBOT_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define BBOT_HARDWARE_EXPORT __attribute__((dllexport))
#define BBOT_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define BBOT_HARDWARE_EXPORT __declspec(dllexport)
#define BBOT_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef BBOT_HARDWARE_BUILDING_DLL
#define BBOT_HARDWARE_PUBLIC BBOT_HARDWARE_EXPORT
#else
#define BBOT_HARDWARE_PUBLIC BBOT_HARDWARE_IMPORT
#endif
#define BBOT_HARDWARE_PUBLIC_TYPE BBOT_HARDWARE_PUBLIC
#define BBOT_HARDWARE_LOCAL
#else
#define BBOT_HARDWARE_EXPORT __attribute__((visibility("default")))
#define BBOT_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define BBOT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define BBOT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define BBOT_HARDWARE_PUBLIC
#define BBOT_HARDWARE_LOCAL
#endif
#define BBOT_HARDWARE_PUBLIC_TYPE
#endif

#endif  // BBOT_HARDWARE__VISIBILITY_CONTROL_H_
