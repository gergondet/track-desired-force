#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define TrackDesiredForce_DLLIMPORT __declspec(dllimport)
#  define TrackDesiredForce_DLLEXPORT __declspec(dllexport)
#  define TrackDesiredForce_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define TrackDesiredForce_DLLIMPORT __attribute__((visibility("default")))
#    define TrackDesiredForce_DLLEXPORT __attribute__((visibility("default")))
#    define TrackDesiredForce_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define TrackDesiredForce_DLLIMPORT
#    define TrackDesiredForce_DLLEXPORT
#    define TrackDesiredForce_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef TrackDesiredForce_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define TrackDesiredForce_DLLAPI
#  define TrackDesiredForce_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef TrackDesiredForce_EXPORTS
#    define TrackDesiredForce_DLLAPI TrackDesiredForce_DLLEXPORT
#  else
#    define TrackDesiredForce_DLLAPI TrackDesiredForce_DLLIMPORT
#  endif // TrackDesiredForce_EXPORTS
#  define TrackDesiredForce_LOCAL TrackDesiredForce_DLLLOCAL
#endif // TrackDesiredForce_STATIC