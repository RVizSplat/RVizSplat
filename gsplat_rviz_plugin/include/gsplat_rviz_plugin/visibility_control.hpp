#ifndef GSPLAT_RVIZ_PLUGIN__VISIBILITY_CONTROL_HPP_
#define GSPLAT_RVIZ_PLUGIN__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GSPLAT_RVIZ_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define GSPLAT_RVIZ_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define GSPLAT_RVIZ_PLUGIN_EXPORT __declspec(dllexport)
    #define GSPLAT_RVIZ_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef GSPLAT_RVIZ_PLUGIN_BUILDING_LIBRARY
    #define GSPLAT_RVIZ_PLUGIN_PUBLIC GSPLAT_RVIZ_PLUGIN_EXPORT
  #else
    #define GSPLAT_RVIZ_PLUGIN_PUBLIC GSPLAT_RVIZ_PLUGIN_IMPORT
  #endif
  #define GSPLAT_RVIZ_PLUGIN_LOCAL
#else
  #define GSPLAT_RVIZ_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define GSPLAT_RVIZ_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define GSPLAT_RVIZ_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define GSPLAT_RVIZ_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GSPLAT_RVIZ_PLUGIN_PUBLIC
    #define GSPLAT_RVIZ_PLUGIN_LOCAL
  #endif
#endif

#endif  // GSPLAT_RVIZ_PLUGIN__VISIBILITY_CONTROL_HPP_
