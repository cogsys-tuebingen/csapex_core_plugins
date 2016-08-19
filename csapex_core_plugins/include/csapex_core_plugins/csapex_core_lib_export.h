
#ifndef CSAPEX_CORE_LIB_EXPORT_H
#define CSAPEX_CORE_LIB_EXPORT_H

#ifdef CSAPEX_CORE_LIB_STATIC_DEFINE
#  define CSAPEX_CORE_LIB_EXPORT
#  define CSAPEX_CORE_LIB_NO_EXPORT
#else
#  ifndef CSAPEX_CORE_LIB_EXPORT
#    ifdef csapex_core_lib_EXPORTS
        /* We are building this library */
#      define CSAPEX_CORE_LIB_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CSAPEX_CORE_LIB_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CSAPEX_CORE_LIB_NO_EXPORT
#    define CSAPEX_CORE_LIB_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CSAPEX_CORE_LIB_DEPRECATED
#  define CSAPEX_CORE_LIB_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CSAPEX_CORE_LIB_DEPRECATED_EXPORT
#  define CSAPEX_CORE_LIB_DEPRECATED_EXPORT CSAPEX_CORE_LIB_EXPORT CSAPEX_CORE_LIB_DEPRECATED
#endif

#ifndef CSAPEX_CORE_LIB_DEPRECATED_NO_EXPORT
#  define CSAPEX_CORE_LIB_DEPRECATED_NO_EXPORT CSAPEX_CORE_LIB_NO_EXPORT CSAPEX_CORE_LIB_DEPRECATED
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define CSAPEX_CORE_LIB_NO_DEPRECATED
#endif

#endif
