#ifndef CSAPEX_MATH_EXPORT_H
#define CSAPEX_MATH_EXPORT_H

#ifdef CSAPEX_MATH_STATIC_DEFINE
#define CSAPEX_MATH_EXPORT
#define CSAPEX_MATH_NO_EXPORT
#else
#ifndef CSAPEX_MATH_EXPORT
#ifdef csapex_MATH_EXPORTS
/* We are building this library */
#define CSAPEX_MATH_EXPORT __attribute__((visibility("default")))
#else
/* We are using this library */
#define CSAPEX_MATH_EXPORT __attribute__((visibility("default")))
#endif
#endif

#ifndef CSAPEX_MATH_NO_EXPORT
#define CSAPEX_MATH_NO_EXPORT __attribute__((visibility("hidden")))
#endif
#endif

#ifndef CSAPEX_MATH_DEPRECATED
#define CSAPEX_MATH_DEPRECATED __attribute__((__deprecated__))
#endif

#ifndef CSAPEX_MATH_DEPRECATED_EXPORT
#define CSAPEX_MATH_DEPRECATED_EXPORT CSAPEX_MATH_EXPORT CSAPEX_MATH_DEPRECATED
#endif

#ifndef CSAPEX_MATH_DEPRECATED_NO_EXPORT
#define CSAPEX_MATH_DEPRECATED_NO_EXPORT CSAPEX_MATH_NO_EXPORT CSAPEX_MATH_DEPRECATED
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
#define CSAPEX_MATH_NO_DEPRECATED
#endif

#endif  // CSAPEX_MATH_EXPORT_H
