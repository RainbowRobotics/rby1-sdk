#pragma once

#if defined(_WIN32) || defined(_WIN64)
#if defined(RBY1_SDK_STATIC)
#define RBY1_SDK_API
#elif defined(RBY1_SDK_EXPORTS)
#define RBY1_SDK_API __declspec(dllexport)
#else
#define RBY1_SDK_API __declspec(dllimport)
#endif
#else
#if defined(RBY1_SDK_STATIC)
#define RBY1_SDK_API
#else
#define RBY1_SDK_API __attribute__((visibility("default")))
#endif
#endif