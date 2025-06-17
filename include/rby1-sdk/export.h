#pragma once

#if defined(_WIN32) || defined(_WIN64)
  #ifdef RBY1_SDK_EXPORTS
    #define RBY1_SDK_API __declspec(dllexport)
  #else
    #define RBY1_SDK_API __declspec(dllimport)
  #endif
#else
  #define RBY1_SDK_API __attribute__((visibility("default")))
#endif