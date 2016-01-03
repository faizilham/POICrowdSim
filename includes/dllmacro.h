#ifndef POICSDLLMACRO_H
#define POICSDLLMACRO_H

#ifdef _WIN32
#ifdef BUILD_POICS_DLL
#define POICS_API __declspec(dllexport)
#else
#define POICS_API __declspec(dllimport)
#endif
#else
#define POICS_API
#endif

#endif