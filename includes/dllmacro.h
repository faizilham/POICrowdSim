#ifndef POICSDLLMACRO_H
#define POICSDLLMACRO_H

#ifdef BUILD_POICS_DLL
#define POICS_API __declspec(dllexport)
#else
#define POICS_API __declspec(dllimport)
#endif

#endif