#ifndef VERSION_H_
#define VERSION_H_


#include <stdio.h>

#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define REVISION 3
#define STRINGIFY(x) #x
#define VERSION_STR(A,B,C) STRINGIFY(A) "." STRINGIFY(B) "."  STRINGIFY(C)


static std::string version_string = VERSION_STR(VERSION_MAJOR, VERSION_MINOR, REVISION);

#endif