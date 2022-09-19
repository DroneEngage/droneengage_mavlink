#ifndef VERSION_H_
#define VERSION_H_


#include <stdio.h>

#define VERSION_MAJOR 2
#define VERSION_MINOR 1
#define REVISION 0
#define STRINGIFY(x) #x
#define VERSION_STR(A,B,C) STRINGIFY(A) "." STRINGIFY(B) "."  STRINGIFY(C)


static std::string version_string = VERSION_STR(VERSION_MAJOR, VERSION_MINOR, REVISION);

#endif