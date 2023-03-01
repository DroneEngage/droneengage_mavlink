#include <iostream>

#include "./helpers/json.hpp"
using Json = nlohmann::json;
                                 

#if !defined(UNUSED)
#define UNUSED(x) (void)(x) // Variables and parameters that are not used
#endif

#define BIT(x) (1 << (x))
