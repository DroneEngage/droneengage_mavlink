#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <iostream>

#include "./de_common/helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;
                                 

#if !defined(UNUSED)
#define UNUSED(x) (void)(x) // Variables and parameters that are not used
#endif

#define BIT(x) (1 << (x))

#define SKIP_RC_CHANNEL -999
#define RC_CHANNELS_MAX 18
#endif

