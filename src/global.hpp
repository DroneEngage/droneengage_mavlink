#include <iostream>

#include "./helpers/json.hpp"
using Json = nlohmann::json;

typedef void (*SENDJMSG_CALLBACK)(const std::string& targetPartyID, const Json&, const int&, const bool& );
