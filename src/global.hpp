
#include "./helpers/json.hpp"
using Json = nlohmann::json;

typedef void (*SENDJMSG_CALLBACK)(const char * targetPartyID, const Json&);
