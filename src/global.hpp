#include <iostream>

#include "./helpers/json.hpp"
using Json = nlohmann::json;
                                 
typedef void (*SEND_JMSG_CALLBACK)(const std::string& targetPartyID, const Json&, const int&, const bool& );
typedef void (*SEND_BMSG_CALLBACK)(const std::string& targetPartyID, const char *, const int bmsg_length, const int& , const bool&);