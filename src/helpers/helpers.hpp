#ifndef HELPERS_H_
#define HELPERS_H_

#include <iostream>
#include <cctype>
#include <algorithm>
#include <sys/time.h>
#include <vector>
#include <sstream>

uint64_t get_time_usec();

int wait_time_nsec (const time_t& seconds, const long& nano_seconds);

std::string str_tolower(std::string s);

std::vector<std::string> split_string_by_delimeter(const std::string& str, const char& delimeter);

std::vector<std::string> split_string_by_newline(const std::string& str);

std::string removeComments(std::string prgm);

extern std::string get_linux_machine_id ();
#endif