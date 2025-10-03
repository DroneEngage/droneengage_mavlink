#ifndef HELPERS_H_
#define HELPERS_H_

#include <iostream>
#include <cctype>
#include <algorithm>
#include <sys/time.h>
#include <vector>
#include <sstream>
#include <math.h>

#include "json_nlohmann.hpp"
using Json_de = nlohmann::json;

std::string get_time_string();

uint64_t get_time_usec();

int wait_time_nsec (const time_t& seconds, const long& nano_seconds);

std::string str_tolower(std::string s);

std::vector<std::string> split_string_by_delimeter(const std::string& str, const char& delimeter);

std::vector<std::string> split_string_by_newline(const std::string& str);

std::string removeComments(std::string prgm);

extern bool is_ascii(const signed char *c, size_t len);

extern bool validateField (const Json_de& message, const char *field_name, const Json_de::value_t& field_type);

extern std::string get_linux_machine_id ();


template <typename T> inline constexpr
int signum(T x, std::false_type is_signed) {
    return T(0) < x;
}

template <typename T> inline constexpr
int signum(T x, std::true_type is_signed) {
    return (T(0) < x) - (x < T(0));
}

template <typename T> inline constexpr
int signum(T x) {
    return signum(x, std::is_signed<T>());
}

double roundToPrecision(double value, int decimalPlaces);

#endif