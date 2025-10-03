#include <iostream>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <sys/time.h>
#include <vector>
#include <sstream>

#include "helpers.hpp"


std::string get_time_string()
{
  struct timeval _time_stamp;
  gettimeofday(&_time_stamp, NULL);

  // Convert the timestamp to a time_t value
  time_t time_t_value = _time_stamp.tv_sec;

  // Convert the time_t value to a tm struct
  struct tm *tm = localtime(&time_t_value);

  // Format the time string
  char time_string[20];
  strftime(time_string, sizeof(time_string), "%Y_%m_%d_%H_%M_%S", tm);

  return time_string;
}

uint64_t get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

int wait_time_nsec (const time_t& seconds, const long& nano_seconds)
{
	struct timespec _time_wait, tim2;
	_time_wait.tv_sec = seconds;
	_time_wait.tv_nsec = nano_seconds;
	
	return nanosleep(&_time_wait, &tim2);
}

std::string str_tolower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), 
                // static_cast<int(*)(int)>(std::tolower)         // wrong
                // [](int c){ return std::tolower(c); }           // wrong
                // [](char c){ return std::tolower(c); }          // wrong
                   [](unsigned char c){ return std::tolower(c); } // correct
                  );
    return s;
}



std::vector<std::string> split_string_by_delimeter(const std::string& str, const char& delimeter)
{
    auto result = std::vector<std::string>{};
    auto ss = std::stringstream{str};

    for (std::string line; std::getline(ss, line, delimeter);)
        result.push_back(line);

    return result;
}



std::vector<std::string> split_string_by_newline(const std::string& str)
{
    return split_string_by_delimeter (str, '\n');
}


/**
 * @brief 
 * Remove comments from strings
 * http://www.cplusplus.com/forum/beginner/163419/
 * @param prgm 
 * @return std::string 
 */
std::string removeComments(std::string prgm) 
{ 
    int n = prgm.length(); 
    std::string res; 
  
    // Flags to indicate that single line and multpile line comments 
    // have started or not. 
    bool s_cmt = false; 
    bool m_cmt = false; 
  
  
    // Traverse the given program 
    for (int i=0; i<n; i++) 
    { 
        // If single line comment flag is on, then check for end of it 
        if (s_cmt == true && prgm[i] == '\n') 
            s_cmt = false; 
  
        // If multiple line comment is on, then check for end of it 
        else if  (m_cmt == true && prgm[i] == '*' && (i + 1) < n && prgm[i+1] == '/') 
            m_cmt = false,  i++; 
  
        // If this character is in a comment, ignore it 
        else if (s_cmt || m_cmt) 
            continue; 
  
        // Check for beginning of comments and set the approproate flags 
        else if ((prgm[i] == '/') && ((i + 1) < n) && prgm[i+1] == '/') 
            s_cmt = true, i++; 
        else if ((prgm[i] == '/') && ((i + 1) < n) && prgm[i+1] == '*') 
            m_cmt = true,  i++; 
  
        // If current character is a non-comment character, append it to res 
        else  res += prgm[i]; 
    } 
    return res; 
} 


/**
 * @brief Get the linux machine id object
 * 
 * @return std::string 
 */
std::string get_linux_machine_id ()
{
    FILE *f = fopen("/etc/machine-id", "r");
	if (!f) {
		return std::string();
	}
    char line[256]; 
    memset (line,0,255);
    char * read = fgets(line, 256, f);
    if (read!= NULL)
    {
        size_t len = strlen(line);
        if (len > 0 && line[len-1] == '\n') line[len-1] = '\0'; // remove "\n" if present
        std::string id(line);
        fclose(f);
        return id;
    }
    else
    {
        fclose(f);
        return std::string("");
    }
}


bool is_ascii(const signed char *c, size_t len) {
  for (size_t i = 0; i < len; i++) {
        // cast to unsigned to avoid implementation-defined signedness issues
        if (static_cast<unsigned char>(c[i]) > 127) return false;
  }
  return true;
}


/**
 * @brief 
 *  returns true if field exist and of specified type
 * @param message Json_de object
 * @param field_name requested field name
 * @param field_type specified type
 * @return true if found and of specified type
 * @return false 
 */
bool validateField (const Json_de& message, const char *field_name, const Json_de::value_t& field_type)
{
    if (
        (message.contains(field_name) == false) 
        || (message[field_name].type() != field_type)
        ) 
    
        return false;

    return true;
}


// Function to round a double to a specified number of decimal places
double roundToPrecision(double value, int decimalPlaces) {
    double multiplier = std::pow(10.0, decimalPlaces);
    return std::round(value * multiplier) / multiplier;
}