
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <memory> 
#include "../helpers/colors.hpp"
#include "../helpers/helpers.hpp"

#include "localConfigFile.hpp"

using namespace de;


const Json_de& CLocalConfigFile::GetConfigJSON()
{
    return  m_ConfigJSON;
}


void CLocalConfigFile::InitConfigFile (const char* fileURL)
{
    m_ConfigJSON={};
    
    m_fileURL = std::string(fileURL);
    
    ReadFile (fileURL);
    
    ParseData (m_fileContents.str());
}


void CLocalConfigFile::apply()
{
    WriteFile (m_fileURL.c_str());
}

void CLocalConfigFile::clearFile()
{
    m_ConfigJSON={};
    WriteFile (m_fileURL.c_str());
}
            
            

void CLocalConfigFile::WriteFile (const char * fileURL)
{
    std::ofstream stream;
    std::cout << _LOG_CONSOLE_BOLD_TEXT<< "Write internal config file: " << _SUCCESS_CONSOLE_TEXT_ << fileURL << _NORMAL_CONSOLE_TEXT_ << " ...." ;

    stream.open (fileURL , std::ifstream::out | std::ios::trunc );
    if (!stream) {
        std::cout << _ERROR_CONSOLE_TEXT_ << " FAILED " << _NORMAL_CONSOLE_TEXT_ << std::endl;
        exit(1); // terminate with error
    }

    std::string j = m_ConfigJSON.dump();
    stream << j;
    stream.close();
    std::cout << _SUCCESS_CONSOLE_TEXT_ << " succeeded "  << _NORMAL_CONSOLE_TEXT_ << std::endl;

    return ;
}

void CLocalConfigFile::ReadFile (const char * fileURL)
{
    std::ifstream stream;
    std::cout << _LOG_CONSOLE_BOLD_TEXT<< "Read internal config file: " << _INFO_CONSOLE_TEXT << fileURL << _NORMAL_CONSOLE_TEXT_ << " ...." ;

    stream.open (fileURL , std::ifstream::in);
    if (!stream) {
        std::cout << _INFO_CONSOLE_TEXT << " trying to create one " << _NORMAL_CONSOLE_TEXT_ << std::endl;
        WriteFile (fileURL);
        // put JSON as string to keep contents consistent.
        m_fileContents << m_ConfigJSON;
        return ;
    }
    
    m_fileContents <<  stream.rdbuf();
    
    std::cout << _SUCCESS_CONSOLE_TEXT_ << " succeeded "  << _NORMAL_CONSOLE_TEXT_ << std::endl;

    return ;
}


bool CLocalConfigFile::ParseData (std::string jsonString)
{
   try
   {
        m_ConfigJSON = Json_de::parse(removeComments(jsonString));
   }
   catch(const std::exception& e)
   {
    std::cerr << e.what() << '\n';
    return false;
   }

   return true;
    
}


void CLocalConfigFile::addStringField(const char * field, const char * value)
{
    m_ConfigJSON[std::string(field)] = std::string(value);
}


void CLocalConfigFile::ModifyStringField(const char* field, const char* newValue)
{
    std::string key = std::string(field);
    std::string value = std::string(newValue);

    if (m_ConfigJSON.contains(key))
    {
        m_ConfigJSON[key] = value;
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "Modified string field: " << _SUCCESS_CONSOLE_TEXT_ << field << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
    else
    {
        m_ConfigJSON[key] = value; // Add it if it doesn't exist
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "Added new string field: " << _INFO_CONSOLE_TEXT << field << _NORMAL_CONSOLE_TEXT_ << " (field did not exist)" << std::endl;
    }
}

std::string CLocalConfigFile::getStringField(const char * field) const
{
    if (!m_ConfigJSON.contains(std::string(field))) return {};

    return m_ConfigJSON[std::string(field)].get<std::string>();
}


void CLocalConfigFile::addNumericField(const char * field, const u_int32_t & value)
{
    m_ConfigJSON[std::string(field)] = value;
}


const u_int32_t CLocalConfigFile::getNumericField(const char * field) const 
{
    if (!m_ConfigJSON.contains(std::string(field))) return 0xffffffff;

    return m_ConfigJSON[std::string(field)].get<int>();
}

void CLocalConfigFile::ModifyNumericField(const char* field, const u_int32_t& newValue)
{
    std::string key = std::string(field);

    if (m_ConfigJSON.contains(key))
    {
        m_ConfigJSON[key] = newValue;
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "Modified numeric field: " << _SUCCESS_CONSOLE_TEXT_ << field << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
    else
    {
        m_ConfigJSON[key] = newValue; // Add it if it doesn't exist
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "Added new numeric field: " << _INFO_CONSOLE_TEXT << field << _NORMAL_CONSOLE_TEXT_ << " (field did not exist)" << std::endl;
    }
}


void CLocalConfigFile::removeFieldByName(const char* fieldName)
{
    std::string key = std::string(fieldName);
    if (m_ConfigJSON.contains(key))
    {
        m_ConfigJSON.erase(key);
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "Removed field: " << _SUCCESS_CONSOLE_TEXT_ << fieldName << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
    else
    {
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "Field not found: " << _INFO_CONSOLE_TEXT << fieldName << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
}



void CLocalConfigFile::addDoubleField(const char * field, double value)
{
    m_ConfigJSON[std::string(field)] = value;
}

double CLocalConfigFile::getDoubleField(const char * field) const
{
    std::string key = std::string(field);
    if (!m_ConfigJSON.contains(key))
    {
        std::cout << _INFO_CONSOLE_TEXT << "Double field '" << key << "' not found." << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return 0.0; // Or some other default/error value
    }
    if (!m_ConfigJSON[key].is_number()) { // Check if it's any number (integer or float)
        std::cout << _ERROR_CONSOLE_TEXT_ << "Field '" << key << "' is not a number." << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return 0.0;
    }
    return m_ConfigJSON[key].get<double>();
}

void CLocalConfigFile::ModifyDoubleField(const char* field, double newValue)
{
    std::string key = std::string(field);

    if (m_ConfigJSON.contains(key))
    {
        m_ConfigJSON[key] = newValue;
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "Modified double field: " << _SUCCESS_CONSOLE_TEXT_ << field << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
    else
    {
        m_ConfigJSON[key] = newValue;
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "Added new double field: " << _INFO_CONSOLE_TEXT << field << _NORMAL_CONSOLE_TEXT_ << " (field did not exist)" << std::endl;
    }
}