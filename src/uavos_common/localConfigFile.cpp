
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <memory> 
#include "../helpers/colors.hpp"
#include "../helpers/helpers.hpp"

#include "localConfigFile.hpp"

using namespace uavos;


const Json& CLocalConfigFile::GetConfigJSON()
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
    std::cout << _LOG_CONSOLE_TEXT_BOLD_ << "Write internal config file: " << _SUCCESS_CONSOLE_TEXT_ << fileURL << _NORMAL_CONSOLE_TEXT_ << " ...." ;

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
    std::cout << _LOG_CONSOLE_TEXT_BOLD_ << "Read internal config file: " << _INFO_CONSOLE_TEXT << fileURL << _NORMAL_CONSOLE_TEXT_ << " ...." ;

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
        m_ConfigJSON = Json::parse(removeComments(jsonString));
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