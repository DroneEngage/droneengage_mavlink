
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <memory> 
#include "../helpers/colors.hpp"
#include "../helpers/helpers.hpp"

#include "configFile.hpp"

using namespace de;


const Json_de& CConfigFile::GetConfigJSON()
{
    return  m_ConfigJSON;
}


void CConfigFile::initConfigFile (const char* fileURL)
{
    m_file_url = std::string(fileURL);

    CConfigFile::ReadFile (m_file_url.c_str());
    
    CConfigFile::ParseData (m_fileContents.str());
}


void CConfigFile::reloadFile ()
{
    CConfigFile::ReadFile (m_file_url.c_str());
    
    CConfigFile::ParseData (m_fileContents.str());
}


bool CConfigFile::fileUpdated ()
{
    try {
        std::filesystem::file_time_type lastWriteTime = std::filesystem::last_write_time(m_file_url.c_str());
        if (lastWriteTime == m_lastWriteTime) return false;
        
        m_lastWriteTime = lastWriteTime;
            
        std::cout << _INFO_CONSOLE_TEXT << "Initial last write time obtained." << _NORMAL_CONSOLE_TEXT_ << std::endl;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Could not get initial file write time for '" << m_file_url.c_str() << "': " << e.what() << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }

    return true;
}


void CConfigFile::ReadFile (const char * fileURL)
{
    std::ifstream stream;
    
    std::cout << _LOG_CONSOLE_BOLD_TEXT<< "Read config file: " << _INFO_CONSOLE_TEXT << fileURL << "\033[0m ...." ;
    
    stream.open (fileURL , std::ifstream::in);
    if (!stream) {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _ERROR_CONSOLE_TEXT_ << " FAILED to read config file " << _NORMAL_CONSOLE_TEXT_ << std::endl;
        exit(1); // terminate with error
    }
    
    
    m_fileContents.str("");
    m_fileContents <<  stream.rdbuf();
    
    std::cout << _SUCCESS_CONSOLE_TEXT_ << " succeeded "  << _NORMAL_CONSOLE_TEXT_ << std::endl;

    
    return ;
}

void CConfigFile::ParseData (std::string jsonString)
{

    m_ConfigJSON = Json_de::parse(removeComments(jsonString));

    std::cout << _SUCCESS_CONSOLE_TEXT_ << " config file parsed successfully "  << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
}