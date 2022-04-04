
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <memory> 
#include "../helpers/colors.hpp"
#include "../helpers/helpers.hpp"

#include "configFile.hpp"

using namespace uavos;


const Json& CConfigFile::GetConfigJSON()
{
    return  m_ConfigJSON;
}


void CConfigFile::initConfigFile (const char* fileURL)
{
    m_file_url = std::string(fileURL);

    std::cout << _LOG_CONSOLE_TEXT_BOLD_ << "Read config file: " << _INFO_CONSOLE_TEXT << fileURL << "\033[0m ...."  << std::endl;
    
    CConfigFile::ReadFile (m_file_url.c_str());
    
    CConfigFile::ParseData (m_fileContents.str());
}


void CConfigFile::reloadFile ()
{
    CConfigFile::ReadFile (m_file_url.c_str());
    
    CConfigFile::ParseData (m_fileContents.str());
}



void CConfigFile::ReadFile (const char * fileURL)
{
    std::ifstream stream;
    
    stream.open (fileURL , std::ifstream::in);
    if (!stream) {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _ERROR_CONSOLE_TEXT_ << " FAILED to read config file " << _NORMAL_CONSOLE_TEXT_ << std::endl;
        exit(1); // terminate with error
    }
    
    
    m_fileContents.str("");
    m_fileContents <<  stream.rdbuf();
    
   
    
    return ;
}

void CConfigFile::ParseData (std::string jsonString)
{

    m_ConfigJSON = Json::parse(removeComments(jsonString));
    
}