
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <memory> 
#include "./helpers/colors.h"


#include "configFile.hpp"

using namespace uavos;


Json& CConfigFile::GetConfigJSON()
{
    return  m_ConfigJSON;
}


void CConfigFile::InitConfigFile (const char* fileURL)
{
    CConfigFile::ReadFile (fileURL);
    
    CConfigFile::ParseData (m_fileContents.str());
    
}


void CConfigFile::ReadFile (const char * fileURL)
{
    std::ifstream stream;
    std::cout << "Read config file: " << _LOG_CONSOLE_TEXT_BOLD_ << fileURL << "\033[0m ...." ;

    stream.open (fileURL , std::ifstream::in);
    if (!stream) {
        std::cout << _ERROR_CONSOLE_TEXT_ << " FAILED " << _NORMAL_CONSOLE_TEXT_ << std::endl;
        exit(1); // terminate with error
    }
    
    std::cout << _SUCCESS_CONSOLE_TEXT_ << " succeeded "  << _NORMAL_CONSOLE_TEXT_ << std::endl;

    m_fileContents <<  stream.rdbuf();
    
    return ;
}

void CConfigFile::ParseData (std::string jsonString)
{
    

    m_ConfigJSON = Json::parse(jsonString);

    // cout << "module_id: " << m_ConfigJSON["module_id"] << endl;

    // cout << "module_features: " << m_ConfigJSON["module_features"][1] << endl;

    // cout << "module_features len: " << m_ConfigJSON["module_features"].size() << endl;
}