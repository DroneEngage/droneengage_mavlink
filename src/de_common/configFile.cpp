#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <memory> 
#include <iomanip>
#include <ctime>
#include "../helpers/colors.hpp"
#include "../helpers/helpers.hpp"

#include "configFile.hpp"

using namespace de;

const Json_de& CConfigFile::GetConfigJSON()
{
    return m_ConfigJSON;
}

void CConfigFile::initConfigFile (const char* fileURL)
{
    m_file_url = std::string(fileURL);
    CConfigFile::ReadFile (m_file_url.c_str());
    CConfigFile::ParseData (m_fileContents.str());

    try {
        m_lastWriteTime = std::filesystem::last_write_time(m_file_url.c_str());
        std::cout << _INFO_CONSOLE_TEXT << "Initial last write time obtained." << _NORMAL_CONSOLE_TEXT_ << std::endl;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Could not get initial file write time for '" << m_file_url << "': " << e.what() << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
}

void CConfigFile::reloadFile ()
{
    CConfigFile::ReadFile (m_file_url.c_str());
    CConfigFile::ParseData (m_fileContents.str());

    try {
        m_lastWriteTime = std::filesystem::last_write_time(m_file_url.c_str());
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Could not update file write time after reload for '" << m_file_url << "': " << e.what() << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
}

bool CConfigFile::fileUpdated ()
{
    if (m_file_url.empty()) {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: File URL is empty." << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false;
    }

    if (!std::filesystem::exists(m_file_url)) {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Config file does not exist: " << m_file_url << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false;
    }

    try {
        std::filesystem::file_time_type lastWriteTime = std::filesystem::last_write_time(m_file_url.c_str());
        if (lastWriteTime == m_lastWriteTime) return false;
        
        m_lastWriteTime = lastWriteTime;
        std::cout << _INFO_CONSOLE_TEXT << "Initial last write time obtained." << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Could not get file write time for '" << m_file_url << "': " << e.what() << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false;
    }
}

void CConfigFile::ReadFile (const char * fileURL)
{
    std::ifstream stream;
    std::cout << _LOG_CONSOLE_BOLD_TEXT << "Read config file: " << _INFO_CONSOLE_TEXT << fileURL << "\033[0m ...." ;
    
    stream.open (fileURL , std::ifstream::in);
    if (!stream) {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "FATAL ERROR:" << _ERROR_CONSOLE_TEXT_ << " FAILED to read config file " << _NORMAL_CONSOLE_TEXT_ << std::endl;
        exit(1); // terminate with error
    }
    
    m_fileContents.str("");
    m_fileContents << stream.rdbuf();
    std::cout << _SUCCESS_CONSOLE_TEXT_ << " succeeded " << _NORMAL_CONSOLE_TEXT_ << std::endl;
    return;
}

void CConfigFile::ParseData (std::string jsonString)
{
    m_ConfigJSON = Json_de::parse(removeComments(jsonString));
    std::cout << _SUCCESS_CONSOLE_TEXT_ << " config file parsed successfully " << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void CConfigFile::updateJSON(const std::string& jsonString)
{
    try {
        Json_de updateJson = Json_de::parse(removeComments(jsonString));
        for (const auto& item : updateJson.items()) {
            // Update existing entry or add new entry
            m_ConfigJSON[item.key()] = item.value();
            std::cout << _INFO_CONSOLE_TEXT << "Updated/Added JSON key: " << item.key() << _NORMAL_CONSOLE_TEXT_ << std::endl;
        }
        saveConfigFile();
    } catch (const std::exception& e) {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Failed to parse update JSON: " << e.what() << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return;
    }
}

void CConfigFile::saveConfigFile()
{
    // Create backup if file exists
    if (std::filesystem::exists(m_file_url)) {
        // Generate timestamp for backup
        auto now = std::time(nullptr);
        std::stringstream timestamp;
        timestamp << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
        std::string backup_url = m_file_url + ".bak_" + timestamp.str();

        try {
            std::filesystem::copy_file(m_file_url, backup_url, std::filesystem::copy_options::overwrite_existing);
            std::cout << _INFO_CONSOLE_TEXT << "Backup created: " << backup_url << _NORMAL_CONSOLE_TEXT_ << std::endl;
        } catch (const std::filesystem::filesystem_error& e) {
            std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Could not create backup file '" << backup_url << "': " << e.what() << _NORMAL_CONSOLE_TEXT_ << std::endl;
        }
    }

    std::ofstream outFile(m_file_url);
    if (!outFile) {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Could not open config file for writing: " << m_file_url << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return;
    }
    outFile << m_ConfigJSON.dump(4);
    outFile.close();
    std::cout << _SUCCESS_CONSOLE_TEXT_ << "Config file saved successfully: " << m_file_url << _NORMAL_CONSOLE_TEXT_ << std::endl;

    try {
        m_lastWriteTime = std::filesystem::last_write_time(m_file_url.c_str());
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Could not update file write time after save for '" << m_file_url << "': " << e.what() << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
}