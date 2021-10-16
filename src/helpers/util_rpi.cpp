#include <stdio.h>
#include <cstring>
#include "util_rpi.hpp"



helpers::CUtil_Rpi::CUtil_Rpi ()
{
    _check_rpi_version();
}

int helpers::CUtil_Rpi::get_rpi_model () const 
{
    return _rpi_version;
}

bool helpers::CUtil_Rpi::get_cpu_serial (std::string &cpu_serial)   const 
{
    bool found = false;
    FILE *f = fopen("/proc/cpuinfo", "r");
	if (!f) {
		return found;
	}
	
	char line[256]; 
    
	while (fgets(line, 256, f)) {
		if (strncmp(line, "Serial", 6) == 0) {
			char serial_string[16 + 1];
			strcpy(serial_string, strchr(line, ':') + 2);
            serial_string[16]=0; // remove extra \n
            cpu_serial = serial_string;
            found = true;
		}
	}
    
    if (!found) cpu_serial = "NORPI";
	
    fclose(f);

     return found;
}

int helpers::CUtil_Rpi::_check_rpi_version()
{
    const unsigned int MAX_SIZE_LINE = 50;
    char buffer[MAX_SIZE_LINE];
    
    FILE *f = fopen("/sys/firmware/devicetree/base/model", "r");
    if (f != nullptr && fgets(buffer, MAX_SIZE_LINE, f) != nullptr) {
        int ret = sscanf(buffer + 12, "%d", &_rpi_version);
        fclose(f);
        if (ret != EOF) {
            if (_rpi_version > 3)  {
                _rpi_version = 4;
            } else if (_rpi_version > 2) {
                // Preserving old behavior.
                _rpi_version = 2;
            } else if (_rpi_version == 0) {
                // RPi 1 doesn't have a number there, so sscanf() won't have read anything.
                _rpi_version = 1;
            }

            printf("%s. (intern: %d)\n", buffer, _rpi_version);

            return _rpi_version;
        }
    }
    
    return _rpi_version;
}

