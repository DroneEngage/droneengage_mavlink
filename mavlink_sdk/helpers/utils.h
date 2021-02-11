#ifndef UTILS_H_
#define UTILS_H_

#include <sys/time.h>

inline uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

#endif
