#ifndef UTILS_H_
#define UTILS_H_

#include <sys/time.h>
#include <time.h>
#include <cstdint>

inline uint64_t get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

// Monotonic clock for rate limiting/timeouts: get_time_usec() uses gettimeofday
// and can step backward on NTP/RTC corrections, which would underflow a uint64 diff.
inline uint64_t get_time_usec_monotonic()
{
	static struct timespec _time_stamp;
	clock_gettime(CLOCK_MONOTONIC, &_time_stamp);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_nsec/1000;
}

inline int wait_time_nsec (const time_t& seconds, const long& nano_seconds)
{
	struct timespec _time_wait, tim2;
	_time_wait.tv_sec = seconds;
	_time_wait.tv_nsec = nano_seconds;
	
	return nanosleep(&_time_wait, &tim2);
}

#endif
