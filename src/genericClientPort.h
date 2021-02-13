
#ifndef GENERIC_CLIENT_PORT_H_
#define GENERIC_CLIENT_PORT_H_

/*
 * Generic Port Class
 *
 * This is an abstract port definition to handle both serial and UDP ports.
 */

namespace uavos::comm
{
	class CGenericClientPort
	{
		public:
			CGenericClientPort(){};
			virtual ~CGenericClientPort(){};
			virtual int read_message(const char*  message)=0;
			virtual int write_message(const char* message, const int len)=0;
			virtual bool is_running()=0;
			virtual void start()=0;
			virtual void stop()=0;
	};
}


#endif // GENERIC_CLIENT_PORT_H_


