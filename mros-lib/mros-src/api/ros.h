#ifndef _ROS_H_
#define _ROS_H_

#include <string>

namespace ros {

void init(int argc, char *argv, std::string node_name);


class Publisher {
public:
	template <class T>
	void publish(T& data);
	void set(void *cobj)
	{
		this->cobj = cobj;
	}
	void* get()
	{
		return this->cobj;
	}
private:
	void *cobj;
};

class Subscriber {
public:
	void set(void *cobj)
	{
		this->cobj = cobj;
	}
	void* get()
	{
		return this->cobj;
	}
private:
	void *cobj;
};


class NodeHandle {
public:
	template<class T>
	Subscriber subscribe(std::string topic, int queue_size, void (*fp) (T));
	template<class T>
	Publisher advertise(std::string topic, int queue_size);
};

#define ROS_RATE_RATE_SEC_UNIT	1000
class Rate{
public:
	Rate(int rate)
	{
		this->rate = rate;
	};
	void sleep(void);
private:
	int rate;
};

void spin(void);

}

namespace message_traits
{
	template <int V>
	struct MD5Sum{static const char* value();};

	template <class T>
	struct DataType{static const char* value();};

	template <class T>
	struct DataTypeId{static int value(void);};

	template <class T>
	struct Definition{static const char* value();};
}

namespace subtask_methods
{
	template <int T>
	struct CallCallbackFuncs{static void call(void (*fp)(), char *rbuf, int len);};
}

#include "mros_log.h"

#endif /* _ROS_H_ */
