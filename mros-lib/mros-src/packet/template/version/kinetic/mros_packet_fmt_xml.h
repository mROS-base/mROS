#ifndef _MROS_PACKET_FMT_XML_H_
#define _MROS_PACKET_FMT_XML_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MROS_PACKET_XMLRPC_REQ_END_STR		"</methodCall>"
#define MROS_PACKET_XMLRPC_RES_END_STR		"</methodResponse>"

/*
 * ARGS1: method name
 * ARGS2: node name
 * ARGS3: topic
 * ARGS4: type
 * ARGS5: uri
 */
#define MROS_PACKET_FMT_XML_REGISTER_REQ		\
	"<?xml version='1.0'?>\n"					\
	"<methodCall>\n"						\
		"<methodName>%s</methodName>\n"		\
		"<params>\n"						\
			"<param>\n"						\
				"<value>%s</value>\n"		\
			"</param>\n"					\
			"<param>\n"						\
				"<value>%s</value>\n"		\
			"</param>\n"					\
			"<param>\n"						\
				"<value>%s</value>\n"		\
			"</param>\n"					\
			"<param>\n"						\
				"<value>%s</value>\n"		\
			"</param>\n"					\
		"</params>"							\
	"</methodCall>\n"

/*
 * ARGS1: method name
 * ARGS2: node name
 * ARGS3: topic
 * ARGS4: tcpros
 */
#define MROS_PACKET_FMT_XML_REQUEST_TOPIC_REQ				\
	"<?xml version='1.0'?>\n"								\
	"<methodCall>\n"										\
		"<methodName>%s</methodName>\n"						\
		"<params>\n"										\
			"<param>\n"										\
				"<value>%s</value>\n"						\
			"</param>\n"									\
			"<param>\n"										\
				"<value>%s</value>\n"						\
			"</param>\n"									\
			"<param>\n"										\
				"<value>"									\
					"<array>\n"								\
						"<data>"							\
							"<value>"						\
								"<array>\n"					\
									"<data>"				\
										"<value>%s</value>"	\
									"</data>\n"				\
								"</array>"					\
							"</value>"						\
						"</data>\n"							\
					"</array>"								\
				"</value>\n"								\
			"</param>\n"									\
		"</params>"											\
	"</methodCall>\n"										\

/*
 * ARGS1: TCPROS
 * ARGS2: ipaddr
 * ARGS3: port
 */
#define MROS_PACKET_FMT_XML_REQUEST_TOPIC_RES		\
	"<?xml version='1.0'?>\n"		\
	"<methodResponse>\n"	\
		"<params>\n"	\
			"<param>\n"	\
				"<value>" \
					"<array>" \
						"<data>" \
							"<value>" \
								"<i4>1</i4>"	\
							"</value>\n"	\
							"<value></value>\n"	\
							"<value>"	\
								"<array>\n"	\
									"<data>" \
										"<value>%s</value>\n"	\
										"<value>%s</value>\n"	\
										"<value><i4>%u</i4></value>\n"	\
									"</data>"	\
								"</array>"	\
							"</value>\n"	\
						"</data>"	\
					"</array>"	\
				"</value>"	\
			"</param>\n"	\
		"</params>\n"	\
	"</methodResponse>\n"	\


#ifdef __cplusplus
}
#endif

#endif /* _MROS_PACKET_FMT_XML_H_ */
