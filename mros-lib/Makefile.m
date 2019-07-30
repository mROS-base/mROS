ROS_VERSION	:= kinetic
OS_NAME		:= os_asp

MROS_SRC_DIR	:= $(MROS_DIR)/mros-src
MROS_MSG_DIR	:= $(MROS_DIR)/mros-msg

MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/inc
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/config
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/config/os/target/$(OS_NAME)
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/api
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/protocol/cimpl
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/node/cimpl
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/topic/cimpl
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/os/target/$(OS_NAME)
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/comm/cimpl/target/lwip
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/comm/cimpl
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/packet/cimpl
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/packet/template/version/$(ROS_VERSION)
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/packet/cimpl/version/$(ROS_VERSION)
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/protocol/cimpl
MROS_IFLAGS	+= -I$(MROS_SRC_DIR)/transfer/cimpl
MROS_IFLAGS	+= -I$(MROS_MSG_DIR)


VPATH	:= $(MROS_SRC_DIR)/api
VPATH	+= $(MROS_SRC_DIR)/comm/cimpl/target/lwip
VPATH	+= $(MROS_SRC_DIR)/comm/cimpl/target/lwip
VPATH	+= $(MROS_SRC_DIR)/comm/cimpl
VPATH	+= $(MROS_SRC_DIR)/node/cimpl
VPATH	+= $(MROS_SRC_DIR)/lib
VPATH	+= $(MROS_SRC_DIR)/os
VPATH	+= $(MROS_SRC_DIR)/os/target/$(OS_NAME)
VPATH	+= $(MROS_SRC_DIR)/packet/cimpl/version/$(ROS_VERSION)
VPATH	+= $(MROS_SRC_DIR)/protocol/cimpl
VPATH	+= $(MROS_SRC_DIR)/topic/cimpl
VPATH	+= $(MROS_SRC_DIR)/transfer/cimpl
VPATH	+= $(MROS_SRC_DIR)/config
VPATH	+= $(MROS_SRC_DIR)/config/os/target/$(OS_NAME)

MROS_CXX_OBJ := ros.o
MROS_C_OBJ += mros_comm_cimpl.o
MROS_C_OBJ += mros_comm_socket_cimpl.o
MROS_C_OBJ += mros_comm_tcp_client_cimpl.o
MROS_C_OBJ += mros_comm_tcp_client_factory_cimpl.o
MROS_C_OBJ += mros_comm_tcp_server_cimpl.o
MROS_C_OBJ += mros_memory.o
MROS_C_OBJ += mros_wait_queue.o
MROS_C_OBJ += mros_node_cimpl.o
MROS_C_OBJ += mros_exclusive_area.o
MROS_C_OBJ += mros_exclusive_ops.o
MROS_C_OBJ += mros_os.o
MROS_C_OBJ += mros_packet_decoder_cimpl.o
MROS_C_OBJ += mros_packet_encoder_cimpl.o
MROS_C_OBJ += mros_protocol_client_rpc_cimpl.o
MROS_C_OBJ += mros_protocol_master_cimpl.o
MROS_C_OBJ += mros_protocol_operation_cimpl.o
MROS_C_OBJ += mros_protocol_publish_cimpl.o
MROS_C_OBJ += mros_protocol_server_proc_cimpl.o
MROS_C_OBJ += mros_protocol_slave_cimpl.o
MROS_C_OBJ += mros_protocol_subscribe_cimpl.o
MROS_C_OBJ += mros_topic_data_publisher_cimpl.o
MROS_C_OBJ += mros_topic_data_subscriber_cimpl.o
MROS_C_OBJ += mros_topic_cimpl.o
MROS_C_OBJ += mros_topic_connector_cimpl.o
MROS_C_OBJ += mros_topic_connector_factory_cimpl.o
MROS_C_OBJ += mros_name.o
MROS_C_OBJ += mros_os_config.o
MROS_C_OBJ += mros_usr_config.o
MROS_C_OBJ += mros_sys_config.o

MROS_INCLUDE_PATHS += $(MROS_IFLAGS)

