CXX=g++
EXE=uavos_ardupilot
BIN=bin
INCLUDE= -I ../c_library_v2 -I ../mavlink_sdk
LIBS= -lpthread
CXXFLAGS = 
CXXFLAGS_RELEASE= $(CXXFLAGS) 
CXXFLAGS_DEBUG= $(CXXFLAGS)  -DDEBUG -g
SRC = src
BUILD = build

OBJS = $(BUILD)/mavlink_sdk.o \
	   $(BUILD)/mavlink_communicator.o \
	   $(BUILD)/serial_port.o \
	   $(BUILD)/udp_port.o \
	   $(BUILD)/andruav_facade.o \
	   $(BUILD)/andruav_message_parser.o \
	   $(BUILD)/andruav_traffic_optimizer.o \
	   $(BUILD)/plugin.o \

SRCS = ../mavlink_sdk/mavlink_sdk.cpp \
	   ../mavlink_sdk/mavlink_communicator.cpp \
	   ../mavlink_sdk/serial_port.cpp \
	   ../mavlink_sdk/udp_port.cpp \
	   ../$(SRC)/andruav_facade.cpp \
	   ../$(SRC)/andruav_message_parser.cpp \
	   ../$(SRC)/andruav_traffic_optimizer.cpp \
	   ../$(SRC)/plugin.cpp \
	   


all: git_submodule release


mavlink_control: mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp
	$(CXX) -g -Wall  $(INCLUDE) mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp -o $(BIN)/$(EXE) $(LIBS)



release: uavos_ardupilot.release
	$(CXX)  -o $(BIN)/$(EXE).so  $(CXXFLAGS_RELEASE)  $(OBJS)   $(LIBS)  ;
	@echo "building finished ..."; 
	@echo "DONE."

debug: uavos_ardupilot.debug
	$(CXX)  -o $(BIN)/$(EXE).so  $(CXXFLAGS_DEBUG)  $(OBJS)   $(LIBS)  ;
	@echo "building finished ..."; 
	@echo "DONE."

uavos_ardupilot.release: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXX)   -c   $(SRCS)  $(INCLUDE)  -Waddress-of-packed-member ; 
	cd .. ; 
	@echo "compliling finished ..."

uavos_ardupilot.debug: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXX)   -c   $(SRCS)  $(INCLUDE)  ; 
	cd .. ; 
	@echo "compliling finished ..."



git_submodule:
	git submodule update --init --recursive


copy: clean
	mkdir -p $(BIN);
	#cp config.*.json $(BIN); 
	@echo "copying finished"

clean:
	rm -rf $(BIN); 
	rm -rf $(BUILD);
	@echo "cleaning finished"

