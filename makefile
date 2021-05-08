CXX=g++
CXXARM=/opt/cross-pi-gcc/bin/arm-linux-gnueabihf-g++
EXE=uavos_ardupilot
BIN=bin
INCLUDE= -I ../c_library_v2 -I ../mavlink_sdk
LIBS=  -lpthread 
CXXFLAGS =  -std=c++11 -Wno-return-type -Wno-address-of-packed-member 

CXXFLAGS_RELEASE= $(CXXFLAGS) -DRELEASE -s
CXXFLAGS_DEBUG= $(CXXFLAGS)  -DDEBUG -g  
SRC = src
BUILD = build

OBJS = $(BUILD)/mavlink_sdk.o \
	   $(BUILD)/mavlink_waypoint_manager.o \
	   $(BUILD)/mavlink_communicator.o \
	   $(BUILD)/mavlink_command.o \
	   $(BUILD)/serial_port.o \
	   $(BUILD)/udp_port.o \
	   $(BUILD)/vehicle.o \
	   $(BUILD)/mavlink_helper.o \
	   $(BUILD)/fcb_main.o \
	   $(BUILD)/fcb_modes.o \
	   $(BUILD)/fcb_andruav_message_parser.o \
	   $(BUILD)/fcb_traffic_optimizer.o \
	   $(BUILD)/fcb_facade.o \
	   $(BUILD)/configFile.o \
	   $(BUILD)/udpClient.o \
	   $(BUILD)/missions.o \
	   $(BUILD)/mission_translator.o \
	   $(BUILD)/helpers.o \
	   $(BUILD)/main.o \

SRCS = ../mavlink_sdk/mavlink_sdk.cpp \
	   ../mavlink_sdk/mavlink_waypoint_manager.cpp \
	   ../mavlink_sdk/mavlink_communicator.cpp \
	   ../mavlink_sdk/mavlink_command.cpp \
	   ../mavlink_sdk/serial_port.cpp \
	   ../mavlink_sdk/udp_port.cpp \
	   ../mavlink_sdk/vehicle.cpp \
	   ../mavlink_sdk/mavlink_helper.cpp \
	   ../$(SRC)/fcb_main.cpp \
	   ../$(SRC)/fcb_modes.cpp \
	   ../$(SRC)/fcb_andruav_message_parser.cpp \
	   ../$(SRC)/fcb_traffic_optimizer.cpp \
	   ../$(SRC)/fcb_facade.cpp \
	   ../$(SRC)/configFile.cpp \
	   ../$(SRC)/udpClient.cpp \
	   ../$(SRC)/mission/missions.cpp \
	   ../$(SRC)/mission/mission_translator.cpp \
	   ../$(SRC)/helpers/helpers.cpp \
	   ../$(SRC)/main.cpp \
	   


all: git_submodule release


mavlink_control: mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp
	$(CXX) -g -Wall  $(INCLUDE) mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp -o $(BIN)/$(EXE) $(LIBS)



release: uavos_ardupilot.release
	$(CXX)  $(CXXFLAGS_RELEASE)  -o $(BIN)/$(EXE).so  $(OBJS)   $(LIBS)  ;
	@echo "building finished ..."; 
	@echo "DONE."

debug: uavos_ardupilot.debug
	$(CXX) $(CXXFLAGS_DEBUG) -o $(BIN)/$(EXE).so     $(OBJS)   $(LIBS) ;
	@echo "building finished ..."; 
	@echo "DONE."

arm_debug: uavos_ardupilot.arm.debug
	$(CXXARM)  $(CXXFLAGS_DEBUG) -o $(BIN)/$(EXE)_arm.so   $(OBJS)   $(LIBS) ;
	@echo "building finished ..."; 
	@echo "DONE."

uavos_ardupilot.release: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXX)   $(CXXFLAGS_RELEASE)  -c   $(SRCS)  $(INCLUDE)  ; 
	cd .. ; 
	@echo "compliling finished ..."

uavos_ardupilot.debug: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXX)   $(CXXFLAGS_DEBUG)  -c  $(SRCS)  $(INCLUDE);
	cd .. ; 
	@echo "compliling finished ..."


uavos_ardupilot.arm.debug: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXXARM)   $(CXXFLAGS_DEBUG) -g -DDEBUG -c   $(SRCS)  $(INCLUDE)  ; 
	cd .. ; 
	@echo "compliling finished ..."



git_submodule:
	git submodule update --init --recursive


copy: clean
	mkdir -p $(BIN); \
	cp config.*.json $(BIN); 
	@echo "copying finished"

clean:
	rm -rf $(BIN); 
	rm -rf $(BUILD);
	@echo "cleaning finished"

