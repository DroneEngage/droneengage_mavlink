CXX=g++
CXXARM=/opt/cross-pi-gcc/bin/arm-linux-gnueabihf-g++
CXXARM_ZERO=g++
EXE=de_ardupilot
BIN=bin
INCLUDE= -I ../c_library_v2 -I ../mavlink_sdk 
INCLUDE_ARM_ZERO =  -I ../c_library_v2 -I ../mavlink_sdk
LIBS=  -lpthread -lmavlink_sdk -L ./mavlink_sdk/bin
LIBS_ARM_ZERO = -lpthread -lmavlink_sdk -L ./mavlink_sdk/bin

CXXFLAGS =  -std=c++11 -Wno-return-type -Wno-address-of-packed-member 
CXXFLAGS_RELEASE= $(CXXFLAGS) -DRELEASE -s
CXXFLAGS_DEBUG= $(CXXFLAGS)  -DDEBUG -g  
SRC = src
BUILD = build

OBJS = $(BUILD)/uavos_module.o \
	   $(BUILD)/fcb_main.o \
	   $(BUILD)/fcb_modes.o \
	   $(BUILD)/fcb_andruav_message_parser.o \
	   $(BUILD)/fcb_traffic_optimizer.o \
	   $(BUILD)/fcb_facade.o \
	   $(BUILD)/configFile.o \
	   $(BUILD)/udpClient.o \
	   $(BUILD)/fcb_geo_fence_base.o \
	   $(BUILD)/fcb_geo_fence_manager.o \
	   $(BUILD)/fcb_swarm_manager.o \
	   $(BUILD)/missions.o \
	   $(BUILD)/mission_translator.o \
	   $(BUILD)/helpers.o \
	   $(BUILD)/util_rpi.o \
	   $(BUILD)/getopt_cpp.o \
	   $(BUILD)/gps.o \
	   $(BUILD)/main.o \

SRCS = ../$(SRC)/uavos_common/uavos_module.cpp \
	   ../$(SRC)/fcb_main.cpp \
	   ../$(SRC)/fcb_modes.cpp \
	   ../$(SRC)/fcb_andruav_message_parser.cpp \
	   ../$(SRC)/fcb_traffic_optimizer.cpp \
	   ../$(SRC)/fcb_facade.cpp \
	   ../$(SRC)/uavos_common/configFile.cpp \
	   ../$(SRC)/uavos_common/udpClient.cpp \
	   ../$(SRC)/geofence/fcb_geo_fence_base.cpp \
	   ../$(SRC)/geofence/fcb_geo_fence_manager.cpp \
	   ../$(SRC)/fcb_swarm_manager.cpp \
	   ../$(SRC)/mission/missions.cpp \
	   ../$(SRC)/mission/mission_translator.cpp \
	   ../$(SRC)/helpers/helpers.cpp \
	   ../$(SRC)/helpers/util_rpi.cpp \
	   ../$(SRC)/helpers/getopt_cpp.cpp \
	   ../$(SRC)/helpers/gps.cpp \
	   ../$(SRC)/main.cpp \
	   

mavlink_control: mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp
	$(CXX) -g -Wall  $(INCLUDE) mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp -o $(BIN)/$(EXE) $(LIBS)

release: mavlink_sdk.release
	$(CXX)  $(CXXFLAGS_RELEASE)  -O2 -o $(BIN)/$(EXE).so  $(OBJS)   $(LIBS)  ;
	@echo "building finished ..."; 
	@echo "DONE."

debug: mavlink_sdk.debug
	$(CXX) $(CXXFLAGS_DEBUG) -Og -o $(BIN)/$(EXE).so     $(OBJS)   $(LIBS) ;
	@echo "building finished ..."; 
	@echo "DONE."

arm_release: mavlink_sdk.arm.release
	$(CXXARM)  $(CXXFLAGS_RELEASE) -O2 -o $(BIN)/$(EXE)_arm.so   $(OBJS)   $(LIBS) ;
	@echo "building finished ..."; 
	@echo "DONE."


arm_debug: mavlink_sdk.arm.debug
	$(CXXARM)  $(CXXFLAGS_DEBUG) -o $(BIN)/$(EXE)_arm.so   $(OBJS)   $(LIBS) ;
	@echo "building finished ..."; 
	@echo "DONE."


arm_release_zero: mavlink_sdk.arm.release.zero
	$(CXXARM_ZERO)  $(CXXFLAGS_RELEASE) -o $(BIN)/$(EXE).so   $(OBJS)   $(LIBS_ARM_ZERO) ;
	@echo "building finished ..."; 
	@echo "DONE."

mavlink_sdk.release: uavos_ardupilot.release
	@echo "Building MAVlink SDK"
	cd ./mavlink_sdk ; \
	make release ;

mavlink_sdk.debug: uavos_ardupilot.debug
	@echo "Building MAVlink SDK"
	cd ./mavlink_sdk ; \
	make debug ;

mavlink_sdk.arm.release: uavos_ardupilot.arm.release
	@echo "Building MAVlink SDK"
	cd ./mavlink_sdk ; \
	make arm_release ;
	
mavlink_sdk.arm.debug: uavos_ardupilot.arm.debug
	@echo "Building MAVlink SDK"
	cd ./mavlink_sdk ; \
	make arm_debug ;

mavlink_sdk.arm.release.zero: uavos_ardupilot.arm.release.zero
	@echo "Building MAVlink SDK"
	cd ./mavlink_sdk ; \
	make arm_release_zero ;

uavos_ardupilot.release: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXX)   $(CXXFLAGS_RELEASE)  -O2 -c   $(SRCS)  $(INCLUDE)  ; 
	cd .. ; 
	@echo "compliling finished ..."

uavos_ardupilot.debug: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXX)   $(CXXFLAGS_DEBUG) -Og -c  $(SRCS)  $(INCLUDE);
	cd .. ; 
	@echo "compliling finished ..."


uavos_ardupilot.arm.release: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXXARM)   $(CXXFLAGS_RELEASE) -O2 -c   $(SRCS)  $(INCLUDE)  ; 
	cd .. ; 
	@echo "compliling finished ..."

uavos_ardupilot.arm.debug: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXXARM)   $(CXXFLAGS_DEBUG) -c   $(SRCS)  $(INCLUDE)  ; 
	cd .. ; 
	@echo "compliling finished ..."

uavos_ardupilot.arm.release.zero: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXXARM_ZERO)   $(CXXFLAGS_RELEASE) -Wno-psabi  -c   $(SRCS)  $(INCLUDE_ARM_ZERO)  ; 
	cd .. ; 
	@echo "compliling finished ..."

git_submodule:
	git submodule update --init --recursive


copy: clean
	mkdir -p $(BIN); \
	cp config.*.json $(BIN); 
	cp config_*.json $(BIN); 
	@echo "copying finished"

clean:
	rm -rf $(BIN); 
	rm -rf $(BUILD);
	cd ./mavlink_sdk ; \
	make clean ;
	@echo "cleaning finished"

