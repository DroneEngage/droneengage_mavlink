CXX=g++
CXXARM=/opt/cross-pi-gcc/bin/arm-linux-gnueabihf-g++
EXE=uavos_ardupilot
BIN=bin
INCLUDE= -I ../c_library_v2 -I ../mavlink_sdk
LIBS=  -lpthread -lmavlink_sdk -L ./mavlink_sdk/bin
CXXFLAGS =  -std=c++11 -Wno-return-type -Wno-address-of-packed-member 

CXXFLAGS_RELEASE= $(CXXFLAGS) -DRELEASE -s
CXXFLAGS_DEBUG= $(CXXFLAGS)  -DDEBUG -g  
SRC = src
BUILD = build

OBJS = $(BUILD)/fcb_main.o \
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

SRCS = ../$(SRC)/fcb_main.cpp \
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
	   

mavlink_control: mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp
	$(CXX) -g -Wall  $(INCLUDE) mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp -o $(BIN)/$(EXE) $(LIBS)

release: mavlink_sdk.release
	$(CXX)  $(CXXFLAGS_RELEASE)  -o $(BIN)/$(EXE).so  $(OBJS)   $(LIBS)  ;
	@echo "building finished ..."; 
	@echo "DONE."

debug: mavlink_sdk.debug
	$(CXX) $(CXXFLAGS_DEBUG) -o $(BIN)/$(EXE).so     $(OBJS)   $(LIBS) ;
	@echo "building finished ..."; 
	@echo "DONE."

arm_release: mavlink_sdk.arm.release
	$(CXXARM)  $(CXXFLAGS_RELEASE) -o $(BIN)/$(EXE)_arm.so   $(OBJS)   $(LIBS) ;
	@echo "building finished ..."; 
	@echo "DONE."


arm_debug: mavlink_sdk.arm.debug
	$(CXXARM)  $(CXXFLAGS_DEBUG) -o $(BIN)/$(EXE)_arm.so   $(OBJS)   $(LIBS) ;
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


uavos_ardupilot.arm.release: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXXARM)   $(CXXFLAGS_RELEASE) -c   $(SRCS)  $(INCLUDE)  ; 
	cd .. ; 
	@echo "compliling finished ..."

uavos_ardupilot.arm.debug: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXXARM)   $(CXXFLAGS_DEBUG) -c   $(SRCS)  $(INCLUDE)  ; 
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
	@echo "cleaning finished"

