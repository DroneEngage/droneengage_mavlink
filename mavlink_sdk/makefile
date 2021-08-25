CXX=g++
CXXARM=/opt/cross-pi-gcc/bin/arm-linux-gnueabihf-g++
EXE=libmavlink_sdk
BIN=bin
INCLUDE= -I ../../c_library_v2 -I ../mavlink_sdk
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


SRCS = ../mavlink_sdk.cpp \
	   ../mavlink_waypoint_manager.cpp \
	   ../mavlink_communicator.cpp \
	   ../mavlink_command.cpp \
	   ../serial_port.cpp \
	   ../udp_port.cpp \
	   ../vehicle.cpp \
	   ../mavlink_helper.cpp \
	   


all: git_submodule release


release: mavlink_sdk.release
	cd $(BUILD) ; \
	ar r $(EXE).a *.o ; \
	mv *.a ../$(BIN) ;
	cd .. ;
	pwd ;
	@echo "building finished ..."; 
	@echo "DONE."

debug: mavlink_sdk.debug
	cd $(BUILD) ; \
	ar r $(EXE).a *.o ; \
	mv *.a ../$(BIN) ;
	cd .. ;
	pwd ;
	@echo "building finished ..."; 
	@echo "DONE."

arm_debug: mavlink_sdk.arm.debug
	cd $(BUILD) ; \
	ar r $(EXE).a *.o ; \
	mv *.a ../$(BIN) ;
	cd .. ;
	pwd ;
	@echo "building finished ..."; 
	@echo "DONE."

mavlink_sdk.release: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXX)   $(CXXFLAGS_RELEASE)  -c   $(SRCS)  $(INCLUDE)  ; \
	cd .. ; 
	@echo "compliling finished ..."


mavlink_sdk.debug: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXX)   $(CXXFLAGS_DEBUG)  -c  $(SRCS)  $(INCLUDE);
	cd .. ; 
	@echo "compliling finished ..."


mavlink_sdk.arm.debug: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXXARM)   $(CXXFLAGS_DEBUG) -g -DDEBUG -c   $(SRCS)  $(INCLUDE)  ; 
	cd .. ; 
	@echo "compliling finished ..."



git_submodule:
	git submodule update --init --recursive


copy: clean
	mkdir -p $(BIN); \
	#config.*.json $(BIN); 
	#cp config_*.json $(BIN); 
	@echo "copying finished"

clean:
	rm -rf $(BIN); 
	rm -rf $(BUILD);
	@echo "cleaning finished"





#https://stackoverflow.com/questions/2206128/how-to-call-makefile-from-another-makefile