CXX=g++
EXE=uavos_ardupilot
BIN=bin
INCLUDE= -I ../c_library_v2
LIBS= -lpthread
CXXFLAGS= 
CXXFLAGS_DEBUG= -DDEBUG -g
SRC = src
BUILD = build

OBJS = $(BUILD)/plugin.o \
	   $(BUILD)/autopilot_interface.o \
	   $(BUILD)/serial_port.o \
	   $(BUILD)/udp_port.o 

SRCS = ../$(SRC)/plugin.cpp \
	   ../$(SRC)/autopilot_interface.cpp \
	   ../$(SRC)/serial_port.cpp \
	   ../$(SRC)/udp_port.cpp 


all: git_submodule release


mavlink_control: mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp
	$(CXX) -g -Wall  $(INCLUDE) mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp -o $(BIN)/$(EXE) $(LIBS)



release: uavos_ardupilot.release
	$(CXX)  -o $(BIN)/$(EXE).so  $(CXXFLAGS)  $(OBJS)   $(LIBS)  ;
	@echo "building finished ..."; 
	@echo "DONE."

debug: uavos_ardupilot.debug


uavos_ardupilot.release: copy
	mkdir -p $(BUILD); \
	cd $(BUILD); \
	$(CXX)   -c   $(SRCS)  $(INCLUDE)  ; 
	cd .. ; 
	@echo "compliling finished ..."

uavos_ardupilot.debug: copy




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

