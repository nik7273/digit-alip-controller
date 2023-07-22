COMMON        = -O2 -I../include -L../bin -std=c++14 -pthread -Wl,-rpath,'$$ORIGIN'
PYBIND_COMMON = -O3 -Wall -shared -std=c++11 -fPIC 
PYBIND_INCLUDE = -Iextern/pybind11/include
INIT_DIR    = -c -Iinclude/controller
CTRL_DIR    = -Iinclude/controller
INCLUDE_DIR = -Iinclude
LLAPI         = agility/

init:
	g++ $(COMMON) $(INIT_DIR) $(INCLUDE_DIR) -I$(LLAPI) src/controller/Digit_Controller.cpp src/controller/utils.cpp src/gen/dyn/*.cc src/gen/kin/*.cc
	mv *.o build

controller:
	g++ $(COMMON) $(INIT_DIR) $(INCLUDE_DIR) -I$(LLAPI) src/controller/Digit_Controller.cpp
	mv *.o build

pybind:
	g++ $(COMMON) $(INIT_DIR) $(INCLUDE_DIR) -I$(LLAPI) src/controller/Digit_Controller.cpp
	c++ $(PYBIND_COMMON) $(shell python3-config --includes) $(PYBIND_INCLUDE) $(INCLUDE_DIR) $(INIT_DIR) -I$(LLAPI) src/pybind/digit_controller_pybind.cpp Digit_Controller.o -o digit_controller_pybind$(shell python3-config --extension-suffix)
	mv *.o build
	mv *.so build
