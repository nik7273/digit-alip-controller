COMMON        = -O2 -I../include -L../bin -std=c++14 -fPIC -pthread -Wl,-rpath,'$$ORIGIN' -c
PYBIND_COMMON = -O2 -Wall -shared -std=c++14 -fPIC
PYBIND_INCLUDE = -Iextern/pybind11/include
INIT_DIR    = -Iinclude/controller
CTRL_DIR    = -Iinclude/controller
INCLUDE_DIR = -Iinclude
LLAPI         = -Iagility/

init:
	g++ $(COMMON) $(INIT_DIR) $(INCLUDE_DIR) $(LLAPI) src/controller/Digit_Controller.cpp src/controller/utils.cpp src/gen/dyn/*.cc src/gen/kin/*.cc
	mv *.o build

controller:
	g++ $(COMMON) $(INIT_DIR) $(INCLUDE_DIR) $(LLAPI) src/controller/Digit_Controller.cpp
	mv *.o build

pybind:
	g++ $(PYBIND_COMMON) $(shell python3-config --includes) $(PYBIND_INCLUDE) $(INCLUDE_DIR) $(INIT_DIR) $(LLAPI) build/* src/pybind/digit_controller_pybind.cpp -o digit_controller_pybind$(shell python3-config --extension-suffix)
	find . -maxdepth 1 -name "*.o" -exec mv {} build/ \; # move .o files if they exist
	mv digit_controller_pybind$(shell python3-config --extension-suffix) example
	find . -maxdepth 1 -name "*.so" -exec mv {} build/ \; # move .so files if they exist

all:
	g++ $(COMMON) $(INIT_DIR) $(INCLUDE_DIR) $(LLAPI) src/controller/Digit_Controller.cpp src/controller/utils.cpp src/gen/dyn/*.cc src/gen/kin/*.cc
	mv *.o build
	g++ $(PYBIND_COMMON) $(shell python3-config --includes) $(PYBIND_INCLUDE) $(INCLUDE_DIR) $(INIT_DIR) $(LLAPI) build/* src/pybind/digit_controller_pybind.cpp -o digit_controller_pybind$(shell python3-config --extension-suffix)
	mv *.o build
	mv digit_controller_pybind$(shell python3-config --extension-suffix) example
	mv *.so build
