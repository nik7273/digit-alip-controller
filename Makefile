COMMON        = -O2 -I../include -L../bin -std=c++14 -pthread -Wl,-rpath,'$$ORIGIN'
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
