catkin config --cmake-args -DEXTRA_RTC_DIRS="`pwd`/rtc/WholeBodyMasterSlave;`pwd`/rtc/HapticController" -DEXTRA_IDL_FILES="`pwd`/idl/WholeBodyMasterSlaveService.idl;`pwd`/idl/HapticControllerService.idl" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON