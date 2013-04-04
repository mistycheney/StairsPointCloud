################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/StairsPointCloud.cpp \
../src/canny.cpp \
../src/disparityMap.cpp 

OBJS += \
./src/StairsPointCloud.o \
./src/canny.o \
./src/disparityMap.o 

CPP_DEPS += \
./src/StairsPointCloud.d \
./src/canny.d \
./src/disparityMap.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/Users/yuncong/pcl-trunk/release/include/pcl-1.7 -I/opt/local/include/vtk-5.10 -I/Users/yuncong/OpenCV-2.4.3/release/include -I/usr/local/Cellar/eigen/3.1.2/include/eigen3 -I/usr/local/Cellar/boost/1.52.0/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


