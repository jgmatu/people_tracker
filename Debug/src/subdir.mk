################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Borders.cpp \
../src/HSVHistogram.cpp \
../src/Pblob.cpp \
../src/PlobPerception.cpp \
../src/PlobTracker.cpp \
../src/Slice.cpp \
../src/TestMain.cpp \
../src/people_tracker_node.cpp 

OBJS += \
./src/Borders.o \
./src/HSVHistogram.o \
./src/Pblob.o \
./src/PlobPerception.o \
./src/PlobTracker.o \
./src/Slice.o \
./src/TestMain.o \
./src/people_tracker_node.o 

CPP_DEPS += \
./src/Borders.d \
./src/HSVHistogram.d \
./src/Pblob.d \
./src/PlobPerception.d \
./src/PlobTracker.d \
./src/Slice.d \
./src/TestMain.d \
./src/people_tracker_node.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/opt/ros/indigo/include -I/usr/include/pcl-1.7 -I/usr/include/eigen3/ -I/home/javi/catkin_ws/devel/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


