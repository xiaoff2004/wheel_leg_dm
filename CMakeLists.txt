#此文件从模板自动生成! 请勿更改!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.27)



# project settings
project(H7_TEST C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)



include_directories(Core/Inc
        Drivers/STM32H7xx_HAL_Driver/Inc
        Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32H7xx/Include
        Drivers/CMSIS/Include User
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
        User/Algorithm/EKF
        User/Algorithm/kalman
        User/Algorithm/mahony
        User/Algorithm/PID
        User/Algorithm/VMC
        User/APP
        User/Bsp
        User/Controller
        User/Driver
        )

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -DARM_MATH_CM7)

file(GLOB_RECURSE SOURCES
        "Core/*.*"
        "Middlewares/*.*"
        "Drivers/*.*"
        "USB_DEVICE/*.*"
        "User/Algorithm/*.*"
        "User/APP/*.*"
        "User/Bsp/*.*"
        "User/Controller/*.*"
        "User/Driver/*.*"
        )


