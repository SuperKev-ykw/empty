################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/TI/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./empty.o" \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./Hardware/AHRS.o" \
"./Hardware/BlueSerial.o" \
"./Hardware/F32C.o" \
"./Hardware/Grayscale.o" \
"./Hardware/HMC5883L.o" \
"./Hardware/Key.o" \
"./Hardware/MPU6050.o" \
"./Hardware/Serial.o" \
"./Hardware/inv_mpu.o" \
"./Hardware/inv_mpu_dmp_motion_driver.o" \
"./Hardware/mpu_port.o" \
"./Hardware/oled.o" \
"./Hardware/mpu6050/inv_mpu.o" \
"./Hardware/mpu6050/inv_mpu_dmp_motion_driver.o" \
"./Hardware/mpu6050/mpu_port.o" \
"./Test/BlueSerial_Test.o" \
"./Test/GY_87_Test.o" \
"./Test/Gimbal_Test.o" \
"./Test/Grayscale_Test.o" \
"./Test/MPU6050_Test.o" \
"./Test/OLED_LED_Test.o" \
"./Test/Serial_Test.o" \
"./Test/Timer_Key_Test.o" \
"./system/Timer.o" \
"./system/delay.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include Hardware/subdir_vars.mk
-include Hardware/mpu6050/subdir_vars.mk
-include Test/subdir_vars.mk
-include system/subdir_vars.mk
-include subdir_rules.mk
-include Hardware/subdir_rules.mk
-include Hardware/mpu6050/subdir_rules.mk
-include Test/subdir_rules.mk
-include system/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
empty.out 

EXE_OUTPUTS__QUOTED += \
"empty.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "empty.out"

# Tool invocations
empty.out: $(OBJS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"D:/TI/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -gdwarf-3 -Wall -Wl,-m"empty.map" -Wl,-i"C:/TI/mspm0_sdk_2_10_00_04/source" -Wl,-i"D:/STUDY/MSPM0G3507/Myworkspace/empty" -Wl,-i"D:/STUDY/MSPM0G3507/Myworkspace/empty/Debug/syscfg" -Wl,-i"D:/TI/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="empty_linkInfo.xml" -Wl,--rom_model -o "empty.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "empty.o" "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "Hardware\AHRS.o" "Hardware\BlueSerial.o" "Hardware\F32C.o" "Hardware\Grayscale.o" "Hardware\HMC5883L.o" "Hardware\Key.o" "Hardware\MPU6050.o" "Hardware\Serial.o" "Hardware\inv_mpu.o" "Hardware\inv_mpu_dmp_motion_driver.o" "Hardware\mpu_port.o" "Hardware\oled.o" "Hardware\mpu6050\inv_mpu.o" "Hardware\mpu6050\inv_mpu_dmp_motion_driver.o" "Hardware\mpu6050\mpu_port.o" "Test\BlueSerial_Test.o" "Test\GY_87_Test.o" "Test\Gimbal_Test.o" "Test\Grayscale_Test.o" "Test\MPU6050_Test.o" "Test\OLED_LED_Test.o" "Test\Serial_Test.o" "Test\Timer_Key_Test.o" "system\Timer.o" "system\delay.o" 
	-$(RM) "empty.d" "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "Hardware\AHRS.d" "Hardware\BlueSerial.d" "Hardware\F32C.d" "Hardware\Grayscale.d" "Hardware\HMC5883L.d" "Hardware\Key.d" "Hardware\MPU6050.d" "Hardware\Serial.d" "Hardware\inv_mpu.d" "Hardware\inv_mpu_dmp_motion_driver.d" "Hardware\mpu_port.d" "Hardware\oled.d" "Hardware\mpu6050\inv_mpu.d" "Hardware\mpu6050\inv_mpu_dmp_motion_driver.d" "Hardware\mpu6050\mpu_port.d" "Test\BlueSerial_Test.d" "Test\GY_87_Test.d" "Test\Gimbal_Test.d" "Test\Grayscale_Test.d" "Test\MPU6050_Test.d" "Test\OLED_LED_Test.d" "Test\Serial_Test.d" "Test\Timer_Key_Test.d" "system\Timer.d" "system\delay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

