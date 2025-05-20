################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/src/stm32f407xx_gpio_driver.c \
../driver/src/stm32f407xx_i2c_driver.c \
../driver/src/stm32f407xx_spi_driver.c 

OBJS += \
./driver/src/stm32f407xx_gpio_driver.o \
./driver/src/stm32f407xx_i2c_driver.o \
./driver/src/stm32f407xx_spi_driver.o 

C_DEPS += \
./driver/src/stm32f407xx_gpio_driver.d \
./driver/src/stm32f407xx_i2c_driver.d \
./driver/src/stm32f407xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driver/src/%.o driver/src/%.su driver/src/%.cyclo: ../driver/src/%.c driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/jesti/OneDrive/Desktop/Embedded_System_Course/STM32F407G-DISC1_driver/driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-driver-2f-src

clean-driver-2f-src:
	-$(RM) ./driver/src/stm32f407xx_gpio_driver.cyclo ./driver/src/stm32f407xx_gpio_driver.d ./driver/src/stm32f407xx_gpio_driver.o ./driver/src/stm32f407xx_gpio_driver.su ./driver/src/stm32f407xx_i2c_driver.cyclo ./driver/src/stm32f407xx_i2c_driver.d ./driver/src/stm32f407xx_i2c_driver.o ./driver/src/stm32f407xx_i2c_driver.su ./driver/src/stm32f407xx_spi_driver.cyclo ./driver/src/stm32f407xx_spi_driver.d ./driver/src/stm32f407xx_spi_driver.o ./driver/src/stm32f407xx_spi_driver.su

.PHONY: clean-driver-2f-src

