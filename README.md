# Синтезатор для лампового приёмника
Синтезатор выполнен на микросхеме SI5351 и микроконтроллере
stm32f030k6t6.
Код написан на C с использованием библиотеки LibopenCM3, 
для управления SI5351 портирована библиотека
https://github.com/bob-01/STM32-SI5351.


# Instructions
 
 1. $sudo pacman -S openocd arm-none-eabi-binutils arm-none-eabi-gcc arm-none-eabi-newlib arm-none-eabi-gdb
 2. $git clone https://github.com/5881/tube_fm_si5351.git
 3. $cd tube_fm_si5351
 4. $git submodule update --init # (Only needed once)
 5. $TARGETS=stm32/f0 make -C libopencm3 # (Only needed once)
 6. $make 
 7. $make flash

Александр Белый 2020
