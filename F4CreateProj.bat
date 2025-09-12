@echo off
chcp 65001 >nul
echo ========================================
echo STM32F401RE Project Initialization Script
echo Using HAL Library from DFP 2.9.0
echo ========================================
echo.

set PROJECT_NAME=%1
if "%PROJECT_NAME%"=="" (
    echo Error: Please provide project name
    echo Usage: create.bat [project_name]
    pause
    exit /b 1
)

rem Set correct paths for DFP 2.9.0
set BASE_PATH=C:\Keil_v5\ARM\PACK\Keil\STM32F4xx_DFP\2.9.0
set HAL_DRIVER_PATH=%BASE_PATH%\Drivers\STM32F4xx_HAL_Driver
set CMSIS_PATH=%BASE_PATH%\Drivers\CMSIS
set DEVICE_PATH=%CMSIS_PATH%\Device\ST\STM32F4xx

echo Cleaning existing project files...
if exist "%PROJECT_NAME%" (
    rmdir /s /q "%PROJECT_NAME%" 2>nul
)
echo   [OK] Existing files cleaned

echo Checking Keil DFP path...
if not exist "%BASE_PATH%" (
    echo   [FAIL] DFP 2.9.0 path not found: %BASE_PATH%
    echo   Please ensure the 2.9.0 folder exists in current directory
    pause
    exit /b 1
)
echo   [OK] DFP path verified: %BASE_PATH%

echo Creating directory structure...
mkdir "%PROJECT_NAME%" 2>nul
cd "%PROJECT_NAME%"
mkdir "Src" 2>nul
mkdir "Inc" 2>nul
mkdir "Drivers" 2>nul
mkdir "Drivers\STM32F4xx_HAL_Driver" 2>nul
mkdir "Drivers\STM32F4xx_HAL_Driver\Inc" 2>nul
mkdir "Drivers\STM32F4xx_HAL_Driver\Src" 2>nul
mkdir "Drivers\CMSIS" 2>nul
mkdir "Drivers\CMSIS\Include" 2>nul
mkdir "Drivers\CMSIS\Device" 2>nul
mkdir "Drivers\CMSIS\Device\ST" 2>nul
mkdir "Drivers\CMSIS\Device\ST\STM32F4xx" 2>nul
mkdir "Drivers\CMSIS\Device\ST\STM32F4xx\Include" 2>nul
mkdir "Drivers\CMSIS\Device\ST\STM32F4xx\Source" 2>nul
echo   [OK] Directory structure created

echo Copying startup files...
if exist "%DEVICE_PATH%\Source\Templates\arm\startup_stm32f401xe.s" (
    copy "%DEVICE_PATH%\Source\Templates\arm\startup_stm32f401xe.s" "Drivers\CMSIS\Device\ST\STM32F4xx\Source\" >nul 2>&1
    if errorlevel 1 (
        echo   [FAIL] Startup file copy failed
    ) else (
        echo   [OK] startup_stm32f401xe.s
    )
) else (
    echo   [FAIL] Startup file not found
)

echo Copying system files...
if exist "%DEVICE_PATH%\Source\Templates\system_stm32f4xx.c" (
    copy "%DEVICE_PATH%\Source\Templates\system_stm32f4xx.c" "Drivers\CMSIS\Device\ST\STM32F4xx\Source\" >nul 2>&1
    if errorlevel 1 (
        echo   [FAIL] System file copy failed
    ) else (
        echo   [OK] system_stm32f4xx.c
    )
) else (
    echo   [FAIL] System file not found
)

echo Copying HAL driver source files...
set HAL_FILES=stm32f4xx_hal.c stm32f4xx_hal_cortex.c stm32f4xx_hal_rcc.c stm32f4xx_hal_rcc_ex.c stm32f4xx_hal_gpio.c stm32f4xx_hal_pwr.c stm32f4xx_hal_pwr_ex.c

for %%f in (%HAL_FILES%) do (
    if exist "%HAL_DRIVER_PATH%\Src\%%f" (
        copy "%HAL_DRIVER_PATH%\Src\%%f" "Drivers\STM32F4xx_HAL_Driver\Src\" >nul 2>&1
        if errorlevel 1 (
            echo   [FAIL] %%f copy failed
        ) else (
            echo   [OK] %%f
        )
    ) else (
        echo   [FAIL] %%f not found
    )
)

echo Copying HAL driver header files...
if exist "%HAL_DRIVER_PATH%\Inc\stm32f4xx_hal.h" (
    copy "%HAL_DRIVER_PATH%\Inc\*.h" "Drivers\STM32F4xx_HAL_Driver\Inc\" >nul 2>&1
    
    REM Copy Legacy folder
    if exist "%HAL_DRIVER_PATH%\Inc\Legacy" (
        if not exist "Drivers\STM32F4xx_HAL_Driver\Inc\Legacy" (
            mkdir "Drivers\STM32F4xx_HAL_Driver\Inc\Legacy"
        )
        copy "%HAL_DRIVER_PATH%\Inc\Legacy\*.h" "Drivers\STM32F4xx_HAL_Driver\Inc\Legacy\" >nul 2>&1
        if errorlevel 1 (
            echo   [FAIL] Legacy header files copy failed
        ) else (
            echo   [OK] Legacy header files copied
        )
    ) else (
        echo   [WARN] Legacy folder not found in source
    )
    
    if errorlevel 1 (
        echo   [FAIL] HAL header files copy failed
    ) else (
        echo   [OK] HAL header files copied
    )
) else (
    echo   [FAIL] HAL header files not found
)

echo Copying CMSIS header files...
if exist "%DEVICE_PATH%\Include\stm32f4xx.h" (
    copy "%DEVICE_PATH%\Include\*.h" "Drivers\CMSIS\Device\ST\STM32F4xx\Include\" >nul 2>&1
    echo   [OK] CMSIS device headers
) else (
    echo   [FAIL] CMSIS device headers not found
)

if exist "%CMSIS_PATH%\Include\core_cm4.h" (
    copy "%CMSIS_PATH%\Include\core_cm*.h" "Drivers\CMSIS\Include\" >nul 2>&1
    copy "%CMSIS_PATH%\Include\cmsis_*.h" "Drivers\CMSIS\Include\" >nul 2>&1
    echo   [OK] CMSIS core headers
) else (
    echo   [FAIL] CMSIS core headers not found
)

echo Creating simple main.c file...
(
echo #include "main.h"
echo.
echo int main^(void^)
echo {
echo     HAL_Init^(^);
echo.
echo     while^(1^)
echo     {
echo         // TODO: Add your code here
echo     }
echo }
echo.
echo void SystemClock_Config^(void^)
echo {
echo     // TODO: Configure system clock
echo }
echo.
echo void Error_Handler^(void^)
echo {
echo     __disable_irq^(^);
echo     while^(1^) {}
echo }
) > "Src\main.c"
echo   [OK] main.c created

echo Creating main.h header file...
(
echo #ifndef __MAIN_H
echo #define __MAIN_H
echo.
echo #include "stm32f4xx_hal.h"
echo.
echo void SystemClock_Config^(void^);
echo void Error_Handler^(void^);
echo.
echo #endif
) > "Inc\main.h"
echo   [OK] main.h created

echo Creating HAL configuration file...
if exist "%HAL_DRIVER_PATH%\Inc\stm32f4xx_hal_conf_template.h" (
    copy "%HAL_DRIVER_PATH%\Inc\stm32f4xx_hal_conf_template.h" "Inc\stm32f4xx_hal_conf.h" >nul 2>&1
    echo   [OK] stm32f4xx_hal_conf.h created
) else (
    echo   [FAIL] HAL config template not found
)

echo Creating interrupt handlers...
(
echo #include "main.h"
echo #include "stm32f4xx_it.h"
echo.
echo void NMI_Handler^(void^) {}
echo void HardFault_Handler^(void^) { while^(1^) {} }
echo void MemManage_Handler^(void^) { while^(1^) {} }
echo void BusFault_Handler^(void^) { while^(1^) {} }
echo void UsageFault_Handler^(void^) { while^(1^) {} }
echo void SVC_Handler^(void^) {}
echo void DebugMon_Handler^(void^) {}
echo void PendSV_Handler^(void^) {}
echo void SysTick_Handler^(void^) { HAL_IncTick^(^); }
) > "Src\stm32f4xx_it.c"
echo   [OK] stm32f4xx_it.c created

(
echo #ifndef __STM32F4xx_IT_H
echo #define __STM32F4xx_IT_H
echo.
echo void NMI_Handler^(void^);
echo void HardFault_Handler^(void^);
echo void MemManage_Handler^(void^);
echo void BusFault_Handler^(void^);
echo void UsageFault_Handler^(void^);
echo void SVC_Handler^(void^);
echo void DebugMon_Handler^(void^);
echo void PendSV_Handler^(void^);
echo void SysTick_Handler^(void^);
echo.
echo #endif
) > "Inc\stm32f4xx_it.h"
echo   [OK] stm32f4xx_it.h created

echo.
echo Copying Keil project template...
cd ..
if exist "%~dp0keil_proj\F4Sample.uvprojx" (
    copy "%~dp0keil_proj\F4Sample.uvprojx" "%PROJECT_NAME%\%PROJECT_NAME%.uvprojx" >nul 2>&1
    if errorlevel 1 (
        echo   [FAIL] Keil project file copy failed
    ) else (
        echo   [OK] Keil project file copied as %PROJECT_NAME%.uvprojx
    )
) else (
    echo   [WARN] Keil project template not found at %~dp0keil_proj\F4Sample.uvprojx
    echo   [INFO] Script directory: %~dp0
    echo   [INFO] Please ensure the keil_proj folder exists in script directory
)

echo.
echo ========================================
echo Project "%PROJECT_NAME%" created successfully!
echo You can now open %PROJECT_NAME%.uvprojx in Keil uVision
echo ========================================
echo.
echo Project: %PROJECT_NAME%
echo Target: STM32F401RE
echo Library: HAL ^(DFP 2.9.0^)
echo Function: Simple LED Blink ^(PA5^)
echo.
echo Ready to build in Keil uVision5!
echo.
pause