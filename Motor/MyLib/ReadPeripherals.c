#include "main.h"
#include "GY86.h"
#include "OLED.h"
#include "UART_Set.h"
#include <stdio.h>
#include <string.h>

typedef enum { SENSOR_MPU = 0, SENSOR_HMC = 1, SENSOR_BARO = 2 } SensorType;

static SensorType current_sensor = SENSOR_MPU;
static uint32_t last_switch_ms = 0;
static const uint32_t SAMPLE_PERIOD_MS = 500;

void ReadPeripherals_Init(void) {
    OLED_Init();
    MPU6050_Init();
    HMC5883L_Init();
    MS561101BA_Init();
    uint8_t mpu_id = MPU6050_GetId();
//    OLED_ShowString(1, 1, "ID: ");
//    OLED_ShowHexNum(1, 5, mpu_id, 4);
    last_switch_ms = HAL_GetTick();
}

void ReadPeripherals_Process(void) {
    uint32_t now = HAL_GetTick();
    char buffer[128];

    if (now - last_switch_ms >= SAMPLE_PERIOD_MS) {
        last_switch_ms = now;
        // OLED_Clear();
        switch (current_sensor) {
            case SENSOR_MPU: {
                int16_t ax, ay, az, gx, gy, gz;
                MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
                
                // OLED_ShowString(1, 1, "MPU6050");
                // OLED_ShowString(2, 1, "AX:"); OLED_ShowSignedNum(2, 4, ax, 6);
                // OLED_ShowString(3, 1, "AY:"); OLED_ShowSignedNum(3, 4, ay, 6);
                // OLED_ShowString(4, 1, "AZ:"); OLED_ShowSignedNum(4, 4, az, 6);
                // OLED_ShowString(2, 11, "GX:"); OLED_ShowSignedNum(2, 14, gx, 6);
                // OLED_ShowString(3, 11, "GY:"); OLED_ShowSignedNum(3, 14, gy, 6);
                // OLED_ShowString(4, 11, "GZ:"); OLED_ShowSignedNum(4, 14, gz, 6);
                
                sprintf(buffer, "MPU6050 AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d\r\n", ax, ay, az, gx, gy, gz);
                HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);

                current_sensor = SENSOR_HMC;
            } break;
            case SENSOR_HMC: {
                int16_t mx, my, mz;
                HMC5883L_GetData(&mx, &my, &mz);
                
                // OLED_ShowString(1, 1, "HMC5883L");
                // OLED_ShowString(2, 1, "MX:"); OLED_ShowSignedNum(2, 4, mx, 6);
                // OLED_ShowString(3, 1, "MY:"); OLED_ShowSignedNum(3, 4, my, 6);
                // OLED_ShowString(4, 1, "MZ:"); OLED_ShowSignedNum(4, 4, mz, 6);

                sprintf(buffer, "HMC5883L MX:%d MY:%d MZ:%d\r\n", mx, my, mz);
                HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);

                current_sensor = SENSOR_BARO;
            } break;
            case SENSOR_BARO: {
                MS561101BA_Data_t ms_data;
                // OLED_ShowString(1, 1, "MS5611");
                if (MS561101BA_GetData(&ms_data, MS561101BA_OSR_4096)) {
                    int32_t tC = (int32_t)(ms_data.temperature + 0.5f);
                    int32_t p_hPa = (int32_t)(ms_data.pressure * 0.01f + 0.5f);
                    int32_t alt_m = (int32_t)(ms_data.altitude + 0.5f);
                    
                    // OLED_ShowString(2, 1, "T:"); OLED_ShowSignedNum(2, 3, tC, 4); OLED_ShowString(2, 8, "C");
                    // OLED_ShowString(3, 1, "P:"); OLED_ShowSignedNum(3, 3, p_hPa, 4); OLED_ShowString(3, 8, "hPa");
                    // OLED_ShowString(4, 1, "A:"); OLED_ShowSignedNum(4, 3, alt_m, 4); OLED_ShowString(4, 8, "m");

                    sprintf(buffer, "MS5611 T:%dC P:%dhPa A:%dm\r\n", tC, p_hPa, alt_m);
                    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
                } else {
                    // OLED_ShowString(2, 1, "MS5611 Error");
                    sprintf(buffer, "MS5611 Error\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
                }
                current_sensor = SENSOR_MPU;
            } break;
        }
    }
}

