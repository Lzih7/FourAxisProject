#include "main.h"

int main(void)
{
    HAL_Init();

    while(1)
    {
        // TODO: Add your code here
    }
}

void SystemClock_Config(void)
{
    // TODO: Configure system clock
}

void Error_Handler(void)
{
    __disable_irq();
    while(1) {}
}
