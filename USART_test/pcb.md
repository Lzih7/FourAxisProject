## GY86
- SCL_Pin PB8
- SDA_Pin PB9
- VCC 3.3V
- GND
- 两个额外的孔(多出来两个没用的引脚)
## OLED
- OLED_SCL 随便
- OLED_SDA 随便
- VCC 3.3V
- GND
- 个人认为OLED不需要整个面都在转接板上，下部分可以略微超出，节省空间
## HC-05
- 改用UART1了，防止引脚冲突
### USART1
- RX_Pin PA10
- TX_Pin PA9
- VCC 3.3V
- GND
## Motor
### 使用TIM3的4通道进行输出比较
- Motor1_Pin PA6
- Motor2_Pin PA7
- Motor3_Pin PB0
- Motor4_Pin PB1
## 接收机
### 使用TIM5的4通道进行输入捕获
- VCC需5V
- GND
- TIM5_CH1_Pin PA0
- TIM5_CH2_Pin PA1
- TIM5_CH3_Pin PA2
- TIM5_CH4_Pin PA3
