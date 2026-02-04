## HAL_TIM_IC_Start_IT
- 功能：使能 TIM 输入捕获通道中断
- 参数：
  - `htim`：TIM 句柄指针
  - `Channel`：TIM 输入捕获通道
- 返回值：HAL_StatusTypeDef 类型，成功返回 HAL_OK，失败返回其他错误码
```c
HAL_StatusTypeDef HAL_TIM_IC_Start_IT (TIM_HandleTypeDef *htim, uint32_t Channel)
{
  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));
  
  switch (Channel)
  {
    case TIM_CHANNEL_1:
    {       
      /* Enable the TIM Capture/Compare 1 interrupt */
      __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
    }
    break;
    
    case TIM_CHANNEL_2:
    {
      /* Enable the TIM Capture/Compare 2 interrupt */
      __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC2);
    }
    break;
    
    case TIM_CHANNEL_3:
    {
      /* Enable the TIM Capture/Compare 3 interrupt */
      __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC3);
    }
    break;
    
    case TIM_CHANNEL_4:
    {
      /* Enable the TIM Capture/Compare 4 interrupt */
      __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC4);
    }
    break;
    
    default:
    break;
  }  
  /* Enable the Input Capture channel */
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);
    
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(htim);  

  /* Return function status */
  return HAL_OK;  
} 
```
- `TIM_IT_CC1`：捕获/比较通道 1 中断
- `TIM_IT_CC2`：捕获/比较通道 2 中断
- `TIM_IT_CC3`：捕获/比较通道 3 中断
- `TIM_IT_CC4`：捕获/比较通道 4 中断
- `__HAL_TIM_ENABLE_IT`：使能 TIM 中断
- `__HAL_TIM_ENABLE`：使能 TIM 外设
    - 所有的HAL库外设函数的start相关函数里都会使能相关外设，所以都==不需额外使能==
- `TIM_CCxChannelCmd`：使能 TIM 输入捕获通道
    - `HAL_TIM_IC_ConfigChannel`仅仅配置了通道的触发源，而使能通道中断需要在`HAL_TIM_IC_Start_IT`中完成
    - `TIM_CCx_ENABLE`：使能 TIM 输入捕获通道
```c
void TIM_CCxChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelState)
{
  uint32_t tmp = 0U;

  /* Check the parameters */
  assert_param(IS_TIM_CC1_INSTANCE(TIMx)); 
  assert_param(IS_TIM_CHANNELS(Channel));

  tmp = TIM_CCER_CC1E << Channel;

  /* Reset the CCxE Bit */
  TIMx->CCER &= ~tmp;

  /* Set or reset the CCxE Bit */ 
  TIMx->CCER |= (uint32_t)(ChannelState << Channel);
}
```
## HAL_TIM_IC_ConfigChannel
- 功能：配置 TIM 输入捕获通道
- 参数：
  - `htim`：TIM 句柄指针
  - `Channel`：TIM 输入捕获通道
  - `TIM_IC_InitTypeDef`：TIM 输入捕获通道配置结构体指针
- 返回值：HAL_StatusTypeDef 类型，成功返回 HAL_OK，失败返回其他错误码
```c
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef* sConfig, uint32_t Channel)
{
  /* Check the parameters */
  assert_param(IS_TIM_CC1_INSTANCE(htim->Instance));
  assert_param(IS_TIM_IC_POLARITY(sConfig->ICPolarity));
  assert_param(IS_TIM_IC_SELECTION(sConfig->ICSelection));
  assert_param(IS_TIM_IC_PRESCALER(sConfig->ICPrescaler));
  assert_param(IS_TIM_IC_FILTER(sConfig->ICFilter));
  
  __HAL_LOCK(htim);
  
  htim->State = HAL_TIM_STATE_BUSY;
  
  if (Channel == TIM_CHANNEL_1)
  {
    /* TI1 Configuration */
    TIM_TI1_SetConfig(htim->Instance,
               sConfig->ICPolarity,
               sConfig->ICSelection,
               sConfig->ICFilter);
               
    /* Reset the IC1PSC Bits */
    htim->Instance->CCMR1 &= ~TIM_CCMR1_IC1PSC;

    /* Set the IC1PSC value */
    htim->Instance->CCMR1 |= sConfig->ICPrescaler;
  }
  else if (Channel == TIM_CHANNEL_2)
  {
    /* TI2 Configuration */
    assert_param(IS_TIM_CC2_INSTANCE(htim->Instance));
    
    TIM_TI2_SetConfig(htim->Instance, 
                      sConfig->ICPolarity,
                      sConfig->ICSelection,
                      sConfig->ICFilter);
               
    /* Reset the IC2PSC Bits */
    htim->Instance->CCMR1 &= ~TIM_CCMR1_IC2PSC;

    /* Set the IC2PSC value */
    htim->Instance->CCMR1 |= (sConfig->ICPrescaler << 8U);
  }
  else if (Channel == TIM_CHANNEL_3)
  {
    /* TI3 Configuration */
    assert_param(IS_TIM_CC3_INSTANCE(htim->Instance));
    
    TIM_TI3_SetConfig(htim->Instance,  
               sConfig->ICPolarity,
               sConfig->ICSelection,
               sConfig->ICFilter);
               
    /* Reset the IC3PSC Bits */
    htim->Instance->CCMR2 &= ~TIM_CCMR2_IC3PSC;

    /* Set the IC3PSC value */
    htim->Instance->CCMR2 |= sConfig->ICPrescaler;
  }
  else
  {
    /* TI4 Configuration */
    assert_param(IS_TIM_CC4_INSTANCE(htim->Instance));
    
    TIM_TI4_SetConfig(htim->Instance, 
               sConfig->ICPolarity,
               sConfig->ICSelection,
               sConfig->ICFilter);
               
    /* Reset the IC4PSC Bits */
    htim->Instance->CCMR2 &= ~TIM_CCMR2_IC4PSC;

    /* Set the IC4PSC value */
    htim->Instance->CCMR2 |= (sConfig->ICPrescaler << 8U);
  }
  
  htim->State = HAL_TIM_STATE_READY;
    
  __HAL_UNLOCK(htim);
  
  return HAL_OK; 
}
```
- `TIM_TI1_SetConfig`：配置 TIM 输入捕获通道 1
- `TIM_TI2_SetConfig`：配置 TIM 输入捕获通道 2
- `TIM_TI3_SetConfig`：配置 TIM 输入捕获通道 3
- `TIM_TI4_SetConfig`：配置 TIM 输入捕获通道 4
- 本质是==把sConfig结构体里的参数写入到TIM的寄存器里==，再配置相关通道