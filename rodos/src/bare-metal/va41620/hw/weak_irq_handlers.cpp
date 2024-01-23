#include "irq_handlers.h"

extern "C" void Default_Handler(){
    while(true);
}

[[gnu::weak, gnu::alias("Default_Handler")]] void NMI_Handler();
[[gnu::weak, gnu::alias("Default_Handler")]] void HardFault_Handler();
[[gnu::weak, gnu::alias("Default_Handler")]] void MemManage_Handler();
[[gnu::weak, gnu::alias("Default_Handler")]] void BusFault_Handler();
[[gnu::weak, gnu::alias("Default_Handler")]] void UsageFault_Handler();
[[gnu::weak, gnu::alias("Default_Handler")]] void SVC_Handler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DebugMon_Handler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PendSV_Handler();
[[gnu::weak, gnu::alias("Default_Handler")]] void SysTick_Handler();

[[gnu::weak, gnu::alias("Default_Handler")]] void SPI0_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void SPI0_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void SPI1_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void SPI1_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void SPI2_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void SPI2_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void SPI3_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void SPI3_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void UART0_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void UART0_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void UART1_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void UART1_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void UART2_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void UART2_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C0_MS_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C0_SL_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C1_MS_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C1_SL_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C2_MS_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C2_SL_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void Ether_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void SpW_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DAC0_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DAC1_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TRNG_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DMA_Error_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void ADC_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void LoCLK_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void LVD_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void WDT_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM0_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM1_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM2_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM3_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM4_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM5_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM6_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM7_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM8_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM9_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM10_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM11_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM12_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM13_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM14_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM15_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM16_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM17_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM18_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM19_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM20_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM21_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM22_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TIM23_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void CAN0_IRQHandler();

[[gnu::weak, gnu::alias("Default_Handler")]] void CAN1_IRQHandler();

[[gnu::weak, gnu::alias("Default_Handler")]] void EDAC_MBE_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void EDAC_SBE_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA0_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA1_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA2_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA3_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA4_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA5_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA6_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA7_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA8_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA9_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA10_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA11_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA12_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA13_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA14_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PA15_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB0_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB1_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB2_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB3_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB4_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB5_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB6_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB7_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB8_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB9_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB10_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB11_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB12_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB13_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB14_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PB15_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC0_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC1_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC2_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC3_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC4_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC5_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC6_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC7_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC8_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC9_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC10_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC11_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC12_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC13_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC14_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PC15_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD0_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD1_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD2_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD3_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD4_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD5_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD6_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD7_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD8_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD9_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD10_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD11_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD12_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD13_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD14_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PD15_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE0_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE1_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE2_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE3_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE4_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE5_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE6_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE7_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE8_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE9_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE10_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE11_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE12_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE13_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE14_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PE15_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF0_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF1_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF2_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF3_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF4_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF5_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF6_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF7_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF8_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF9_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF10_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF11_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF12_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF13_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF14_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void PF15_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DMA_Active_0_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DMA_Active_1_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DMA_Active_2_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DMA_Active_3_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DMA_Done_0_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DMA_Done_1_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DMA_Done_2_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void DMA_Done_3_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C0_MS_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C0_MS_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C0_SL_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C0_SL_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C1_MS_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C1_MS_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C1_SL_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C1_SL_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C2_MS_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C2_MS_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C2_SL_RX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void I2C2_SL_TX_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void FPU_IRQHandler();
[[gnu::weak, gnu::alias("Default_Handler")]] void TXEV_IRQHandler();