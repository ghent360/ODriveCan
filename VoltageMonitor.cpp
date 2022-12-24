
/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#include <Arduino.h>
#include <ADC.h>
#include <ADC_util.h>
#include "globals.h"

static const int batteryVoltagePin = A17; // ADC0

static ADC adc; // adc object;

/*
 * The following is a copy of the hibernation code from the Snooze Teensy library
 * https://github.com/duff2013/Snooze
 * 
 * Somehow using the Snooze library crashes the firmware, so this is a copy
 * of the code that puts the Teensy 4.x CPU in low power mode.
*/
#if defined(__IMXRT1062__)

#define XTALOSC24M_LOWPWR_CTRL_SET_OSC_SEL ((uint32_t)(0x10U))
#define DCDC_REG2_LOOPCTRL_EN_RCSCALE1(n)        ((uint32_t)(((n) & 0x07) << 9))

// SCB D-Cache Clean by Set-way Register Definitions
#define SCB_DCCSW_SET_POS   5
#define SCB_DCCSW_SET_MASK  (0x1FF << SCB_DCCSW_SET_POS)

#define SCB_DCCSW_WAY_POS   30
#define SCB_DCCSW_WAY_Msk   (3 << SCB_DCCSW_WAY_POS)

// Cache Size ID Register Macros
#define CCSIDR_WAYS(x)  (((x) & SCB_CCSIDR_ASSOCIATIVITY_MASK) >> SCB_CCSIDR_ASSOCIATIVITY_POS)
#define CCSIDR_SETS(x)  (((x) & SCB_CCSIDR_NUMSETS_MASK) >> SCB_CCSIDR_NUMSETS_POS      )

#define SCB_CCSIDR_ASSOCIATIVITY_POS    3
#define SCB_CCSIDR_ASSOCIATIVITY_MASK   (0x3FF << SCB_CCSIDR_ASSOCIATIVITY_POS)

#define SCB_CCSIDR_NUMSETS_POS          13
#define SCB_CCSIDR_NUMSETS_MASK         (0x7FFF << SCB_CCSIDR_NUMSETS_POS)

#define SCB_DCCISW_SET_POS              5
#define SCB_DCCISW_SET_MASK             (0x1FF << SCB_DCCISW_SET_POS)

#define SCB_DCCISW_WAY_POS              30
#define SCB_DCCISW_WAY_MASK              (3 << SCB_DCCISW_WAY_POS)

#define SCB_CCR_DC_POS                  16
#define SCB_CCR_DC_MASK                 (1 << SCB_CCR_DC_POS)

static void SCB_CleanDCache (void) {
    uint32_t ccsidr;
    uint32_t sets;
    uint32_t ways;
    
    SCB_ID_CSSELR = 0U; // Level 1 data cache
    __asm volatile ( "dsb \n" );
    
    ccsidr = SCB_ID_CCSIDR;
    
    // clean D-Cache
    sets = (uint32_t)(CCSIDR_SETS(ccsidr));
    do {
        ways = (uint32_t)(CCSIDR_WAYS(ccsidr));
        do {
            SCB_CACHE_DCCSW = (((sets << SCB_DCCSW_SET_POS) & SCB_DCCSW_SET_MASK) |
                               ((ways << SCB_DCCSW_WAY_POS) & SCB_DCCSW_WAY_Msk)  );
        } while (ways-- != 0U);
    } while(sets-- != 0U);
    __asm volatile ( "dsb \n" );
    __asm volatile ( "isb \n" );
}

static void SCB_DisableDCache (void) {
    register uint32_t ccsidr;
    register uint32_t sets;
    register uint32_t ways;
    
    SCB_ID_CSSELR = 0U; // Level 1 data cache
    __asm volatile ( "dsb \n" );
    
    SCB_CCR &= ~(uint32_t)SCB_CCR_DC_MASK;  // disable D-Cache
    __asm volatile ( "dsb \n" );
    
    ccsidr = SCB_ID_CCSIDR;
    
    // clean & invalidate D-Cache
    sets = (uint32_t)(CCSIDR_SETS(ccsidr));
    do {
        ways = (uint32_t)(CCSIDR_WAYS(ccsidr));
        do {
            SCB_CACHE_DCCISW = (((sets << SCB_DCCISW_SET_POS) & SCB_DCCISW_SET_MASK) |
                                ((ways << SCB_DCCISW_WAY_POS) & SCB_DCCISW_WAY_MASK)  );
        } while (ways-- != 0U);
    } while(sets-- != 0U);
    __asm volatile ( "dsb \n" );
    __asm volatile ( "isb \n" );
}

static FASTRUN uint32_t set_clock_osc( uint32_t frequency ) {
    if ( 24000000 % frequency || frequency > 24000000 ) return 0;
    uint32_t cbcdr = CCM_CBCDR;
    uint32_t cbcmr = CCM_CBCMR;
    uint32_t dcdc = DCDC_REG3;
    
    uint32_t div_clk2 = 1;
    uint32_t div_ahb = 1;
    uint32_t freq_div = 24000000/frequency;
    for ( int i = 1; i < 9; i++ ) {
        div_ahb = freq_div / i;
        div_clk2 = i;
        if ( div_ahb <= 8 ) break;
    }
    
    cbcdr &= ~CCM_CBCDR_PERIPH_CLK2_PODF_MASK;
    cbcdr |= CCM_CBCDR_PERIPH_CLK2_PODF( div_clk2 - 1 );
    CCM_CBCDR = cbcdr;
    
    cbcmr &= ~CCM_CBCMR_PERIPH_CLK2_SEL_MASK;
    cbcmr |= CCM_CBCMR_PERIPH_CLK2_SEL( 1 );
    CCM_CBCMR = cbcmr;
    while ( CCM_CDHIPR & CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY );
    
    cbcdr &= ~CCM_CBCDR_AHB_PODF_MASK;
    cbcdr |= CCM_CBCDR_AHB_PODF( div_ahb - 1 );
    CCM_CBCDR = cbcdr;
    while ( CCM_CDHIPR & CCM_CDHIPR_AHB_PODF_BUSY );
    
    cbcdr |= CCM_CBCDR_PERIPH_CLK_SEL;
    CCM_CBCDR = cbcdr;
    while ( CCM_CDHIPR & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY );
    
    cbcdr &= ~CCM_CBCDR_IPG_PODF_MASK;
    cbcdr |= CCM_CBCDR_IPG_PODF( 0 );
    
    CCM_CBCDR = cbcdr;
    
    CCM_ANALOG_PLL_ARM_SET = CCM_ANALOG_PLL_ARM_BYPASS;
    CCM_ANALOG_PLL_ARM_SET = CCM_ANALOG_PLL_ARM_POWERDOWN;
    
    F_CPU_ACTUAL = frequency;
    F_BUS_ACTUAL = frequency;
    scale_cpu_cycles_to_microseconds = 0xFFFFFFFFu / ( uint32_t )( frequency / 1000000u );
    
    dcdc &= ~DCDC_REG3_TRG_MASK;
    dcdc |= DCDC_REG3_TRG( 6 );
    DCDC_REG3 = dcdc;
    while ( !( DCDC_REG0 & DCDC_REG0_STS_DC_OK ) );
    
    return frequency;
}
//----------------------------------------------------------------------------------
static FASTRUN void set_clock_rc_osc( void ) {
    // Switch to RC-OSC
    XTALOSC24M_LOWPWR_CTRL_SET = XTALOSC24M_LOWPWR_CTRL_SET_OSC_SEL;
    // Turn off XTAL-OSC
    CCM_ANALOG_MISC0_SET = CCM_ANALOG_MISC0_XTAL_24M_PWD;
    while ( CCM_CDHIPR != 0 );
    for ( uint32_t i = 0; i < 5 * 50; i++ ) asm volatile( "nop \n" );
}

//----------------------------------------------------------------------------------
static void hal_hibernate ( void ) {
    SCB_SCR = SCB_SCR_SLEEPDEEP;
    SYST_CSR &= ~SYST_CSR_TICKINT;
    /**************************************************************/
    // Wait mode sequence errata
    IOMUXC_GPR_GPR1 |= IOMUXC_GPR_GPR1_GINT;
    //NVIC_SET_PENDING(IRQ_GPR_IRQ);
    //NVIC_ENABLE_IRQ(IRQ_GPR_IRQ);
    GPC_IMR2 &= ~0x200;
    uint32_t clpcr;
    clpcr = CCM_CLPCR | ( CCM_CLPCR_LPM( 0x02 ) | CCM_CLPCR_VSTBY | 0x600 | CCM_CLPCR_SBYOS | CCM_CLPCR_ARM_CLK_DIS_ON_LPM | 0x08280000 );
    CCM_CLPCR = clpcr;
    GPC_IMR2 |= 0x200;
    //NVIC_DISABLE_IRQ(IRQ_GPR_IRQ);
    //NVIC_CLEAR_PENDING(IRQ_GPR_IRQ);
    /**************************************************************/
    CCM_CCGR0 = 0x45;
    CCM_CCGR1 = 0 | ( CCM_CCGR1 & ( CCM_CCGR1_GPT1_SERIAL( 2 ) | CCM_CCGR1_GPT1_BUS( 2 ) ) );
    CCM_CCGR2 = 0x1 | ( CCM_CCGR2 & CCM_CCGR2_IOMUXC_SNVS( 2 ) );
    CCM_CCGR3 = 0x10000000 | ( CCM_CCGR3 & ( CCM_CCGR3_ACMP1( 2 ) | CCM_CCGR3_ACMP2( 2 ) | CCM_CCGR3_ACMP3( 2 ) | CCM_CCGR3_ACMP4( 2 ) ) );
    CCM_CCGR4 = 0x110;
    CCM_CCGR5 = 0x1105;
    // We can enable DCDC when need to config it and close it after configuration
    CCM_CCGR6 = 0x400000;
    
    // Turn off FlexRAM1
    PGC_MEGA_CTRL |= PGC_MEGA_CTRL_PCR;
    
    // Turn off FlexRAM0
    GPC_CNTR |= GPC_CNTR_PDRAM0_PGE;
    
#define __VECTOR_TABLE _VectorsRam
    //IOMUXC_GPR_GPR16 = 0x200007;//( uint32_t )( (( uint32_t )__VECTOR_TABLE) | 7 );
    
    SCB_CleanDCache();
    SCB_DisableDCache();
    
    set_clock_osc( 24000000 );
    CCM_ANALOG_PLL_ARM |= CCM_ANALOG_PLL_ARM_BYPASS;
    CCM_ANALOG_PLL_ARM_SET = CCM_ANALOG_PLL_ARM_POWERDOWN;
    
    __disable_irq();
    set_clock_rc_osc( );
    __enable_irq();
    
    //Disconnected the internal load resistor
    DCDC_REG1 &= ~DCDC_REG1_REG_RLOAD_SW;
    
    //Power Down output range comparator
    DCDC_REG0 |= DCDC_REG0_PWD_CMP_OFFSET;
    
    DCDC_REG0 &= ~( DCDC_REG0_PWD_ZCD | DCDC_REG0_PWD_CMP_OFFSET );
    DCDC_REG2 = ( ~0xE00 & DCDC_REG2 ) | DCDC_REG2_LOOPCTRL_EN_RCSCALE1( 0x4 ) | DCDC_REG2_DCM_SET_CTRL;
    // Switch DCDC to use DCDC internal OSC
    // Configure the DCDC_REG0 register.
    uint32_t reg0 = DCDC_REG0 & ~( DCDC_REG0_XTAL_24M_OK | DCDC_REG0_DISABLE_AUTO_CLK_SWITCH | DCDC_REG0_SEL_CLK |DCDC_REG0_PWD_OSC_INT );
    reg0 |= DCDC_REG0_DISABLE_AUTO_CLK_SWITCH;
    DCDC_REG0 = reg0;
    
    // Power down CPU when requested
    PGC_CPU_CTRL = PGC_CPU_CTRL_PCR;
    //PGC_MEGA_CTRL = PGC_CPU_CTRL_PCR;
    //PGC_CPU_PDNSCR = 0xA0A;
    
    PMU_REG_CORE_SET = PMU_REG_CORE_FET_ODRIVE;
    // Disconnect vdd_high_in and connect vdd_snvs_in
    CCM_ANALOG_MISC0_SET = CCM_ANALOG_MISC0_DISCON_HIGH_SNVS;
    // STOP_MODE config, turn off all analog except RTC in stop mode
    CCM_ANALOG_MISC0_CLR = 0xC00;//CCM_ANALOG_MISC0_STOP_MODE_CONFIG( 0x3 );
    //CCM_ANALOG_MISC0_SET = 0x800;
    
    uint32_t gpc_imr1 = GPC_IMR1;
    uint32_t gpc_imr2 = GPC_IMR2;
    uint32_t gpc_imr3 = GPC_IMR3;
    uint32_t gpc_imr4 = GPC_IMR4;
    uint32_t gpc_imr5 = GPC_IMR5;
    
    GPC_IMR1 = 0xFFFFFFFF;
    GPC_IMR2 = 0xFFFFFFFF;
    GPC_IMR3 = 0xFFFFFFFF;
    GPC_IMR4 = 0xFFFFFFFF;
    GPC_IMR5 = 0xFFFFFFFF;
    
    CCM_CCR = ( CCM_CCR & ~0x7E00000 ) | CCM_CCR_REG_BYPASS_COUNT( 0x3 ) | CCM_CCR_OSCNT( 0xAF );
    CCM_CCR |= ( CCM_CCR_RBC_EN | CCM_CCR_COSC_EN  );
    
    for ( int i = 0; i < 12 * 22; i++ ) __asm volatile ( "nop \n" );
    
    GPC_IMR1 = gpc_imr1;
    GPC_IMR2 = gpc_imr2 | ( 0x20000200 );
    GPC_IMR3 = gpc_imr3;
    GPC_IMR4 = gpc_imr4;
    GPC_IMR5 = gpc_imr5;
    
    IOMUXC_GPR_GPR4 = 0x00000010;
    while( ( IOMUXC_GPR_GPR4 & 0x00100000 ) != 0x00100000 );
    IOMUXC_GPR_GPR4 = 0x000036f0;
    IOMUXC_GPR_GPR7 = 0x0000ffff;
    IOMUXC_GPR_GPR8 = 0xfffcffff;
    IOMUXC_GPR_GPR12 = 0x0000000a;
    while( ( IOMUXC_GPR_GPR4 & 0x36f00000 ) != 0x36f00000 );
    while( ( IOMUXC_GPR_GPR7 & 0xffff0000 ) != 0xffff0000 );

    /*IOMUXC_GPR_GPR4 = 0x00000011;
     while ((IOMUXC_GPR_GPR4 & 0x00110000) != 0x00110000){};
     digitalWriteFast( LED_BUILTIN, HIGH );
     IOMUXC_GPR_GPR4 = 0x000036ff;
     IOMUXC_GPR_GPR7 = 0x0000ffff;
     IOMUXC_GPR_GPR8 = 0xfffcffff;
     IOMUXC_GPR_GPR12 = 0x0000000a;
     while ((IOMUXC_GPR_GPR4 & 0x36f90000) != 0x36f90000){};
     while ((IOMUXC_GPR_GPR7 & 0xffff0000) != 0xffff0000){};*/
    //save_context( );
    //while(1);
    
    __asm volatile ( "dsb \n" );
    __asm volatile ( "isb \n" );
    __asm volatile ( "wfi \n" );
    
    // future landing spot for context restore from hibernate reset
    while( 1 );
}
#endif // defined(__IMXRT1062__)

void initVoltageMonitor() {
  pinMode(batteryVoltagePin, INPUT);

  adc.adc0->setAveraging(16); // set number of averages
  adc.adc0->setResolution(16); // set bits of resolution

  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED); // change the sampling speed
}

float readBatteryVoltage() {
  // Single reads
  int value = adc.adc0->analogRead(batteryVoltagePin);
  float vIn = value*3.3*(82+22)/(22*adc.adc0->getMaxValue());

  // Print errors, if any.
  if(adc.adc0->fail_flag != ADC_ERROR::CLEAR) {
    Serial.print("ADC0: ");
    Serial.println(getStringADCError(adc.adc0->fail_flag));
    return -1;
  }
  return vIn;
}

void lowPowerMode() {
  canInterface.canSleep();  // Disable the CAN transcievers.
  hal_hibernate();
}