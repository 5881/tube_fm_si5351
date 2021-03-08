/*
 * тест библиотеки i2c oled дисплея
 * /

/**********************************************************************
 * Секция include и defines
**********************************************************************/
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/timer.h>
#include "si5351.h"
#include "oledi2c.h"
#include "oled_printf.h"
#define MIN_LIMIT 200
#define MAX_LIMIT 170000
#define IF_FEQ 10700  //ПЧ

uint32_t encoder=89100-IF_FEQ;
uint16_t pwm1=750;
uint16_t coef=50;
uint8_t encoder_mode=0;

static void adc_init(void){
	//На PA0 висит LV358 c делителем 1:2 в режиме повторителя
	
	uint8_t chanel=0;
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	adc_power_off(ADC1);
	adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
	adc_calibrate(ADC1);
	//adc_set_continuous_conversion_mode(ADC1);
	adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	//adc_enable_temperature_sensor();
	//adc_enable_vbat_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
	adc_set_regular_sequence(ADC1, 1, &chanel);
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++)__asm__("nop");
	
	//adc_start_conversion_regular(ADC1);//start conversion
}

uint16_t measure(){
	//exti_disable_request(EXTI8);
	uint32_t temp=0;
	for(uint16_t i=0;i<4096;i++){
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)));
		temp+=ADC_DR(ADC1);
		}
	temp=temp>>12;
	temp=temp*2*3290/4095; //пересчёт в миливольты беез претензий на точность
	//exti_enable_request(EXTI8);
	return (uint16_t)temp;
	
}



void led_setup(){
	//отладочный светодиод
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT,
					GPIO_PUPD_NONE, GPIO5);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP,
                    GPIO_OSPEED_2MHZ, GPIO5);
    }


void exti_encoder_init(){
		/*	энкодер и две кнопки с подтяжкой к питанию.
	 * PA8 энкодер 1
	 * PA7 энклдер 2
	 * 
	 */
	//Enable GPIOA clock.
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT,
					GPIO_PUPD_NONE, GPIO0|GPIO1|GPIO2);
	exti_select_source(EXTI0,GPIOA);
	exti_set_trigger(EXTI0,EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI0);
	nvic_enable_irq(NVIC_EXTI0_1_IRQ);
	
	}

static void gpio_setup(void){
	
	}
	
void exti0_1_isr(void){
	//indicate(2);
	//o_printf_at(0,6,1,0,"ISR_8_pin9=%x",((gpio_get(GPIOA,GPIO7))>>7));
	int8_t encoder_direction;
	if(!gpio_get(GPIOA,GPIO1))encoder_direction=1;else encoder_direction=-1;
	//o_printf_at(0,6,1,0,"ISR_8_pin9=%x",((gpio_get(GPIOA,GPIO7))>>7));
	if(encoder_mode==0){
		encoder+=coef*encoder_direction;
		if(encoder<MIN_LIMIT)encoder=MIN_LIMIT;
		if(encoder>MAX_LIMIT)encoder=MAX_LIMIT;
		}
	if(encoder_mode==1){
		pwm1+=10*encoder_direction;
		if(pwm1<100)pwm1=100;
		if(pwm1>3300)pwm1=3300;
		timer_set_oc_value(TIM3, TIM_OC4, pwm1);
		}
	//if(encoder_mode==1)select_band(encoder_direction);
	//if(encoder_mode==2)select_coef(encoder_direction);
	
	exti_reset_request(EXTI0);
	}	


static void i2c_setup(void){
	/* Enable clocks for I2C1 and AFIO. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_I2C1);
	/* Set alternate functions for the SCL and SDA pins of I2C1.
	 * SDA PA10
	 * SCL PA9
	 *  */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9|GPIO10);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD,
                    GPIO_OSPEED_2MHZ, GPIO9|GPIO10);
    gpio_set_af(GPIOA,GPIO_AF4,GPIO9|GPIO10);//ремапинг на i2c
	
	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(I2C1);
	i2c_set_speed(I2C1,i2c_speed_fm_400k,8);
	i2c_peripheral_enable(I2C1);
}	





void set_feq_90(uint64_t freq){
	//Используя si5153 можно непосредственно генерировать два сигнала
	//со сдвигом фаз 90 град.
	uint8_t coef=650000000/freq;
	uint64_t pll_freq=coef*freq;
	// We will output 14.1 MHz on CLK0 and CLK1.
	// A PLLA frequency of 705 MHz was chosen to give an even
	// divisor by 14.1 MHz.
	//unsigned long long freq = 14100000 00ULL;
	//unsigned long long pll_freq = 705000000 00ULL;
	// Set CLK0 and CLK1 to output 14.1 MHz with a fixed PLL frequency
	set_freq_manual(freq*100, pll_freq*100, SI5351_CLK0);
	set_freq_manual(freq*100, pll_freq*100, SI5351_CLK1);
	// Now we can set CLK1 to have a 90 deg phase shift by entering
	// 50 in the CLK1 phase register, since the ratio of the PLL to
	// the clock frequency is 50.
	set_phase(SI5351_CLK0, 0);
	set_phase(SI5351_CLK1, coef);
	// We need to reset the PLL before they will be in phase alignment
	pll_reset(SI5351_PLLA);
	}
	
void spi_init(void){
	//spi интерфейс для 74hc595
	//PA7 mosi
	//PA5 sck
	//PA6 indicate
	//Enable SPI1 Periph and gpio clocks
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_SPI1);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT,
					GPIO_PUPD_NONE, GPIO6);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP,
                    GPIO_OSPEED_50MHZ, GPIO6);
    //MOSI,SCK
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP,
                    GPIO_OSPEED_50MHZ, GPIO7|GPIO5);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7|GPIO5);
	gpio_set_af(GPIOA,GPIO_AF0,GPIO7|GPIO5);
	//Настройка SPI1
	spi_disable(SPI1);
	spi_reset(SPI1);
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256,
					SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
					SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_LSBFIRST);
	//spi_set_crcl_8bit(SPI1); Закоментировато тк один хрен не работает
	spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);
	spi_enable(SPI1);
}
	
static const uint8_t sym_table[10]={0,1,4,5,8,9,12,13,2,3};
void indicate(uint32_t fkhz){
	uint8_t a,b=0b10000,c;
	uint16_t ac;
	uint8_t temp;
	//тушим лишние разряды
	if(fkhz>999)b=0b11110000;
		else if(fkhz>99)b=0b1110000;
		else if(fkhz>9)b=0b110000;
	for(uint8_t i=0;i<4;i++){
		temp=fkhz%10;
		fkhz/=10;
		ac<<=4;
		ac|=sym_table[temp];
		//ac|=temp;
		}
	a=(uint8_t)(ac&0xff);
	c=(uint8_t)(ac>>8);
	spi_send8(SPI1, c);
	spi_send8(SPI1, b);
	spi_send8(SPI1, a);
	//while (!(SPI_SR(SPI1) & SPI_SR_BSY));
	gpio_set(GPIOA,GPIO6);
	//for(uint16_t i=0;i<0xff;i++) __asm__("nop");
	gpio_clear(GPIOA,GPIO6);
	}

void rcc_clock_setup_in_hsi_out_64mhz(void)
 {
         rcc_osc_on(RCC_HSI);
         rcc_wait_for_osc_ready(RCC_HSI);
         rcc_set_sysclk_source(RCC_HSI);
  
         rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
         rcc_set_ppre(RCC_CFGR_PPRE_NODIV);
  
         flash_prefetch_enable();
         flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);
  
         /* 8MHz * 12 / 2 = 48MHz */
         rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL16);
         rcc_set_pll_source(RCC_CFGR_PLLSRC_HSI_CLK_DIV2);
  
         rcc_osc_on(RCC_PLL);
         rcc_wait_for_osc_ready(RCC_PLL);
         rcc_set_sysclk_source(RCC_PLL);
  
         rcc_apb1_frequency = 64000000;
         rcc_ahb_frequency = 64000000;
 }

void rcc_clock_setup_in_hsi_out_8mhz(void)
 {
         rcc_osc_on(RCC_HSI);
         rcc_wait_for_osc_ready(RCC_HSI);
         rcc_set_sysclk_source(RCC_HSI);
  
         rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
         rcc_set_ppre(RCC_CFGR_PPRE_NODIV);
  
         flash_prefetch_enable();
         flash_set_ws(FLASH_ACR_LATENCY_000_024MHZ);
  
         /* 8MHz * 4 / 2 = 16MHz */
         rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL2);
         rcc_set_pll_source(RCC_CFGR_PLLSRC_HSI_CLK_DIV2);
  
         rcc_osc_on(RCC_PLL);
         rcc_wait_for_osc_ready(RCC_PLL);
         rcc_set_sysclk_source(RCC_PLL);
  
         rcc_apb1_frequency = 8000000;
         rcc_ahb_frequency = 8000000;
 }

void pwm_init(void){
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP,
                        GPIO_OSPEED_2MHZ, GPIO0|GPIO1);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0|GPIO1);
	gpio_set_af(GPIOB,GPIO_AF1,GPIO0|GPIO1);
	rcc_periph_clock_enable(RCC_TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT_MUL_4, TIM_CR1_CMS_CENTER_1,
               TIM_CR1_DIR_UP);
	timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);
	timer_enable_oc_output(TIM3, TIM_OC3);
	timer_enable_oc_output(TIM3, TIM_OC4);
	//timer_enable_break_main_output(TIM1);
	timer_set_oc_value(TIM3, TIM_OC3, 750);
	timer_set_oc_value(TIM3, TIM_OC4, 750);
	timer_set_period(TIM3, 3000);
	//timer_set_oc_value(TIM1, TIM_OC1, 1400);
	timer_enable_counter(TIM3);
}
	
void main(){
	//rcc_clock_setup_in_hsi_out_48mhz();
	rcc_clock_setup_in_hsi_out_8mhz();
	led_setup();
	//gpio_setup();
	exti_encoder_init();
	i2c_setup();
	spi_init();
	pwm_init();
	gpio_set(GPIOB,GPIO5);
	si5351_init(SI5351_CRYSTAL_LOAD_10PF, 25000000, 0);
	si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_6MA);
	si5351_set_freq(10200000ULL, SI5351_CLK0);
	//si5351_set_freq(IF_FEQ*100000ULL, SI5351_CLK1);
	//si5351_set_freq(1020000*100ULL, SI5351_CLK2);
	uint32_t old_enc=0;
	uint32_t count=0;
	//si5351_set_freq(encoder*100000ULL, SI5351_CLK0);
	uint16_t j=0;
	uint32_t old_encoder=0;
	int varicap_val=750;
	while(1){
		switch(encoder_mode){
			case 0:
				indicate((encoder+IF_FEQ)/100);
				break;
			case 1:
				indicate(pwm1);
				break;
		}
		
		if(!gpio_get(GPIOA,GPIO2)){
			while(!gpio_get(GPIOA,GPIO2))__asm__("nop");
			encoder_mode++;
			if(encoder_mode>1)encoder_mode=0;
			if(encoder_mode)pwm1=(uint16_t)varicap_val;
			}
		if(old_encoder!=encoder){
			si5351_set_freq(encoder*100000ULL, SI5351_CLK0);
			varicap_val=(int)((double)encoder*0.07088-4709);
			if(varicap_val<100)varicap_val=100;
			if(varicap_val>3300)varicap_val=3300;
			timer_set_oc_value(TIM3, TIM_OC4, (uint16_t)varicap_val);
			old_encoder=encoder;
		}
		for(uint32_t i=0;i<0x5fff;i++)__asm__("nop");
		gpio_toggle(GPIOB,GPIO5);
		
		}
	}

