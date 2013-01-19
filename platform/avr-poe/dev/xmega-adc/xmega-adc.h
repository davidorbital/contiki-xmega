#ifndef __XMEGA_ADC_H__
#define __XMEGA_ADC_H__

void adc_init(void);
uint16_t adc_sample_channel(uint8_t chan);
int16_t adc_sample_differential(uint8_t mode); 
uint16_t adc_sample_temperature(void);
uint16_t adc_sample_vcc(void);

#endif
