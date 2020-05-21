GPIOs:
SDn => PA10 en el x-Nucleo --NC en PCB
SPI MISO => PA6 en x-Nucleo -- PA6 en PCB
SPI MOSI => PA7 en x-Nucleo -- PC4 en PCB
SPI SCK => PB3 en x-Nucleo -- PA5 en PCB
SPI CS => PB6 en x-Nucleo -- PB0 S_Cn en PCB
			     PB1 H_Cn en PCB

GPIO EXIT conectado al GPIO3 del Spirit que alerta de 
tramas enviadas/recibidas:
PC7 en x-Nucleo -- PB10 si H_GPIO3
		   PA4 si S_GPIO3


SPI confs:
BaudratePrescaler = 4.
Direction 2 lines.
Phase 1 edge
Polarity low
CRC CALCULATION DISABLE
CRC polynomial = 7
DataSize = 8 Bits
MSB First
NSS Soft
TIMODE disabled
Master mode.


Parámetros iniciales de RF:
XTAL OFFSET PPM 0 => Nuestro reloj: +-10PPM
Base Frequency: 868 => 868 y 433 en la PCB

Siguiendo las ecuaciones:
fc = fbase+foffset+(fXO/2^15)*CHannelSpace*ChannelNumber	
fbase = (fXO)/((B*D)/2)*SYNT/2^18
foffset= (fXO/2^18) * FC_OFFSET

fbase=868 (o 433)

CHSPACE = 100e3
CHNUMBER = 0

Donde FXO: Frecuencia del XTAL (50MHz).
SYNT: 26bits programables.
F_OFFSET: 12 bits programables
B: 6 para bandas altas (779MHz a 956MHz, BS=1)
   12 para bandas medias (387MHz a 470MHz,  BS=3)
   16 para bandas bajas (300MHz a 348MHz, BS=4)
   32 para bandas muy bajas(169MHz, BS=5)

D: 1 Si REFDIV 0 (Internal reference divider is disabled)
   2 Si REFDIV 1 (Internal reference divider is enabled) 


En las librerías del spirit:
s_assert_param(IS_FREQUENCY_BAND((pxSRadioInitStruct->lFrequencyBase + ((xtalOffsetFactor*s_lXtalFrequency)/FBASE_DIVIDER) + pxSRadioInitStruct->nChannelSpace * pxSRadioInitStruct->cChannelNumber)));  
868 + (50000000/2^18) + 100e3*0 =  868 (o 433) + 1058.73

De manera predefinida por las librerías tenemos:
Modulación: FSK
Data Rate: 38400
FREQ Derivation 20e3 => Variación en la portadora en función de las variaciones d ela moduladora.
BandWidth 100e3

POWER_INDEX = 7 => Este índice se utiliza para establecer el PA_Level_max a 7 (nivel máximo). Esto implica configurar la rampa de potencia de salida a su máximo nivel
		   (p. 53 del datasheet).
POWER_DBM = 11.6 => Potencia de salida establecida. Las librerías del SPIRIT calculan automáticamente el valor de los registros PA_Level en función de este valor.

SQI: Synchronization Quality Indicator => 4 niveles de error (en 0,1,2 o 3 bits). De manera predefinida puesto en 0. 
RSSI_THRESHOLD: Sigue la fórmula (RSSI[Dbm] + 130)/0.5 => Predefinido a -120dB (mayor al piso de ruido) => (-120+130)*0.5 = 20dBm. Entiendo que rechaza las señales
		que no superen este valor.