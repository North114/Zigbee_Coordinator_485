CC=avr-gcc
CP=avr-objcopy
CFLAGS=-Wall -Os
TFLAGS=-j .text -j .data -O ihex
DFLAG=-p m644p -c usbasp -e -U 
HEADERS=include/ds1307.c include/usart.c include/at24c128.c include/init.c
MACROS=#-DDEBUG#
## file name lists
## GPRS_Config_NakedSend.c
## Zigbee_Coordinator_July_24.c

SOURCE=ZigbeeCoordinator_485.c
#SOURCE=temp.c
OBJECT=$(SOURCE:.c=.o)
OUTPUT=output.hex

all:$(OUTPUT)

$(OUTPUT):$(OBJECT)
	$(CP) $(TFLAGS) $< $@

$(OBJECT):$(SOURCE)
	$(CC) $(MACROS) -mmcu=atmega644p $(CFLAGS) -o $@ $< $(HEADERS)

#$(SOURCE):$(HEADERS)

## dl for down load
dl:
	sudo avrdude $(DFLAG) flash:w:$(OUTPUT)

clean:
	rm $(OBJECT) $(OUTPUT)
