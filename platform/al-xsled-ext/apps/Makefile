#
# Build with the following commands:
#
# make clean
# make al-xs-app.hex
# sudo /usr/local/avr/bin/avrdude -p x256a3 -c avrispmkii -P usb -U flash:w:al-xs-app.hex


CONTIKI_PROJECT = al-xs-app 

all: $(CONTIKI_PROJECT)

CONTIKI = ../../..
APPS = antelope
include $(CONTIKI)/Makefile.include
