export PATH=$PATH:/home/fademike/Programs/drivers/avrdude-7.0/
avrdude -c usbasp -p t13 -i 10000 -Uflash:w:brashless-esc.hex:i
