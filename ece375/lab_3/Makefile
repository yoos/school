######################################################################
### ECE 375
###
### Oregon State - EECS
### author: Gavin Lloyd
### date: Oct. 5th, 2011
### modified by: Thomas Ruggeri (Nov. 1st 2011)
###
### requires: [gavrasm or AVRA] and [avrdude with osuisp2].
###             gavrasm: http://www.avr-asm-tutorial.net/gavrasm/index_en.html
###				AVRA:	 http://avra.sourceforge.net/
###             osuisp2: http://beaversource.oregonstate.edu/projects/osuisp2/
######################################################################

# Name of .asm file here
PROG = mult
TEX = writeup

# Assemble code
all: $(PROG).asm
	#gavrasm $(PROG).asm
	avra --includepath /usr/share/avra/includes/ $(PROG).asm

# Flash code
#
# avrdude with OSU-ISP.2 for Arch Linux:
# https://aur.archlinux.org/packages.php?ID=52763
#
# you may need to change the connection port (-P)
#
flash: $(PROG).hex
	sudo /home/yoos/devel/osuisp2/avrdude/avrdude -C /home/yoos/devel/osuisp2/avrdude/avrdude.conf -c osuisp2 -P usb -p m128 -e -U flash:w:$(PROG).hex -v

tex:
	pdflatex ${TEX}.tex
	pdflatex ${TEX}.tex

# Cleanup
clean:
	@rm -rf *.lst *.hex
