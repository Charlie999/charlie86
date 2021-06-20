all:
	nasm -f bin testloader.asm -o testloader.bin
	cp testloader.bin ../disk/BOOTSEC.BIN

	nasm -f bin rom.asm -o rom.bin
	cp rom.bin ../disk/BOOTROM.BIN
