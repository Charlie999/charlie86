bits 16
cpu 8086
org 0x100
__start:
    jmp 0x0000:_start ; set cs etc

_start:
    sti
    cld

    mov ax, [0x7DFE]
    cmp ax, 0x55AA
    jmp _run_bootsect
    mov si,nomagicmsg
    mov ah,0x0e
.loop:
    lodsb
    or al,al
    jz .err_done
    int 0x10
    jmp .loop
.err_done:
    mov si,nomagicmsg
    int 0xFE
    cli
    hlt
_run_bootsect:
    mov ax, 0x0000 ; stack starts at 0000:3000
    mov ss, ax
    mov sp, 0x3000

    mov si, bootmsg
    int 0xFE

    mov si, bootmsg
    mov ah, 0x0e
    .loop:
        lodsb
        or al,al
        jz .okprint_done
        int 0x10
        jmp .loop
    .okprint_done:

    mov ax, 1200
    int 0xFD ; delay by AX milliseconds, have no timer so this works OK

    mov ax, 0
    mov bx, 0
    mov cx, 0
    mov dx, 0
    mov si, 0
    mov di, 0
    mov bp, 0
    nop

    ;int 0xFC ; clear display

    jmp 0x0000:0x7C00
nomagicmsg: db "** BOOT ROM: Boot sector does not contain magic! **",10,0
bootmsg: db "** BOOT ROM: Jumping to 0x7C00 [boot delay 1200ms] **", 0

times 512 - ($-$$) db 0
