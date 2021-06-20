bits 16
org 0x7c00
boot:
    mov si,msg
    mov ah, 0xe
    jmp donetext
.loop:
    lodsb
    or al,al
    jz donetext
    int 0x10
    jmp .loop
donetext:
    mov ah, 0
    mov al, 0x13
    int 0x10

    jmp drawtext ; jump beyond the bootsector

CHARLIE_START equ 30

msg: db "If you're seeing this, the emulator worked!",0

times 510 - ($-$$) db 0
dw 0xaa55

drawtext:
    mov cl, 70
    mov ch, 0x17
    call drawhline

    mov cl, 0
    mov ch, 0x30
    call drawhline
    mov cl, 199
    call drawhline
    mov cl, 0
    call drawvline
    mov bp, 319
    call drawvline

    mov cx, CHARLIE_START
    mov dx, 20
    mov al, 0x0B
    mov di, 4

;; BEGIN DRAWING "C" CHARACTER
    xor si,si
    .textLoop1:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 5
        jne .textLoop1

    sub dx, 16
    add cx, 4
    call drawvszpixel
    add cx, 4
    call drawvszpixel
    add cx, 4
    call drawvszpixel

    add dx, 16
    sub cx, 8
    call drawvszpixel
    add cx, 4
    call drawvszpixel
    add cx, 4
    call drawvszpixel

;; START DRAWING "H" CHARACTER
    add cx, 12
    mov dx, 20

    xor si,si
    .textLoop2:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 5
        jne .textLoop2

    sub dx, 8
    add cx, 4
    call drawvszpixel
    add cx, 4
    call drawvszpixel
    add cx, 4

    mov dx, 20
    xor si,si
    .textLoop3:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 5
        jne .textLoop3

;; START DRAWING "A" CHARACTER

    mov dx, 28
    add cx, 12
    xor si, si
    .textLoop4:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 3
        jne .textLoop4
    mov dx, 24
    add cx, 4
    xor si, si
    .textLoop5:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 2
        jne .textLoop5

    mov dx, 24
    call drawvszpixel
    add cx, 4
    call drawvszpixel
    add cx, 4
    call drawvszpixel

    xor si, si
    .textLoop6:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 2
        jne .textLoop6
    sub cx, 4
    call drawvszpixel

    sub dx, 4
    add cx, 8
    xor si, si
    .textLoop7:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 3
        jne .textLoop7

;; START DRAWING "R" CHARACTER

    mov dx, 20
    add cx, 12
    xor si,si
    .textLoop8:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 5
        jne .textLoop8

    mov dx, 24
    add cx, 4
    call drawvszpixel
    add dx, 8
    call drawvszpixel
    sub dx, 8
    add cx, 4
    call drawvszpixel
    add dx, 4
    call drawvszpixel
    add dx, 4
    call drawvszpixel
    add cx, 4
    xor si,si
    .textLoop9:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 2
        jne .textLoop9

;; START DRAWING "L" CHARACTER
    mov dx, 20
    add cx, 12
    xor si,si
    .textLoop10:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 5
        jne .textLoop10

    add cx, 4
    call drawvszpixel
    add cx, 4
    call drawvszpixel
    add cx, 4
    call drawvszpixel

;; START DRAWING "I" CHARACTER
    add cx, 8
    call drawvszpixel
    add cx, 4
    call drawvszpixel

    mov dx, 20
    xor si,si
    .textLoop11:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 5
        jne .textLoop11

    add cx, 4
    call drawvszpixel

    sub cx, 8
    mov dx, 24
    call drawvszpixel
    add cx, 8
    call drawvszpixel

;; BEGIN DRAWING "E" CHARACTER
    add cx, 12
    sub dx, 4
    xor si,si
    .textLoop12:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 5
        jne .textLoop12

    sub dx, 16
    add cx, 4
    call drawvszpixel
    add cx, 4
    call drawvszpixel
    add cx, 4
    call drawvszpixel

    add dx, 8
    sub cx, 8
    call drawvszpixel
    add cx, 4
    call drawvszpixel

    add dx, 8
    sub cx, 4
    call drawvszpixel
    add cx, 4
    call drawvszpixel
    add cx, 4
    call drawvszpixel

;; BEGIN DRAWING '8' CHARACTER
    mov al, 0x7F

    add cx, 64
    mov dx, 20
    xor si,si
    .textLoop14:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 5
        jne .textLoop14

    jmp .afterjcall_8
    .jcall_8:
        sub dx, 8
        add cx, 4
        call drawvszpixel
        add cx, 4
        call drawvszpixel
        sub cx, 8
        ret

    .afterjcall_8:
    add dx, 8
    call .jcall_8
    call .jcall_8
    call .jcall_8

    add cx, 12
    mov dx, 20
    xor si,si
    .textLoop15:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 5
        jne .textLoop15
        xor si,si

;; BEGIN DRAWING '6' CHARACTER
    add cx, 12
    mov dx, 20
    xor si,si
    .textLoop17:
        add dx, 4
        call drawvszpixel
        inc si
        cmp si, 5
        jne .textLoop17

    add dx, 8
    call .jcall_8
    call .jcall_8
    call .jcall_8
    add cx, 12
    add dx, 8
    call drawvszpixel
    add dx, 4
    call drawvszpixel
    add dx, 4
    call drawvszpixel

drawface:
    mov cx, 100  ; X=100
    mov dx, 90   ; Y=90
    call drawEye

    mov cx, 180
    mov dx, 90
    call drawEye

    mov cx, 90
    mov dx, 130
    mov al, 0x0F
    mov di, 4
    xor si, si
    .mouthLoop1:
        push si
        call drawvszpixel
        pop si
        inc si
        add cx, 3
        add dx, 3
        cmp si, 12
        jne .mouthLoop1

    xor si,si
    .mouthLoop2Flat:
        push si
        call drawvszpixel
        pop si
        add cx, 4
        inc si
        cmp si, 10
        jne .mouthLoop2Flat

    xor si, si
    .mouthLoop3:
        push si
        call drawvszpixel
        pop si
        inc si
        add cx, 3
        sub dx, 3
        cmp si, 13
        jne .mouthLoop3

    cli
    hlt

drawEye:
    mov al, 0x0F ; C=0x0F
    mov di, 16   ; 16x16 square
    call drawvszpixel

    add cx, 4
    add dx, 4
    mov al, 0x00
    mov di, 8
    call drawvszpixel
    ret

; cl : line number to fill
; ch : what to fill it with
; this function is basically memset lol
drawhline:
    xor ax, ax
    mov al, cl
    mov dx, 0x140
    mul dx

    push ax
    mov bx, ax
    mov ax, 0xA000
    mov ds, ax
    pop ax
    add ax, 320

    mov cl, ch

    .loop2:
        mov word [bx], cx
        add bx, 2
        cmp bx, ax
        jne .loop2

    ret

; bp : line number to fill
; ch : what to fill it with
drawvline:
    xor ax, ax
    xor bx, bx
    mov bx, bp

    mov ax, 320
    push dx
    mov dx, 200
    mul dx
    pop dx
    add ax, bp

    .loop2:
        mov byte [bx], ch
        add bx, 320
        cmp bx, ax
        jne .loop2

    ret

;cx : X coordinate
;dx : Y coordinate
;di : square size
;al : pixel content
drawvszpixel:
    push si
    push cx
    push dx
    push ax

    mov bp, cx
    add bp, di

    mov si, dx
    mov bx, dx
    add si, di

    .xloop:
        .yloop:
            push bx
            call drawpixel
            pop bx
            inc dx
            cmp dx, si
            jne .yloop
        mov dx, bx
        inc cx
        cmp cx, bp
        jne .xloop

    pop ax
    pop dx
    pop cx
    pop si

;cx : X coordinate
;dx : Y coordinate
;al : pixel content
drawpixel:
    push ds
    push ax
    mov ax, 0xA000
    mov ds, ax
    pop ax
    push ax
    push cx
    push dx

    push ax

    push cx
    mov ax, dx ; move Y coord in to dx
    mov cx, 320 ; multiply Y coord by screen width
    mul cx      ; ^--
    pop cx
    add ax, cx  ; add X coord to dx
    mov bx, ax

    pop ax

    push bx
    mov byte [bx], al
    pop bx

    pop dx
    pop cx
    pop ax
    pop ds
    ret

times 2048 - ($-$$) db 0