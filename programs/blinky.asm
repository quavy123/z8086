; 8086 "blinky" program
; NASM format, flat binary
; Loaded at F000:0000. Reset starts executing at physical FFFF0h (F000:FFF0),
; so place a FAR JMP at 0xFFF0 that jumps to the entry at 0x0000.

[BITS 16]           ; 16 bit code

        org 0x0000

; Entry point at F000:0000
start:
        cli                     ; set up stack safely before using CALL
        xor     ax, ax
        mov     ss, ax          ; stack in segment 0
        mov     sp, 0xFFFE      ; word-aligned stack near top of segment 0
        mov     ds, ax          ; working/data segment 0 (heap/stack here)
        sti

        xor     ax, ax          ; AL = 0 pattern

.loop:
        out     5, al           ; write lower 8 bits to I/O port 5
        call    delay
        inc     al              ; change pattern (count up)
        mov     [0x1000], al    ; store counter in memory
        jmp     short .loop

; Simple software delay for visible blinking
; Clobbers: CX, DX
delay:
        push    cx
        push    dx
        mov     cx, 0x1
.d1:
        mov     dx, 0x0000
.d2:
        dec     dx
        jnz     .d2
        loop    .d1
        pop     dx
        pop     cx
        ret

; ---------------------------------------------------------------------------
; Reset vector trampoline at F000:FFF0
        times 0xFFF0 - ($-$$) db 0x00
        jmp     0xF000:0x0000   ; far immediate jump (NASM 3 syntax)
        ; Pad out to the last byte with zeros, then end with 0xFF
        times 0x10000 - ($-$$) - 1 db 0x00
        db 0xFF
