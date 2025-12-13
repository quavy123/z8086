; Animated squares demo for the z8086 TFT SoC
; - Drives ILI9341-compatible TFT on port 7 (word writes, bit8 = D/C)
; - Continuously draws colorful squares of varying sizes at pseudo-random positions
; - Cycles through 8 colors and 8 sizes (8-64 pixels) for a dynamic mosaic effect
; - Built as a flat binary loaded at F000:0000 with reset trampoline at F000:FFF0
; CS=F000h, DS=0000h, SS=0000h

bits 16

%define DATA_SEG 0x40
%define SQUARE_COUNT 0x0
%define LFSR_STATE 0x2

        org 0x0000

; ---------------------------------------------------------------------------
; Reset/setup
start:
        cli
        xor     ax, ax
        mov     ss, ax
        mov     sp, 0xFFFE
        mov     ax, DATA_SEG
        mov     ds, ax
        sti

        call    tft_init

        ; test fill_rect first
        mov     ax, 0000h          ; RED
        mov     bx, 0              ; x0
        mov     cx, 0              ; y0
        mov     dx, 240            ; width
        mov     si, 320            ; height
        call    fill_rect

        ; init animation
        mov     bp, 0              ; color index
.init_size:
        mov     bx, 0
        mov     cx, 40
        mov     dx, 240
        mov     si, 240

        ; draw progressively smaller squares with rotating colors
.draw_square:
        mov     ax, cs:[color_palette + bp]
        call    fill_rect

        inc     bp
        inc     bp
        and     bp, 0x3f            ; 32 colors

        cmp     dx, 0
        jz      .done

        inc     bx
        inc     cx
        dec     dx
        dec     dx
        dec     si
        dec     si

        jmp     .draw_square

.done:
        jmp     .done

; ---------------------------------------------------------------------------
; fill_rect: draw filled rectangle at (BX,CX) of size (DX width, SI height), color AX (RGB565)
; Clobbers: AX,BX,CX,DX,SI,DI,BP
fill_rect:
        push    bp
        mov     bp, sp
        push    ax                 ; [bp-2] color
        push    bx                 ; [bp-4] x0
        push    cx                 ; [bp-6] y0
        push    dx                 ; [bp-8] width
        push    si                 ; [bp-10] height
        push    di

        ; Compute x1 = x0 + w - 1, y1 = y0 + h - 1
        mov     di, bx             ; x0
        add     di, dx
        dec     di                 ; x1
        mov     ax, cx             ; y0
        add     ax, si
        dec     ax                 ; y1 in ax

        ; set window: x0=BX, x1=DI, y0=CX, y1=AX
        push    ax                 ; y1
        push    cx                 ; y0
        push    di                 ; x1
        push    bx                 ; x0
        call    tft_set_addr_window   ; pops 8 bytes on return

        mov     al, 0x2C           ; RAMWR
        call    write_cmd

        mov     dx, [bp-2]         ; color
        mov     si, [bp-10]        ; height
        mov     bx, [bp-8]         ; width
.fill_row:
        mov     cx, bx             ; width counter
.fill_col:
        mov     al, dh
        call    write_data
        mov     al, dl
        call    write_data
        loop    .fill_col
        dec     si
        jnz     .fill_row

        pop     di
        pop     si
        pop     dx
        pop     cx
        pop     bx
        pop     ax
        pop     bp
        ret

; ---------------------------------------------------------------------------
; Helpers

; lfsr_step: 16-bit xorshift; returns AX=new value, updates [LFSR_STATE]
; clobbers: AX, DX
lfsr_step:
        mov     ax, [LFSR_STATE]
        mov     dx, ax
        shl     ax, 7
        xor     ax, dx
        mov     dx, ax
        shr     ax, 9
        xor     ax, dx
        mov     dx, ax
        shl     ax, 8
        xor     ax, dx
        mov     [LFSR_STATE], ax
        ret

; wait_ready: poll port 7 until ready bit set (bit0=1)
wait_ready:
        push    ax
.wr_wait:
        in      ax, 7
        test    al, 1
        jz      .wr_wait
        pop     ax
        ret

; write_cmd: AL = command byte
write_cmd:
        call    wait_ready
        xor     ah, ah             ; DC=0
        out     7, ax              ; word write: AH=0, AL=byte
        ret

; write_data: AL = data byte
write_data:
        call    wait_ready
        mov     ah, 1              ; DC=1
        out     7, ax
        ret

; tft_set_addr_window(x0,x1,y0,y1)  (4 words on stack, little endian)
; Stack layout after push bp/mov bp,sp:
; [bp+4]=x0, [bp+6]=x1, [bp+8]=y0, [bp+10]=y1
tft_set_addr_window:
        push    bp
        mov     bp, sp
        ; CASET 0x2A
        mov     al, 0x2A
        call    write_cmd
        mov     al, [bp+5]         ; x0_hi
        call    write_data
        mov     al, [bp+4]         ; x0_lo
        call    write_data
        mov     al, [bp+7]         ; x1_hi
        call    write_data
        mov     al, [bp+6]         ; x1_lo
        call    write_data
        ; PASET 0x2B
        mov     al, 0x2B
        call    write_cmd
        mov     al, [bp+9]         ; y0_hi
        call    write_data
        mov     al, [bp+8]         ; y0_lo
        call    write_data
        mov     al, [bp+11]        ; y1_hi
        call    write_data
        mov     al, [bp+10]        ; y1_lo
        call    write_data
        mov     sp, bp
        pop     bp
        ret     8

; crude delay ~millisecond-ish (tuned for FPGA clock; not exact)
; delay_short: CX inner loop repeated DX times
delay_short:
        push    ax
.d1:    mov     ax, 0x2000
.d2:    dec     ax
        jnz     .d2
        dec     dx
        jnz     .d1
        pop     ax
        ret

; tft_init: minimal init sequence for ILI9341
tft_init:
        ; SWRESET
        mov     al, 0x01
        call    write_cmd
        mov     dx, 3
        call    delay_short
        ; SLPOUT
        mov     al, 0x11
        call    write_cmd
        mov     dx, 50
        call    delay_short
        ; PIXFMT = 16bpp
        mov     al, 0x3A
        call    write_cmd
        mov     al, 0x55
        call    write_data
        ; MADCTL = 0x48
        mov     al, 0x36
        call    write_cmd
        mov     al, 0x48
        call    write_data
        ; DISPON
        mov     al, 0x29
        call    write_cmd
        ret

; ---------------------------------------------------------------------------
; Data
color_palette:
        dw 0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF, 0xFFFF, 0xFD20
        ; RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE
        dw 0x07E1, 0xFFE1, 0xF81F, 0x07FF, 0xFDFD, 0xBDF7, 0x8410, 0x4208
        ; SPRING GREEN, LIME, PURPLE, AQUA, GRAY, SILVER, NAVY, TEAL
        dw 0xFC10, 0xA145, 0x3800, 0x0010, 0x03EF, 0x7C00, 0x03FF, 0x7FE0
        ; ORANGE-RED, OLIVE, BROWN, DARK BLUE, SKY BLUE, BRIGHT RED, CYAN2, YELLOW-GREEN
        dw 0x5F1C, 0x4210, 0x3186, 0x294A, 0x18C6, 0x1F1F, 0x6B4D, 0x56B5
        ; LIGHT PINK, DARK GRAY, LIGHT GRAY, MEDIUM GRAY, PALE BLUE, HOT PINK, DARK GREEN, LIME GREEN
        dw 0x0C63, 0x15F9, 0x3B0A, 0x6B5A, 0x4631, 0x7C1F, 0x03E0, 0x7BEF
        ; MIDNIGHT, PALE GREEN, MED YELLOW, PEACH, MOCHA, BRIGHT PINK, GREEN2, SAND
square_count    dw 0
lfsr_state      dw 0xACE1

; ---------------------------------------------------------------------------
; Reset vector trampoline at F000:FFF0
        times 0xFFF0 - ($-$$) db 0x00
        jmp     0xF000:0x0000
        times 0x10000 - ($-$$) - 1 db 0x00
        db 0xFF

