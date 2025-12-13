; 8086 LCD demo to drive ILI9341 via port 7 SPI (with RS in bit 8)
; NASM flat binary; loaded at F000:0000 with reset trampoline at F000:FFF0

[BITS 16]

        org 0x0000

start:
        cli
        xor     ax, ax
        mov     ss, ax
        mov     sp, 0xFFFE
        mov     ds, ax
        sti

        call    tft_init

        ; Draw 8 color bars of 40px height across full width (240)
        mov     bx, 0          ; y = 0
        mov     cx, 40         ; h = 40
        mov     dx, 0x8000     ; MAROON
        call    fill_bar

        add     bx, 40
        mov     dx, 0x780F     ; PURPLE
        call    fill_bar

        add     bx, 40
        mov     dx, 0x7BE0     ; OLIVE
        call    fill_bar

        add     bx, 40
        mov     dx, 0xC618     ; LIGHTGREY
        call    fill_bar

        add     bx, 40
        mov     dx, 0x001F     ; BLUE
        call    fill_bar

        add     bx, 40
        mov     dx, 0x07E0     ; GREEN
        call    fill_bar

        add     bx, 40
        mov     dx, 0x07FF     ; CYAN
        call    fill_bar

        add     bx, 40
        mov     dx, 0xF800     ; RED
        call    fill_bar

        hlt
; .halt:
;         jmp     .halt

; fill_bar: draw horizontal bar at y=BX, height=40, width=240, color=DX (RGB565)
; Clobbers: AX, CX, SI, DI
fill_bar:
        push    bx
        push    dx
        ; set window: x0=0,x1=239 ; y0=BX, y1=BX+39
        mov     ax, bx
        add     ax, 39
        push    ax              ; y1 = bx + 39
        push    bx              ; y0 = bx
        mov     ax, 239
        push    ax              ; x1 = 239
        xor     ax, ax
        push    ax              ; x0 = 0
        call    tft_set_addr_window

        ; issue RAMWR (0x2C)
        mov     al, 0x2C
        call    write_cmd

        ; stream pixels: 240*40 = 9600 pixels
        mov     si, 40          ; rows
.row_loop:
        mov     cx, 240
.col_loop:
        ; send high byte then low byte of color DX
        mov     al, dh
        call    write_data
        mov     al, dl
        call    write_data
        loop    .col_loop
        dec     si
        jnz     .row_loop

        pop     dx
        pop     bx
        ret

; ---------------- SPI/ILI9341 helpers ----------------

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
        xor     ah, ah          ; DC=0
        out     7, ax           ; word write: AH=0, AL=byte
        ret

; write_data: AL = data byte
write_data:
        call    wait_ready
        mov     ah, 1           ; DC=1
        out     7, ax
        ret

; tft_set_addr_window(x0_lo, x0_hi, x1_lo, x1_hi, y0_lo, y0_hi, y1_lo, y1_hi)
; Expects 8 words pushed; due to 8086 little-endian stack, after push bp/mov bp,sp,
; the layout is (offsets from BP):
tft_set_addr_window:
        push    bp
        mov     bp, sp
        ; CASET 0x2A
        mov     al, 0x2A
        call    write_cmd
        ; [bp] is old bp, [bp+2] is return address
        ; [bp+4] is x0, [bp+6] is x1, [bp+8] is y0, [bp+10] is y1
        mov     al, [bp+5]      ; x0_hi
        call    write_data
        mov     al, [bp+4]      ; x0_lo
        call    write_data
        mov     al, [bp+7]      ; x1_hi
        call    write_data
        mov     al, [bp+6]      ; x1_lo
        call    write_data
        ; PASET 0x2B
        mov     al, 0x2B
        call    write_cmd
        mov     al, [bp+9]     ; y0_hi
        call    write_data
        mov     al, [bp+8]     ; y0_lo
        call    write_data
        mov     al, [bp+11]    ; y1_hi
        call    write_data
        mov     al, [bp+10]    ; y1_lo
        call    write_data
        mov     sp, bp
        pop     bp             ; restore bp
        ret     8              ; return and pop 4 words (8 bytes)

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

; tft_init: minimal init sequence
tft_init:
        ; SWRESET
        mov     al, 0x01
        call    write_cmd
        mov     dx, 3           ; small delay
        call    delay_short
        ; SLPOUT
        mov     al, 0x11
        call    write_cmd
        mov     dx, 50          ; longer delay
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
; Reset vector trampoline at F000:FFF0
        times 0xFFF0 - ($-$$) db 0x00
        jmp     0xF000:0x0000
        times 0x10000 - ($-$$) - 1 db 0x00
        db 0xFF
