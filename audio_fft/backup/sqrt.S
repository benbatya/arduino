;*****************************************************************************
;*
;* sqrt16 V 2.00 for use with processors with mul instruction
;* From http://www.avrfreaks.net/forum/square-root-and-standard-deviation-functions-needed
;* r16 - SQUARE LOW  BYTE
;* r17 - SQUARE HIGH BYTE
;*
;* r25 - SQRT, returned in r16
;*
;* Low  registers used   : r0, r1
;* High registers used   : r16 - r17, r24, r25
;*
;*****************************************************************************
; .area text
;uint8_t sqrt16(uint16_t A)
;_sqrt16::
.global sqrt16
.func sqrt16
  st    -y,r0
  st    -y,r1
  st    -y,r24
  st    -y,r25
;{ 
;   uint8_t bit =0x80;
  ldi r24,0x80 
;   uint8_t x =0;
  ldi r25,0
;   do
sqrt16loop:
;   { 
;      x += bit ;
  add r25,r24
;      if (((uint16_t)x * x) > A)
  mul r25,r25
  cp  r16,r0
  cpc r17,r1
  brcc continue
;      { 
;         x -= bit ;
  sub r25,r24
;      }
continue:
;      bit >>= 1 ;
  lsr r24
;   } 
;   while (bit != 0);
  brne sqrt16loop
;   return (x) ;
  mov r16,r25
;} 
  ld    r25,y+
  ld    r24,y+
  ld    r1,y+
  ld    r0,y+
  ret

