#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
  name isr_asm
  section .text:CODE
  extern iterCom
  public isr_asm_start
isr_asm_start:
 ; push {lr} ;
  MOV R1, #1
  MOV32 R0, #0x40031000
  STR R1, [R0, #0x24]
  MOV32 R1, iterCom
  LDR R0, [R1]
  ADD R0, R0, #1
  STR R0, [R1]
  bx lr
  ;pop {pc} ; return
  end