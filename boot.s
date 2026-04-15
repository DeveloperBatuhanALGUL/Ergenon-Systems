.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.extern _stack_top
.extern __data_load_start
.extern __data_start
.extern __data_end
.extern __bss_start
.extern __bss_end
.extern kernel_main

.section .isr_vector, "a", %progbits
.type g_pfnVectors, %object
.size g_pfnVectors, .-g_pfnVectors
g_pfnVectors:
    .word _stack_top
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word MemManage_Handler
    .word BusFault_Handler
    .word UsageFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word DebugMon_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler

.text
.thumb_func
.type Reset_Handler, %function
Reset_Handler:
    ldr r0, =__data_load_start
    ldr r1, =__data_start
    ldr r2, =__data_end
    cmp r1, r2
    beq zero_bss
copy_data:
    ldr r3, [r0], #4
    str r3, [r1], #4
    cmp r1, r2
    bne copy_data

zero_bss:
    ldr r0, =__bss_start
    ldr r1, =__bss_end
    mov r2, #0
    cmp r0, r1
    beq call_kernel
zero_loop:
    str r2, [r0], #4
    cmp r0, r1
    bne zero_loop

call_kernel:
    bl kernel_main
    b .

.thumb_func
.weak NMI_Handler
NMI_Handler: b .
.thumb_func
.weak HardFault_Handler
HardFault_Handler: b .
.thumb_func
.weak MemManage_Handler
MemManage_Handler: b .
.thumb_func
.weak BusFault_Handler
BusFault_Handler: b .
.thumb_func
.weak UsageFault_Handler
UsageFault_Handler: b .
.thumb_func
.weak SVC_Handler
SVC_Handler: b .
.thumb_func
.weak DebugMon_Handler
DebugMon_Handler: b .
.thumb_func
.weak PendSV_Handler
PendSV_Handler: b .
.thumb_func
.weak SysTick_Handler
SysTick_Handler: b .
