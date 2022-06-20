#include <stdint.h>

// Symbols from linker
extern unsigned _data_loadaddr, _data, _edata, _ebss, _stack;

// IVT entry point
typedef void (*vector_table_entry_t)(void);

// Interrupt Vector Table
typedef struct {
    unsigned int *initial_sp_value;     // Initial stack pointer value.
    vector_table_entry_t reset;
    vector_table_entry_t nmi;
    vector_table_entry_t hard_fault;
    vector_table_entry_t memory_manage_fault; // not in CM0
    vector_table_entry_t bus_fault;           // not in CM0
    vector_table_entry_t usage_fault;         // not in CM0
    vector_table_entry_t reserved_x001c[4];
    vector_table_entry_t sv_call;
    vector_table_entry_t debug_monitor;       // not in CM0
    vector_table_entry_t reserved_x0034;
    vector_table_entry_t pend_sv;
    vector_table_entry_t systick;
} vector_table_t;

// Placeholder handler
void nmi_handler(void) {
    while (1);
}

__attribute__((naked)) void hard_fault_handler(void) {
    /*
    __asm( ".syntax unified\n"
    "MOVS R0, #4 \n"
    "MOV R1, LR \n"
    "TST R0, R1 \n"
    "BEQ _MSP \n"
    "MRS R0, PSP \n"
    "B HardFault_HandlerC \n"
    "_MSP: \n"
    "MRS R0, MSP \n"
    "B HardFault_HandlerC \n"
    ".syntax divided\n") ;
    */
    while (1);
}

/**
* HardFaultHandler_C:
* This is called from the HardFault_HandlerAsm with a pointer the Fault stack
* as the parameter. We can then read the values from the stack and place them
* into local variables for ease of reading.
* We then read the various Fault Status and Address Registers to help decode
* cause of the fault.
* The function ends with a BKPT instruction to force control back into the debugger
*/

void HardFault_HandlerC(unsigned long *hardfault_args){
    
    /*
    volatile unsigned long stacked_r0 ;
    volatile unsigned long stacked_r1 ;
    volatile unsigned long stacked_r2 ;
    volatile unsigned long stacked_r3 ;
    volatile unsigned long stacked_r12 ;
    volatile unsigned long stacked_lr ;
    volatile unsigned long stacked_pc ;
    volatile unsigned long stacked_psr ;
    volatile unsigned long _CFSR ;
    volatile unsigned long _HFSR ;
    volatile unsigned long _DFSR ;
    volatile unsigned long _AFSR ;
    volatile unsigned long _BFAR ;
    volatile unsigned long _MMAR ;

    stacked_r0 = ((unsigned long)hardfault_args[0]) ;
    stacked_r1 = ((unsigned long)hardfault_args[1]) ;
    stacked_r2 = ((unsigned long)hardfault_args[2]) ;
    stacked_r3 = ((unsigned long)hardfault_args[3]) ;
    stacked_r12 = ((unsigned long)hardfault_args[4]) ;
    stacked_lr = ((unsigned long)hardfault_args[5]) ;
    stacked_pc = ((unsigned long)hardfault_args[6]) ;
    stacked_psr = ((unsigned long)hardfault_args[7]) ;

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;
    // Hard Fault Status Register
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;
    // Debug Fault Status Register
    _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;
    // Auxiliary Fault Status Register
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;
    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
    // Bus Fault Address Register
    _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;
    __asm("BKPT #0\n") ; // Break into the debugger
    
    */
}


void memory_manage_fault_handler(void) {
    while (1);
}
void bus_fault_handler(void) {
    while (1);
}
void usage_fault_handler(void) {
    while (1);
}
void debug_monitor_handler(void) {
    while (1);
}
void sv_call_handler(void) {
    while (1);
}
void pend_sv_handler(void) {
    while (1);
}
void systick_handler(void) {
    while (1);
}

// Less common symbols exported by the linker script(s):
typedef void (*funcp_t) (void);

// Prototype main()
void main(void);

// MCU enters reset_handler on boot
void __attribute__ ((naked)) reset_handler(void) {
    volatile unsigned *src, *dest;

    for (src = &_data_loadaddr, dest = &_data;
        dest < &_edata;
        src++, dest++) {
        *dest = *src;
    }

    while (dest < &_ebss)
        *dest++ = 0;

    // Ensure 8-byte alignment of stack pointer on interrupts
    // Enabled by default on most Cortex-M parts, but not M3 r1
    volatile uint32_t *_scb_ccr = (uint32_t*) 0xE000ED14U;
    *_scb_ccr |= (1 << 9);

    // Jump to bootloader program
    main();
}

// Assign functions to Interrupt Vector Table
__attribute__ ((section(".vectors")))
vector_table_t vector_table = {
    .initial_sp_value           = &_stack, // Populated by linker
    .reset                      = reset_handler,
    .nmi                        = nmi_handler,
    .hard_fault                 = hard_fault_handler,
    .memory_manage_fault        = memory_manage_fault_handler,
    .bus_fault                  = bus_fault_handler,
    .usage_fault                = usage_fault_handler,
    .debug_monitor              = debug_monitor_handler,
    .sv_call                    = sv_call_handler,
    .pend_sv                    = pend_sv_handler,
    .systick                    = systick_handler,
};