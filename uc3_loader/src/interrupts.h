/* interrupts.h */

#ifndef _interrupts_h
#define _interrupts_h

#ifdef __cplusplus
extern "C" {
#endif



#define barrier()   asm volatile("" ::: "memory")

#define interruptsEnable() \
    do { \
        barrier(); \
        __builtin_csrf(AVR32_SR_GM_OFFSET); \
    } while (0)

#define interruptsDisable() \
    do { \
         __builtin_ssrf(AVR32_SR_GM_OFFSET); \
         barrier(); \
    } while (0)

#define interruptsEnableLevel(level) \
    do { \
        barrier(); \
        __builtin_csrf(AVR32_SR_I##level##M_OFFSET)); \
    } while (0)

#define interruptsDisableLevel(level) \
    do { \
        __builtin_ssrf(AVR32_SR_I##level##M_OFFSET)); \
        barrier(); \
    } while (0)


typedef void (*interruptHandler)(void);


/* used by exception.S */
interruptHandler _get_interrupt_handler(uint32_t intLevel);

void interruptsInit(void);

void interruptsRegister(interruptHandler handler, uint32_t irq,
                        uint32_t level);

uint32_t interruptsSaveAndDisable(void);

void interruptsRestore(uint32_t flags);


#ifdef __cplusplus
}
#endif

#endif /* _interrupts_h */


