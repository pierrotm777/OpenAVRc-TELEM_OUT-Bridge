/*
 * ProtocolDebug_AVR.h
 *
 * Debug UART0 minimal pour OpenAVRc / AVR.
 * Compatible C/C++ et avr-libc.
 */

#ifndef PROTOCOL_DEBUG_AVR_H
#define PROTOCOL_DEBUG_AVR_H

#ifndef PROTOCOL_DEBUG
#define PROTOCOL_DEBUG 1
#endif

#ifndef DBG_BAUD
#define DBG_BAUD 115200UL
#endif

#if PROTOCOL_DEBUG

#include <avr/io.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

static inline void dbg_uart_init(void)
{
    static uint8_t initialized = 0U;

    if(initialized)
        return;

    initialized = 1U;

    UCSR0A = _BV(U2X0);

    const uint16_t ubrr =
        (uint16_t)((F_CPU + (DBG_BAUD * 4UL)) /
                   (DBG_BAUD * 8UL) - 1UL);

    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);

    UCSR0B |= _BV(TXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}

static inline void dbg_putc_raw(char c)
{
    dbg_uart_init();

    if(c == '\n')
    {
        while(!(UCSR0A & _BV(UDRE0))) {}
        UDR0 = '\r';
    }

    while(!(UCSR0A & _BV(UDRE0))) {}
    UDR0 = (uint8_t)c;
}

static int dbg_putchar(char c, FILE *stream)
{
    (void)stream;
    dbg_putc_raw(c);
    return 0;
}

static FILE dbg_stream;
static uint8_t dbg_stream_initialized = 0U;

static inline void dbg_stream_init(void)
{
    if(dbg_stream_initialized)
        return;

    fdev_setup_stream(&dbg_stream,
                      dbg_putchar,
                      NULL,
                      _FDEV_SETUP_WRITE);

    dbg_stream_initialized = 1U;
}

static inline void dbg_print(const char *s)
{
    if(!s)
        return;

    while(*s)
        dbg_putc_raw(*s++);
}

static inline void dbg_println(const char *s)
{
    if(s)
        dbg_print(s);

    dbg_putc_raw('\n');
}

static inline int dbg_printf(const char *fmt, ...)
{
    va_list ap;
    int ret;

    dbg_uart_init();
    dbg_stream_init();

    va_start(ap, fmt);
    ret = vfprintf(&dbg_stream, fmt, ap);
    va_end(ap);

    return ret;
}

#define DBG_INIT()      dbg_uart_init()
#define DBG_PRINT(s)    dbg_print((s))
#define DBG_PRINTLN(s)  dbg_println((s))
#define DBG_PRINTF(...) dbg_printf(__VA_ARGS__)

#else

#define DBG_INIT()      do {} while(0)
#define DBG_PRINT(s)    do { (void)sizeof(s); } while(0)
#define DBG_PRINTLN(s)  do { (void)sizeof(s); } while(0)
#define DBG_PRINTF(...) do {} while(0)

#endif

#endif
