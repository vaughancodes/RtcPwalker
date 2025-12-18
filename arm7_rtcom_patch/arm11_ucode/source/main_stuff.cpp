#include <stddef.h>

#include "a11ucode.h"
#include "main_stuff.h"
#include "i2c.h"

// IR I2C registers
#define REG_FIFO	0x00	// Receive / Transmit Holding Register
#define REG_DLL		0x00	// Baudrate Divisor Latch Register Low
#define REG_IER		0x08	// Interrupt Enable Register
#define REG_DLH		0x08	// Baudrate Divisor Latch Register High
#define REG_FCR		0x10	// FIFO Control Register
#define REG_EFR		0x10	// Enhanced Feature Register
#define REG_LCR		0x18	// Line Control Register
#define REG_MCR		0x20	// Modem Control Register
#define REG_LSR		0x28	// Line Status Register
#define REG_TXLVL	0x40	// Transmitter FIFO Level Register
#define REG_RXLVL	0x48	// Receiver FIFO Level Register
#define REG_IOSTATE	0x58	// IOState Register
#define	REG_EFCR	0x78	// Extra Features Control Register

#define	RX_MAX_WAIT	40

u8 ir_buffer[136];
static u8 ir_buffer_size = 0;

static inline void ir_delay(volatile u32 count)
{
	while (count--) { __asm__ volatile(""); }
}

static inline void ir_flush_fifo()
{
	// Drain any garbage bytes currently buffered
	for (int k = 0; k < 32; k++) {
		u8 lvl = I2C_read(REG_RXLVL);
		if (!lvl) break;

		u8 tmp[64];
		if (lvl > sizeof(tmp)) lvl = sizeof(tmp);
		I2C_readArray(REG_FIFO, tmp, lvl);
	}
}

static inline void ir_write_div10_8n1()
{
	// Force 8N1, DLAB clear
	I2C_write(REG_LCR, 0x03);

	// DLAB=1 to access DLL/DLH
	I2C_write(REG_LCR, 0x03 | BIT(7));

	I2C_write(REG_DLL, 10);
	I2C_write(REG_DLH, 0);

	// Back to 8N1, DLAB clear
	I2C_write(REG_LCR, 0x03);
}

static inline void ir_configure_div10_now()
{
	// Hard stop + clear
	I2C_write(REG_EFCR, 0x06);      // disable TX/RX
	I2C_write(REG_FCR,  0x00);      // disable FIFO
	I2C_write(REG_IER,  0x00);      // disable sleep mode
	I2C_write(REG_IOSTATE, 0x00);   // sane IO state

	ir_delay(20000);

	// Program divisor=10 and known-good framing (8N1)
	ir_write_div10_8n1();

	// Re-arm RX
	I2C_write(REG_FCR,  0x07);      // reset+enable FIFO
	I2C_write(REG_EFCR, 0x04);      // enable receiver

	ir_delay(20000);
	ir_flush_fifo();

	// Re-write divisor=10 (same value) to force latch on picky units
	ir_write_div10_8n1();

	I2C_write(REG_FCR,  0x07);
	I2C_write(REG_EFCR, 0x04);
	ir_flush_fifo();
}

void ir_init() {
	I2C_init();
	ir_configure_div10_now();
}

void ir_beginComm() {
	static u8 inited = 0;
	if (!inited)
		ir_init();
	inited = 1;

	// Disable sleep mode
	I2C_write(REG_IER, 0);
	// IOState must be 0
	I2C_write(REG_IOSTATE, 0);
	// Reset and enable FIFO
	I2C_write(REG_FCR, 0x07);

	// Ensure we are listening
	I2C_write(REG_EFCR, 0x04);

	ir_buffer_size = 0;
}

void ir_endComm() {
	// Reset and disable FIFO
	I2C_write(REG_FCR, 0x06);
	// Enable sleep mode
	I2C_write(REG_IER, BIT(4));
	I2C_write(REG_IOSTATE, BIT(0));
}

static inline u8 rx(u16 timeout) {
	u8 *ptr = ir_buffer, tc = 0, rxlvl;
	u16 i;

	do {
		i = 0;
		while (!(rxlvl = I2C_read(REG_RXLVL)) && i++ < timeout) {
			ir_delay(300);
		}
		if (i - 1 == timeout)
			break;
		timeout = RX_MAX_WAIT;

		I2C_readArray(REG_FIFO, ptr, rxlvl);
		ptr += rxlvl;
		tc += rxlvl;
	} while (tc < 136);

	return tc;
}

// Send data using IR and start listening for incoming data
void ir_send(u8 size) {
	u8 *ptr = ir_buffer, tc;

	// Enable transmitter / Disable receiver
	I2C_write(REG_EFCR, 0x02);

	if (size <= 64) {
		I2C_writeArray(REG_FIFO, ptr, size);
	} else {
		u8 txlvl, to_send;
		do {
			txlvl = I2C_read(REG_TXLVL);
			if (txlvl) {
				to_send = size > txlvl ? txlvl : size;
				I2C_writeArray(REG_FIFO, ptr, to_send);
				ptr += to_send;
				size -= to_send;
			}
		} while (size);
	}

	// Wait until THR and TSR are empty
	while (!(I2C_read(REG_LSR) & BIT(6)));

	// Enable receiver / Disable transmitter
	I2C_write(REG_EFCR, 0x04);

	tc = rx(1000);

	// Keep receiver enabled
	I2C_write(REG_EFCR, 0x04);

	ir_buffer_size = tc;
}

u8 ir_recv() {
	u8 tc;
	if (ir_buffer_size) {
		tc = ir_buffer_size;
		ir_buffer_size = 0;
		return tc;
	}

	u8 *ptr = ir_buffer;

	// Reset and enable FIFO
	I2C_write(REG_FCR, 0x07);
	// Enable receiver
	I2C_write(REG_EFCR, 0x04);

	tc = rx(RX_MAX_WAIT);

	return tc;
}
