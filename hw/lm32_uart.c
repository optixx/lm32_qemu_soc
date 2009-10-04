/*
 *  QEMU model of the LatticeMico32 UART block.
 *
 *  Copyright (c) 2010 Michael Walle <michael@walle.cc>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Specification available at:
 *   http://www.latticesemi.com/documents/mico32uart.pdf
 */


#include "sysbus.h"
#include "qemu-char.h"

#define D(x)

#define R_RXTX  0
#define R_IER   1
#define R_IIR   2
#define R_LCR   3
#define R_MCR   4
#define R_LSR   5
#define R_MSR   6
#define R_DIV   7
#define R_MAX   8

#define IER_RBRI (1<<0)
#define IER_THRI (1<<1)
#define IER_RLSI (1<<2)
#define IER_MSI  (1<<3)

#define IIR_STAT (1<<0)
#define IIR_ID0  (1<<1)
#define IIR_ID1  (1<<2)

#define LCR_WLS0 (1<<0)
#define LCR_WLS1 (1<<1)
#define LCR_STB  (1<<2)
#define LCR_PEN  (1<<3)
#define LCR_EPS  (1<<4)
#define LCR_SP   (1<<5)
#define LCR_SB   (1<<6)

#define MCR_DTR  (1<<0)
#define MCR_RTS  (1<<1)

#define LSR_DR   (1<<0)
#define LSR_OE   (1<<1)
#define LSR_PE   (1<<2)
#define LSR_FE   (1<<3)
#define LSR_BI   (1<<4)
#define LSR_THRE (1<<5)
#define LSR_TEMT (1<<6)

#define MSR_DCTS (1<<0)
#define MSR_DDSR (1<<1)
#define MSR_TERI (1<<2)
#define MSR_DDCD (1<<3)
#define MSR_CTS  (1<<4)
#define MSR_DSR  (1<<5)
#define MSR_RI   (1<<6)
#define MSR_DCD  (1<<7)

struct lm32_uart
{
    SysBusDevice busdev;
    CharDriverState *chr;
    qemu_irq irq;

    uint32_t regs[R_MAX];
};

static void uart_update_irq(struct lm32_uart *s)
{
    unsigned int irq;

    if ((s->regs[R_LSR] & (LSR_OE | LSR_PE | LSR_FE | LSR_BI))
            && (s->regs[R_IER] & IER_RLSI))
    {
        irq = 1;
        s->regs[R_IIR] = IIR_ID1 | IIR_ID0;
    } else if ((s->regs[R_LSR] & LSR_DR) && (s->regs[R_IER] & IER_RBRI)) {
        irq = 1;
        s->regs[R_IIR] = IIR_ID1;
    } else if ((s->regs[R_LSR] & LSR_THRE) && (s->regs[R_IER] & IER_THRI)) {
        irq = 1;
        s->regs[R_IIR] = IIR_ID0;
    } else if ((s->regs[R_MSR] & 0x0f) && (s->regs[R_IER] & IER_MSI)) {
        irq = 1;
        s->regs[R_IIR] = 0;
    } else {
        irq = 0;
        s->regs[R_IIR] = IIR_STAT;
    }

    qemu_set_irq(s->irq, irq);
}

static uint32_t uart_read(void *opaque, target_phys_addr_t addr)
{
    struct lm32_uart *s = opaque;
    uint32_t r = 0;
    addr >>= 2;
    switch (addr)
    {
        case R_RXTX:
            r = s->regs[R_RXTX];
            s->regs[R_LSR] &= ~LSR_DR;
            uart_update_irq(s);
            break;

        case R_IIR:
        case R_LSR:
        case R_MSR:
            r = s->regs[addr];
            D(qemu_log("%s addr=%x v=%x\n", __func__, addr, r));
            break;

        /* write only registers */
        case R_IER:
        case R_LCR:
        case R_MCR:
        case R_DIV:
            hw_error("write only register");
            break;

        default:
            hw_error("read from unknown register");
            break;
    }

    D(qemu_log("uart_read addr=%02x value=%08x\n", addr, r));

    return r;
}

static void
uart_write(void *opaque, target_phys_addr_t addr, uint32_t value)
{
    D(qemu_log("uart_write addr=%02x value=%08x\n", addr, value));
    struct lm32_uart *s = opaque;
    unsigned char ch = value;

    addr >>= 2;
    switch (addr)
    {
        case R_RXTX:
            if (s->chr)
                qemu_chr_write(s->chr, &ch, 1);
            break;

        case R_IIR:
        case R_LSR:
        case R_MSR:
            hw_error("read only register");
            break;

        case R_IER:
        case R_LCR:
        case R_MCR:
        case R_DIV:
            s->regs[addr] = value;
            D(qemu_log("%s addr=%x v=%x\n", __func__, addr, value));
            break;

        default:
            hw_error("write to unknown register");
            break;
    }
    uart_update_irq(s);
}

static CPUReadMemoryFunc* const uart_read_fn[] = {
    NULL,
    NULL,
    &uart_read,
};

static CPUWriteMemoryFunc* const uart_write_fn[] = {
    NULL,
    NULL,
    &uart_write,
};

static void uart_rx(void *opaque, const uint8_t *buf, int size)
{
    struct lm32_uart *s = opaque;

    if (s->regs[R_LSR] & LSR_DR)
        s->regs[R_LSR] |= LSR_OE;

    s->regs[R_LSR] |= LSR_DR;
    s->regs[R_RXTX] = *buf;

    uart_update_irq(s);
}

static int uart_can_rx(void *opaque)
{
    struct lm32_uart *s = opaque;

    return !(s->regs[R_LSR] & LSR_DR);
}

static void uart_event(void *opaque, int event)
{
}

static int lm32_uart_init(SysBusDevice *dev)
{
    struct lm32_uart *s = FROM_SYSBUS(typeof (*s), dev);
    int uart_regs;

    sysbus_init_irq(dev, &s->irq);

    uart_regs = cpu_register_io_memory(uart_read_fn, uart_write_fn, s);
    sysbus_init_mmio(dev, R_MAX * 4, uart_regs);

    s->chr = qdev_init_chardev(&dev->qdev);
    if (s->chr)
        qemu_chr_add_handlers(s->chr, uart_can_rx, uart_rx, uart_event, s);

    s->regs[R_LSR] = LSR_THRE | LSR_TEMT;
    return 0;
}

static void lm32_uart_register(void)
{
    sysbus_register_dev("lm32,uart", sizeof (struct lm32_uart),
                        lm32_uart_init);
}

device_init(lm32_uart_register)
