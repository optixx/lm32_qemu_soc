/*
 *  QEMU model of the Milkymist UART block.
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
 *   http://www.milkymist.org/doc/uart.pdf
 */


#include "sysbus.h"
#include "qemu-char.h"

#define D(x) 
#define R_UCR   0
#define R_RXTX  1
#define R_MAX   2

//[ 0 | 0 | 0 | tx_busy | 0 | 0 | rx_error | rx_avail ]

#define rx_none  0
#define rx_avail (1 << 0)
#define rx_error (1 << 1)
#define rx_busy  (1 << 4)

struct lm32_soc_uart
{
    SysBusDevice busdev;
    CharDriverState *chr;
    qemu_irq rx_irq;
    qemu_irq tx_irq;

    uint32_t regs[R_MAX];
};

static uint32_t uart_read(void *opaque, target_phys_addr_t addr)
{
    struct lm32_soc_uart *s = opaque;
    uint32_t r = 0;
    addr >>= 2;
    switch (addr)
    {
        case R_RXTX:
            r = s->regs[R_RXTX];
            s->regs[R_UCR] = rx_none;
            break;
        case R_UCR:
            r = s->regs[R_UCR];
            break;
        default:
            hw_error("read from unknown register");
            break;
    }
    D(qemu_log("uart_read addr=" TARGET_FMT_plx " value=%08x\n", addr, r));
    return r;
}

static void
uart_write(void *opaque, target_phys_addr_t addr, uint32_t value)
{
    D(qemu_log("uart_write addr=" TARGET_FMT_plx " value=%08x\n", addr, value));
    struct lm32_soc_uart *s = opaque;
    unsigned char ch = value;

    addr >>= 2;
    switch (addr)
    {
        case R_RXTX:
            if (s->chr)
                qemu_chr_write(s->chr, &ch, 1);
            break;
        case R_UCR:
            s->regs[addr] = value;
            break;

        default:
            hw_error("write to unknown register");
            break;
    }
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
    struct lm32_soc_uart *s = opaque;
    s->regs[R_RXTX] = *buf;
    s->regs[R_UCR] = rx_avail;
}

static int uart_can_rx(void *opaque)
{
    return 1;
}

static void uart_event(void *opaque, int event)
{
}

static int lm32_soc_uart_init(SysBusDevice *dev)
{
    struct lm32_soc_uart *s = FROM_SYSBUS(typeof (*s), dev);
    int uart_regs;

    sysbus_init_irq(dev, &s->rx_irq);
    sysbus_init_irq(dev, &s->tx_irq);

    uart_regs = cpu_register_io_memory(uart_read_fn, uart_write_fn, s);
    sysbus_init_mmio(dev, R_MAX * 4, uart_regs);

    s->chr = qdev_init_chardev(&dev->qdev);
    if (s->chr)
        qemu_chr_add_handlers(s->chr, uart_can_rx, uart_rx, uart_event, s);

    return 0;
}

static void lm32_soc_uart_register(void)
{
    sysbus_register_dev("lm32_soc,uart", sizeof (struct lm32_soc_uart),
                        lm32_soc_uart_init);
}

device_init(lm32_soc_uart_register)
