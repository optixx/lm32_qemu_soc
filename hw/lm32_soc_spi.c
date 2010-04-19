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

#define D(x) x

#define R_MAX 1


#include "sysbus.h"
#include "qemu-char.h"

struct lm32_soc_spi
{
    SysBusDevice busdev;
    uint32_t regs[R_MAX];
};

static uint32_t spi_read(void *opaque, target_phys_addr_t addr)
{
    D(printf("%s: addr=%08x\n", __func__,addr));
    struct lm32_soc_spi *s = opaque;
    uint32_t r = 0;
    addr >>= 2;
    switch (addr)
    {
        case 0:
            r = s->regs[0];
            break;
        default:
            hw_error("%s: read from unknown register", __func__);
            break;
    }
    return r;
}

static void
spi_write(void *opaque, target_phys_addr_t addr, uint32_t value)
{
    D(printf("%s: addr=%08x value=%08x\n", __func__,addr, value));
    struct lm32_soc_spi *s = opaque;
    addr >>= 2;
    switch (addr)
    {
        case 0:
            s->regs[0] = value;
            break;
        default:
            hw_error("%s: write to unknown register", __func__);
            break;
    }
}

static CPUReadMemoryFunc* const spi_read_fn[] = {
    NULL,
    NULL,
    &spi_read,
};

static CPUWriteMemoryFunc* const spi_write_fn[] = {
    NULL,
    NULL,
    &spi_write,
};

/*
static void spi_rx(void *opaque, const uint8_t *buf, int size)
{
    //struct lm32_soc_spi *s = opaque;
}

static void spi_event(void *opaque, int event)
{
}
*/

static int lm32_soc_spi_init(SysBusDevice *dev)
{
    struct lm32_soc_spi *s = FROM_SYSBUS(typeof (*s), dev);
    int spi_regs;

    spi_regs = cpu_register_io_memory(spi_read_fn, spi_write_fn, s);
    sysbus_init_mmio(dev, R_MAX * 4, spi_regs);

    return 0;
}

static void lm32_soc_spi_register(void)
{
    sysbus_register_dev("lm32_soc,spi", sizeof (struct lm32_soc_spi),
                        lm32_soc_spi_init);
}

device_init(lm32_soc_spi_register)
