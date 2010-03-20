/*
 *  LatticeMico32 CPU interrupt controller logic.
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
 */

#include "hw.h"
#include "sysbus.h"
#include "lm32_pic.h"

#define D(x)

struct lm32_pic
{
    SysBusDevice busdev;
    qemu_irq parent_irq;
    uint32_t im;        /* interrupt mask */
    uint32_t ip;        /* interrupt pending */
    uint32_t irq_state;
};

static struct lm32_pic *lm32_pic;

static void update_irq(struct lm32_pic *s)
{
    s->ip |= s->irq_state;

    if (s->ip & s->im)
        qemu_irq_raise(s->parent_irq);
    else
        qemu_irq_lower(s->parent_irq);
}

static void irq_handler(void *opaque, int irq, int level)
{
    struct lm32_pic *s = opaque;

    if (level) {
        s->irq_state |= (1 << irq);
    } else {
        s->irq_state &= ~(1 << irq);
    }

    update_irq(s);
}

void lm32_pic_set_im(struct lm32_pic *s, uint32_t im)
{
    s->im = im;

    update_irq(s);
}

void lm32_pic_set_ip(struct lm32_pic *s, uint32_t ip)
{
    /* ack interrupt */
    s->ip &= ~ip;

    update_irq(s);
}

uint32_t lm32_pic_get_im(struct lm32_pic *s)
{
    return s->im;
}

uint32_t lm32_pic_get_ip(struct lm32_pic *s)
{
    return s->ip;
}

static int lm32_pic_init(SysBusDevice *dev)
{
    struct lm32_pic *s = FROM_SYSBUS(typeof (*s), dev);
    qdev_init_gpio_in(&dev->qdev, irq_handler, 32);
    sysbus_init_irq(dev, &s->parent_irq);

    lm32_pic = s;

    return 0;
}

static SysBusDeviceInfo lm32_pic_info = {
    .init = lm32_pic_init,
    .qdev.name = "lm32,pic",
    .qdev.size = sizeof(struct lm32_pic),
    .qdev.props = (Property[]) {
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void lm32_pic_register(void)
{
    sysbus_register_withprop(&lm32_pic_info);
}

device_init(lm32_pic_register)
