/*
 *  LatticeMico32 CPU interrupt wrapper logic.
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
#include "pc.h"

#define D(x)

void pic_info(Monitor *mon)
{}
void irq_info(Monitor *mon)
{}

static void lm32_pic_cpu_handler(void *opaque, int irq, int level)
{
    CPUState *env = (CPUState *)opaque;

	env->ip |= (1 << irq);
	if ((env->ie & IE_IE) && (env->ip & env->im))
		cpu_interrupt(env, CPU_INTERRUPT_HARD);
	else
		cpu_reset_interrupt(env, CPU_INTERRUPT_HARD);
}

qemu_irq *lm32_pic_init_cpu(CPUState *env);
qemu_irq *lm32_pic_init_cpu(CPUState *env)
{
    return qemu_allocate_irqs(lm32_pic_cpu_handler, env, 32);
}
