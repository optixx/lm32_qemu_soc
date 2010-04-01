/*
 *  QEMU models for LatticeMico32 uclinux and evr32 boards.
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

#include "sysbus.h"
#include "hw.h"
#include "net.h"
#include "flash.h"
#include "sysemu.h"
#include "devices.h"
#include "boards.h"
#include "loader.h"
#include "elf.h"

static uint32_t bootstrap_pc;

/* XXX */
qemu_irq *lm32_pic_init_cpu(CPUState *env);

static void main_cpu_reset(void *opaque)
{
    CPUState *env = opaque;
    cpu_reset(env);
    env->pc = bootstrap_pc;
}

static void
lm32_evr_init(ram_addr_t ram_size_not_used,
                          const char *boot_device,
                          const char *kernel_filename,
                          const char *kernel_cmdline,
                          const char *initrd_filename, const char *cpu_model)
{
    CPUState *env;
    int kernel_size;
    DriveInfo *dinfo;
    ram_addr_t phys_ram;
    ram_addr_t phys_flash;
    qemu_irq *cpu_irq, irq[32];
    DeviceState *dev;
    int i;

    /* memory map */
    ram_addr_t flash_base    = 0x04000000;
    size_t flash_sector_size = 256 * 1024;
    size_t flash_size        = 32 * 1024 * 1024;
    ram_addr_t ram_base      = 0x08000000;
    size_t ram_size          = 64 * 1024 * 1024;
    ram_addr_t timer0_base   = 0x80002000;
    ram_addr_t uart0_base    = 0x80006000;
    int uart0_irq            = 0;
    int timer0_irq           = 1;

    if (cpu_model == NULL) {
        cpu_model = "lm32-full";
    }
    env = cpu_init(cpu_model);

    env->eba = flash_base;
    qemu_register_reset(main_cpu_reset, env);

    phys_ram = qemu_ram_alloc(ram_size);
    cpu_register_physical_memory(ram_base, ram_size, phys_ram | IO_MEM_RAM);

    phys_flash = qemu_ram_alloc(flash_size);
    dinfo = drive_get(IF_PFLASH, 0, 0);
    /* should be Spansion S29NS128P */
    pflash_cfi02_register(flash_base, phys_flash,
                          dinfo ? dinfo->bdrv : NULL, flash_sector_size,
                          flash_size / flash_sector_size, 1, 4,
                          0x01, 0x7e, 0x43, 0x00, 0x555, 0x2aa);

    cpu_irq = lm32_pic_init_cpu(env);
    dev = sysbus_create_simple("lm32,pic", -1, *cpu_irq);
    /* XXX is there any better way to do this? */
    env->pic_handle = (struct lm32_pic*)dev;
    for (i = 0; i < 32; i++) {
        irq[i] = qdev_get_gpio_in(dev, i);
    }

    sysbus_create_simple("lm32,uart", uart0_base, irq[uart0_irq]);
    sysbus_create_simple("lm32,timer", timer0_base, irq[timer0_irq]);

    bootstrap_pc = flash_base;

    if (kernel_filename) {
        uint64_t entry, low, high;

        /* Boots a kernel elf binary.  */
        kernel_size = load_elf(kernel_filename, 0,
                               &entry, &low, &high,
                               1, ELF_MACHINE, 0);
        bootstrap_pc = entry;
    }
}

static void
lm32_uclinux_init(ram_addr_t ram_size_not_used,
                          const char *boot_device,
                          const char *kernel_filename,
                          const char *kernel_cmdline,
                          const char *initrd_filename, const char *cpu_model)
{
    CPUState *env;
    DriveInfo *dinfo;
    ram_addr_t phys_ram;
    ram_addr_t phys_flash;
    qemu_irq *cpu_irq, irq[32];
    DeviceState *dev;
    int i;

    /* memory map */
    ram_addr_t flash_base    = 0x04000000;
    size_t flash_sector_size = 256 * 1024;
    size_t flash_size        = 32 * 1024 * 1024;
    ram_addr_t ram_base      = 0x08000000;
    size_t ram_size          = 64 * 1024 * 1024;
    ram_addr_t uart0_base    = 0x80000000;
    ram_addr_t timer0_base   = 0x80002000;
    ram_addr_t timer1_base   = 0x80010000;
    ram_addr_t timer2_base   = 0x80012000;
    int uart0_irq            = 0;
    int timer0_irq           = 1;
    int timer1_irq           = 20;
    int timer2_irq           = 21;

    if (cpu_model == NULL) {
        cpu_model = "lm32-full";
    }
    env = cpu_init(cpu_model);

    env->eba = flash_base;
    qemu_register_reset(main_cpu_reset, env);

//    phys_bram = qemu_ram_alloc(bram_size);
//    cpu_register_physical_memory(0x04000000, bram_size,
//                                 phys_bram | IO_MEM_RAM);

    phys_ram = qemu_ram_alloc(ram_size);
    cpu_register_physical_memory(ram_base, ram_size, phys_ram | IO_MEM_RAM);

    phys_flash = qemu_ram_alloc(flash_size);
    dinfo = drive_get(IF_PFLASH, 0, 0);
    /* should be Spansion S29NS128P */
    pflash_cfi02_register(flash_base, phys_flash,
                          dinfo ? dinfo->bdrv : NULL, flash_sector_size,
                          flash_size / flash_sector_size, 1, 4,
                          0x01, 0x7e, 0x43, 0x00, 0x555, 0x2aa);

    cpu_irq = lm32_pic_init_cpu(env);
    dev = sysbus_create_simple("lm32,pic", -1, *cpu_irq);
    /* XXX is there any better way to do this? */
    env->pic_handle = (struct lm32_pic*)dev;
    for (i = 0; i < 32; i++) {
        irq[i] = qdev_get_gpio_in(dev, i);
    }

    sysbus_create_simple("lm32,uart", uart0_base, irq[uart0_irq]);
    sysbus_create_simple("lm32,timer", timer0_base, irq[timer0_irq]);
    sysbus_create_simple("lm32,timer", timer1_base, irq[timer1_irq]);
    sysbus_create_simple("lm32,timer", timer2_base, irq[timer2_irq]);

    bootstrap_pc = flash_base;
#if 0
    if (kernel_filename) {
        uint64_t entry, low, high;

        /* Boots a kernel elf binary.  */
        kernel_size = load_elf(kernel_filename, 0,
                               &entry, &low, &high,
                               1, ELF_MACHINE, 0);
        bootstrap_pc = entry;
    }
#endif

#if 0
    if (kernel_filename) {
        load_image_targphys(kernel_filename, FLASH_BASE,
                                          ram_size);
        bootstrap_pc = flash_base;
    }

    env->pc = bootstrap_pc;
#endif
}

static void
lm32_soc_init(ram_addr_t ram_size_not_used,
                          const char *boot_device,
                          const char *kernel_filename,
                          const char *kernel_cmdline,
                          const char *initrd_filename, const char *cpu_model)
{
    CPUState *env;
    ram_addr_t phys_ram;
    ram_addr_t phys_bram;
    qemu_irq *cpu_irq, irq[32];
    DeviceState *dev;
    int kernel_size;
    int i;

    /* memory map */
    ram_addr_t ram_base      = 0x40000000;
    size_t ram_size          = 16 * 1024 * 1024;
    ram_addr_t bram_base     = 0x00000000;
    size_t bram_size         = 8 * 1024;
    ram_addr_t uart0_base    = 0xf0000000;
    ram_addr_t timer0_base   = 0xf0010000;
    ram_addr_t timer1_base   = 0xf0030000;
    int uart0_irq            = 0;
    int timer0_irq           = 1;
    int timer1_irq           = 3;
    if (cpu_model == NULL) {
        cpu_model = "lm32-full";
    }
    env = cpu_init(cpu_model);

    env->eba = ram_base;
    qemu_register_reset(main_cpu_reset, env);

    phys_bram = qemu_ram_alloc(bram_size);
    cpu_register_physical_memory(bram_base, bram_size, phys_bram | IO_MEM_RAM);

    phys_ram = qemu_ram_alloc(ram_size);
    cpu_register_physical_memory(ram_base, ram_size, phys_ram | IO_MEM_RAM);

    cpu_irq = lm32_pic_init_cpu(env);
    dev = sysbus_create_simple("lm32,pic", -1, *cpu_irq);
    /* XXX is there any better way to do this? */
    env->pic_handle = (struct lm32_pic*)dev;
    for (i = 0; i < 32; i++) {
        irq[i] = qdev_get_gpio_in(dev, i);
    }

    sysbus_create_simple("lm32_soc,uart", uart0_base, irq[uart0_irq]);
    sysbus_create_simple("lm32_soc,timer", timer0_base, irq[timer0_irq]);
    sysbus_create_simple("lm32_soc,timer", timer1_base, irq[timer1_irq]);
    
    kernel_size = load_image_targphys(kernel_filename, ram_base,ram_size);
    printf("load_image_targphys: 0x%08x size=%i (%i kb) \n",(int)ram_base,kernel_size, kernel_size/     1024);
    
    bootstrap_pc = ram_base;
}

static QEMUMachine lm32_evr_machine = {
    .name = "lm32-evr",
    .desc = "lm32-evr desc TBD",
    .init = lm32_evr_init,
    .is_default = 1
};

static QEMUMachine lm32_uclinux_machine = {
    .name = "lm32-uclinux",
    .desc = "lm32 uclinux platform for uclinux and uboot by Theobroma Systems",
    .init = lm32_uclinux_init,
    .is_default = 1
};

static QEMUMachine lm32_soc_machine = {
    .name = "lm32-soc",
    .desc = "lm32 System on a chip",
    .init = lm32_soc_init,
    .is_default = 1
};

static void lm32_machine_init(void)
{
    qemu_register_machine(&lm32_uclinux_machine);
    qemu_register_machine(&lm32_evr_machine);
    qemu_register_machine(&lm32_soc_machine);
}

machine_init(lm32_machine_init);
