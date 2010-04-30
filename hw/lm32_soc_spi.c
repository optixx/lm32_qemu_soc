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

#include <stdio.h>

#include "sysbus.h"
#include "qemu-char.h"

#define D(x) x

#define SPDR   0
#define SPCR   1 
#define SPCS   2
#define DUMMY  3 
#define SPDIV  4
#define R_MAX  5


#define spi_busy() s->regs[SPCR] |= 1
#define spi_ready() s->regs[SPCR] &= ~1


FILE *sdImage;

//SPI state machine states
enum{
    SPI_IDLE_STATE,
    SPI_ARG_X_LO,
    SPI_ARG_X_HI,
    SPI_ARG_Y_LO,
    SPI_ARG_Y_HI,
    SPI_ARG_CRC,
    SPI_RESPOND_SINGLE,
    SPI_RESPOND_MULTI,
    SPI_READ_SINGLE_BLOCK,
    SPI_READ_MULTIPLE_BLOCK,
    SPI_WRITE_SINGLE,
    SPI_WRITE_SINGLE_BLOCK,
};

struct SDPartitionEntry{
    uint8_t state;
    uint8_t startHead;
    uint16_t startCylinder;
    uint8_t type;
    uint8_t endHead;
    uint16_t endCylinder;
    uint32_t sectorOffset;
    uint32_t sectorCount;
};


struct lm32_soc_spi
{
    SysBusDevice busdev;
    uint32_t regs[R_MAX];
    uint8_t spi_response_buffer[12];
    uint8_t *spi_response_ptr;
    uint8_t *spi_response_end;
    uint8_t spi_byte;
    uint16_t spi_clock;
    uint16_t spi_state;
    uint8_t spi_command;
    uint8_t spi_transfer;
    uint32_t spi_bytecount;
    union{
        uint32_t spi_arg;
        union{
            struct{
                uint16_t spi_argy;
                uint16_t spi_argx;
            };
            struct{
                uint8_t spi_argy_lo;
                uint8_t spi_argy_hi;
                uint8_t spi_argx_lo;
                uint8_t spi_argx_hi;
            };
        };
    };
};


static int8_t SDLoadImage( uint8_t* filename){
    
    sdImage = fopen((char*)filename,"rb");
    if(!sdImage){
        D(printf("Cannot find SD image %s\n",filename));
        return 1;
    }
    return 0;
}

static int8_t SDReadByte(void)
{
    uint8_t result;
    if(!fread(&result,1,1,sdImage)){
        D(printf("Error reading SD image\n"));
        return -1;
    }
    return result;
}

static int8_t SDWriteByte(uint8_t value){    
    if(!fwrite(&value,1,1,sdImage)){
        D(printf("Error writing to SD image\n"));
        return -1;
    }
    return 0;
}

static int8_t SDSeekToOffset(uint32_t pos){
    fseek(sdImage,pos,SEEK_SET);
    return 0;
}

static int8_t SDCommit(void){
    return 0;
}

static int8_t ascii(uint8_t ch){
    if(ch >= 32 && ch <= 127){
        return ch;
    }
    return '.';
}



static void spi_update(void *opaque){
    struct lm32_soc_spi *s = opaque;
    D(printf("%s: byte: %02x\n",__func__,s->spi_byte));
    switch(s->spi_state){
    case SPI_IDLE_STATE:
        D(printf("%s: SPI_IDLE_STATE\n",__func__));
        if(s->spi_byte == 0xff){
            s->regs[SPDR] = 0xff; // echo back that we're ready
            spi_ready();
            break;
        }
        s->spi_command = s->spi_byte;
        s->regs[SPDR] = 0x00;
        spi_ready();
        s->spi_state = SPI_ARG_X_HI;
        break;
    case SPI_ARG_X_HI:
        D(printf("%s: x hi: %02X\n",__func__,s->spi_byte));
        s->spi_argx_hi = s->spi_byte;
        s->regs[SPDR] = 0x00;
        spi_ready();
        s->spi_state = SPI_ARG_X_LO;
        break;
    case SPI_ARG_X_LO:
        D(printf("%s: SPI_IDLE_STATE\n",__func__));
        D(printf("%s: x lo: %02X\n",__func__,s->spi_byte));
        s->spi_argx_lo = s->spi_byte;
        s->regs[SPDR] = 0x00;
        spi_ready();
        s->spi_state = SPI_ARG_Y_HI;
        break;
    case SPI_ARG_Y_HI:
        D(printf("%s: y hi: %02X\n",__func__,s->spi_byte));
        s->spi_argy_hi = s->spi_byte;
        s->regs[SPDR] = 0x00;
        spi_ready();
        s->spi_state = SPI_ARG_Y_LO;
        break;
    case SPI_ARG_Y_LO:
        D(printf("%s: y lo: %02X\n",__func__,s->spi_byte));
        s->spi_argy_lo = s->spi_byte;
        s->regs[SPDR] = 0x00;
        spi_ready();
        s->spi_state = SPI_ARG_CRC;
        break;
    case SPI_ARG_CRC:
        D(printf("%s: SPI - CMD%d (%02X) X:%04X Y:%04X CRC: %02X\n",__func__,s->spi_command^0x40,s->spi_command,
                s->spi_argx,s->spi_argy,s->spi_byte));
        // ignore CRC and process commands
        switch(s->spi_command){
        case 0x40: //CMD0 =  RESET / GO_IDLE_STATE
            D(printf("%s: CMD0 - Reset\n",__func__));
            s->regs[SPDR] = 0x00;
            spi_ready();
            s->spi_state = SPI_RESPOND_SINGLE;
            s->spi_response_buffer[0] = 0x00; // 8-clock wait
            s->spi_response_buffer[1] = 0x01; // no errors, going idle
            s->spi_response_ptr = s->spi_response_buffer;
            s->spi_response_end = s->spi_response_ptr+2;
            s->spi_bytecount = 0;
            break;
        case 0x41: //CMD1 =  INIT / SEND_OP_COND
            D(printf("%s: CMD1 - Init\n",__func__));
            s->regs[SPDR] = 0x00;
            spi_ready();
            s->spi_state = SPI_RESPOND_SINGLE;
            s->spi_response_buffer[0] = 0x00; // 8-clock wait
            s->spi_response_buffer[1] = 0x00; // no error
            s->spi_response_ptr = s->spi_response_buffer;
            s->spi_response_end = s->spi_response_ptr+2;
            s->spi_bytecount = 0;
            break;
        case 0x51: //CMD17 =  READ_BLOCK
            D(printf("%s: CMD17 - Read Block\n",__func__));
            s->regs[SPDR] = 0x00;
            spi_ready();
            s->spi_state = SPI_RESPOND_SINGLE;
            s->spi_response_buffer[0] = 0x00; // 8-clock wait
            s->spi_response_buffer[1] = 0x00; // no error
            s->spi_response_buffer[2] = 0xFE; // start block
            s->spi_response_ptr = s->spi_response_buffer;
            s->spi_response_end = s->spi_response_ptr+3;
            SDSeekToOffset(s->spi_arg);
            s->spi_bytecount = 512;
            break;
        case 0x52: //CMD18 =  MULTI_READ_BLOCK
            D(printf("%s: CMD18 - Multi Read\n",__func__));
            s->regs[SPDR] = 0x00;
            spi_ready();
            s->spi_state = SPI_RESPOND_MULTI;
            s->spi_response_buffer[0] = 0x00; // 8-clock wait
            s->spi_response_buffer[1] = 0x00; // no error
            s->spi_response_buffer[2] = 0xFE; // start block
            s->spi_response_ptr = s->spi_response_buffer;
            s->spi_response_end = s->spi_response_ptr+3;
            SDSeekToOffset(s->spi_arg);
            s->spi_bytecount = 512;
            break;   
        case 0x58: //CMD24 =  WRITE_BLOCK
            D(printf("%s: CMD24 - Write Read\n",__func__));
            s->regs[SPDR] = 0x00;
            spi_ready();
            s->spi_state = SPI_WRITE_SINGLE;
            s->spi_response_buffer[0] = 0x00; // 8-clock wait
            s->spi_response_buffer[1] = 0x00; // no error
            s->spi_response_buffer[2] = 0xFE; // start block
            s->spi_response_ptr = s->spi_response_buffer;
            s->spi_response_end = s->spi_response_ptr+3;
            SDSeekToOffset(s->spi_arg);
            s->spi_bytecount = 512;
            break;            
        default:
            D(printf("%s: default\n",__func__));
            s->regs[SPDR] = 0x00;
            spi_ready();
            s->spi_state = SPI_RESPOND_SINGLE;
            s->spi_response_buffer[0] = 0x02; // data accepted
            s->spi_response_buffer[1] = 0x05;  //i illegal command
            s->spi_response_ptr = s->spi_response_buffer;
            s->spi_response_end = s->spi_response_ptr+2;
            break;
        }
        break;
    case SPI_RESPOND_SINGLE:
        s->regs[SPDR] = *s->spi_response_ptr;
        spi_ready();
        D(printf("%s: SPI - Respond: %02X\n",__func__,s->regs[SPDR]));
        s->spi_response_ptr++;
        if(s->spi_response_ptr == s->spi_response_end){
            if(s->spi_bytecount != 0){
                s->spi_state = SPI_READ_SINGLE_BLOCK;
            }
            else{
                s->spi_state = SPI_IDLE_STATE;
            }
        }
        break;
    case SPI_READ_SINGLE_BLOCK:
        s->regs[SPDR] = SDReadByte();
        spi_ready();
	    {
            // output a nice display to see sector data
            int i = 512-s->spi_bytecount;
            int ofs = i&0x000F;
            int j;
            static unsigned char buf[16];
            if(i > 0 && (ofs == 0)){
                D(printf("%04X: ",i-16));
                for(j=0; j<16; j++){ 
                    D(printf("%02X ",buf[j]));
                }
                D(printf("| "));
                for(j=0; j<16; j++) {
                    D(printf("%c",ascii(buf[j])));
                }
                D(printf("\n"));
            }
            buf[ofs] = s->regs[SPDR];
	    }
        s->spi_bytecount--;
        if(s->spi_bytecount == 0){
            s->spi_response_buffer[0] = 0x00; //CRC
            s->spi_response_buffer[1] = 0x00; //CRC
            s->spi_response_ptr = s->spi_response_buffer;
            s->spi_response_end = s->spi_response_ptr+2;
            s->spi_state = SPI_RESPOND_SINGLE;
        }
        break;
    case SPI_RESPOND_MULTI:
        s->regs[SPDR] = *s->spi_response_ptr;
        spi_ready();
        D(printf("%s: SPI - Respond: %02X\n",__func__,s->regs[SPDR]));
        s->spi_response_ptr++;
        if(s->spi_response_ptr == s->spi_response_end){
            s->spi_state = SPI_READ_MULTIPLE_BLOCK;
        }
        break;  
    case SPI_READ_MULTIPLE_BLOCK:
        if(s->regs[SPDR] == 0x4C){ //CMD12
            s->regs[SPDR] = SDReadByte();
            spi_ready();
            memset(s->spi_response_buffer,0xFF,9); // Q&D - return garbage in response to the whole command
            s->spi_response_ptr = s->spi_response_buffer;
            s->spi_response_end = s->spi_response_ptr+9;
            s->spi_state = SPI_RESPOND_SINGLE;
            s->spi_bytecount = 0;
            break;
        }
        else{
            s->regs[SPDR] = SDReadByte();
            spi_ready();
        }
        D(printf("%s: SPI - Data[%d]: %02X\n",__func__,512-s->spi_bytecount,s->regs[SPDR]));
        s->spi_bytecount--;
        if(s->spi_bytecount == 0){
            s->spi_response_buffer[0] = 0x00; //CRC
            s->spi_response_buffer[1] = 0x00; //CRC
            s->spi_response_buffer[2] = 0xFE; // start block
            s->spi_response_ptr = s->spi_response_buffer;
            s->spi_response_end = s->spi_response_ptr+3;
            s->spi_arg+=512; // automatically move to next block
            SDSeekToOffset(s->spi_arg);
            s->spi_bytecount = 512;
            s->spi_state = SPI_RESPOND_MULTI;
        }
        break;
    case SPI_WRITE_SINGLE:
        s->regs[SPDR] = *s->spi_response_ptr;
        D(printf("%s: SPI - Respond: %02X\n",__func__,s->regs[SPDR]));
        s->spi_response_ptr++;
        if(s->spi_response_ptr == s->spi_response_end){
            if(s->spi_bytecount != 0){
                s->spi_state = SPI_WRITE_SINGLE_BLOCK;
            }
            else{
                s->spi_state = SPI_IDLE_STATE;
            }
        }
        break;    
    case SPI_WRITE_SINGLE_BLOCK:
        SDWriteByte(s->regs[SPDR]);
        spi_ready();
        D(printf("%s: SPI - Data[%d]: %02X\n",__func__,s->spi_bytecount,s->regs[SPDR]));
        s->regs[SPDR] = 0xFF;
        s->spi_bytecount--;
        if(s->spi_bytecount == 0){
            s->spi_response_buffer[0] = 0x00; //CRC
            s->spi_response_buffer[1] = 0x00; //CRC
            s->spi_response_ptr = s->spi_response_buffer;
            s->spi_response_end = s->spi_response_ptr+2;
            s->spi_state = SPI_RESPOND_SINGLE;
            SDCommit();
        }
        break;    
    }    
}


static uint32_t spi_read(void *opaque, target_phys_addr_t addr)
{
    //D(printf("%s: addr=%08x\n", __func__,addr));
    struct lm32_soc_spi *s = opaque;
    uint32_t r = 0;
    addr >>= 2;
    switch (addr)
    {
        case SPCR:
            r = s->regs[SPCR];
            D(printf("%s:  <- SPCR=%08x\n", __func__,r));
            break;
        case SPDR:
            r = s->regs[SPDR];
            D(printf("%s:  <- SPDR=%08x\n", __func__,r));
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
    //D(printf("%s: addr=%08x value=%08x\n", __func__,addr, value));
    struct lm32_soc_spi *s = opaque;
    addr >>= 2;
    switch (addr)
    {
        case SPCR:
            s->regs[SPCR] = value;
            D(printf("%s: -> SPCR=%08x\n", __func__,value));
            break;
        case SPDR:
            s->regs[SPDR] = value;
            D(printf("%s: -> SPDR=%08x\n", __func__,value));
            if (value & 0x40){
                s->spi_clock = 64 * 8; // spiclockdivider * 8
                s->spi_transfer = 1;
                s->spi_byte = value;
                spi_busy();
                spi_update(opaque);
            }
            break;
        case SPCS:
            s->regs[SPCS] = value;
            D(printf("%s: -> SPCS=%08x\n", __func__,value));
            break;
        case SPDIV:
            s->regs[SPDIV] = value;
            D(printf("%s: -> SPDIV=%08x\n", __func__,value));
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
    s->spi_byte = 0;
    s->spi_clock = 0;
    s->spi_transfer = 0;
    s->spi_state = SPI_IDLE_STATE;
    SDLoadImage((uint8_t*)"sdcard.img");
    return 0;
}

static void lm32_soc_spi_register(void)
{
    sysbus_register_dev("lm32_soc,spi", sizeof (struct lm32_soc_spi),
                        lm32_soc_spi_init);
}

device_init(lm32_soc_spi_register)
