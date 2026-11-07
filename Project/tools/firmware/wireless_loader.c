/* Resident recovery loader. Uses the existing NXP ROM Flash API wrappers.
 * Download to staging, validate, commit manifest, then copy to application.
 * A reset during copying resumes the copy from staging on the next boot.
 * This does not provide rollback from a valid but malfunctioning application.
 */
/* Adapted from SU7Ultra wireless_bootloader.inc, GPL-3.0-or-later.
 * GCC port: early reset dispatch, polling transport, no live-peripheral jump. */
#include "include.h"
#include "SEEKFREE_WIRELESS.h"
#include "zf_rom_api.h"
#include "fsl_cache.h"
#include "wireless_boot_flag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define SU7_APP_OFFSET 0x20000U
#define SU7_STAGE_OFFSET 0x200000U
#define SU7_META_OFFSET 0x3FF000U
#define SU7_IMAGE_LIMIT (SU7_STAGE_OFFSET - SU7_APP_OFFSET)
#define SU7_MAGIC 0x37555053U
#define SU7_ENTER_MAGIC 0x3755424CU
#define SU7_VECTOR_OFFSET 0x2000U

static void su7_reply(const char *s) {
    uart_putbuff(WIRELESS_UART, (uint8 *)s, strlen(s));
}
/* pyOCD clears DEMCR on disconnect. Re-enable tracing even if it disconnects
 * while a receive timeout is already in progress. CYCCNT alone is not enough. */
static uint32_t su7_cycles(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return DWT->CYCCNT;
}
/* Busy polling avoids dropping the next byte in a 1 ms idle delay. The host
 * sends only one packet at a time and waits for a verified Flash ACK. */
static int su7_line(char *line, uint32_t capacity, uint32_t timeout_ms) {
    uint32_t used=0, last=su7_cycles();
    uint8_t byte;
    int overflow=0;
    while ((uint32_t)(su7_cycles()-last) < (SystemCoreClock/1000U)*timeout_ms) {
        if (!uart_query(WIRELESS_UART,&byte)) continue;
        last=su7_cycles();
        if(byte=='\r') continue;
        if(byte=='\n') {
            line[used]=0;
            if(overflow) return -1;
            if(used) return 1;
            continue;
        }
        if(used+1<capacity) line[used++]=(char)byte; else overflow=1;
    }
    line[0]=0;
    return 0;
}
/* These helpers execute from resident Flash BEFORE .text/.data are copied.
 * No globals, libc calls, or peripheral initialization are permitted here. */
__attribute__((section(".init_text"),noinline))
static uint32_t su7_crc(const uint8_t *data, uint32_t size) {
    uint32_t crc=0xFFFFFFFFU;
    while(size--) {
        crc ^= *data++;
        for(uint32_t b=0;b<8;b++) crc=(crc>>1)^(0xEDB88320U & (0U-(crc&1U)));
    }
    return ~crc;
}
__attribute__((naked,noreturn,section(".init_text")))
static void early_jump(uint32_t address) {
    __asm volatile("ldr r1,[r0]\nldr r2,[r0,#4]\nmsr msp,r1\nbx r2\n");
}
__attribute__((section(".init_text"),noinline))
void su7_boot_dispatch(void) {
    const uint32_t *m=(const uint32_t *)0x703FF000U;
    const uint8_t *app=(const uint8_t *)0x70020000U;
    const uint32_t *v=(const uint32_t *)0x70022000U;
    if(SNVS->LPGPR[0]==SU7_ENTER_MAGIC) return;
    if(m[0]!=SU7_MAGIC || m[1]!=2U || m[2]<=0x2008U || m[2]>SU7_IMAGE_LIMIT) return;
    if(m[4]!=su7_crc((const uint8_t *)m,16) || m[3]!=su7_crc(app,m[2])) return;
    if(v[0]<0x20000000U || v[0]>0x20070000U || (v[0]&7U) || !(v[1]&1U) ||
       v[1]<0x70022000U || v[1]>=0x70020000U+m[2]) return;
    early_jump(0x70022000U);
}


extern flexspi_nor_config_t config;
#define flash_instance instance
extern uint32 instance;
typedef struct {
    uint32_t magic, version, size, crc, header_crc;
} su7_manifest;

static uint32_t page_buffer[64] __attribute__((aligned(32)));
static char command[600];
static uint32_t incoming_size, incoming_crc, received;
static int session_open;
static int flash_ready;

static const uint8_t *flash_at(uint32_t offset)
{
    return (const uint8_t *)(FLASH_BASE_ADDR + offset);
}

static void flash_refresh(uint32_t offset, uint32_t size)
{
    rom_api_flexspi_clear_cache(flash_instance);
    DCACHE_InvalidateByRange(FLASH_BASE_ADDR + offset, (size + 31U) & ~31U);
    __DSB();
}

static int erase_sector(uint32_t offset)
{
    uint32_t mask = DisableGlobalIRQ();
    status_t result = rom_api_flexspi_nor_flash_erase(flash_instance, &config, offset, 4096U);
    EnableGlobalIRQ(mask);
    flash_refresh(offset, 4096U);
    return result == kStatus_Success;
}

static int program_page(uint32_t offset)
{
    uint32_t mask = DisableGlobalIRQ();
    status_t result = rom_api_flexspi_nor_flash_page_program(flash_instance, &config, offset, page_buffer);
    EnableGlobalIRQ(mask);
    flash_refresh(offset, 256U);
    return result == kStatus_Success && !memcmp(flash_at(offset), page_buffer, 256U);
}

static int manifest_valid(su7_manifest *manifest)
{
    memcpy(manifest, flash_at(SU7_META_OFFSET), sizeof(*manifest));
    return manifest->magic == SU7_MAGIC && manifest->version == 2 &&
        manifest->size > SU7_VECTOR_OFFSET + 8 && manifest->size <= SU7_IMAGE_LIMIT &&
        manifest->header_crc == su7_crc((const uint8_t *)manifest, 16);
}

static int vectors_valid(uint32_t offset)
{
    const uint32_t *vectors = (const uint32_t *)flash_at(offset + SU7_VECTOR_OFFSET);
    uint32_t stack = vectors[0], entry = vectors[1];
    return stack >= 0x20000000U && stack <= 0x20070000U && !(stack & 7U) &&
        (entry & 1U) && entry >= FLASH_BASE_ADDR + SU7_APP_OFFSET + SU7_VECTOR_OFFSET &&
        entry < FLASH_BASE_ADDR + SU7_APP_OFFSET + SU7_IMAGE_LIMIT;
}

static int image_valid(uint32_t offset, const su7_manifest *manifest)
{
    return vectors_valid(offset) && su7_crc(flash_at(offset), manifest->size) == manifest->crc;
}

static int install_image(const su7_manifest *manifest)
{
    if (!image_valid(SU7_STAGE_OFFSET, manifest)) return 0;
    for (uint32_t pos = 0; pos < manifest->size; pos += 256) {
        if (!(pos & 4095U) && !erase_sector(SU7_APP_OFFSET + pos)) return 0;
        memset(page_buffer, 0xFF, sizeof(page_buffer));
        uint32_t count = manifest->size - pos;
        if (count > 256) count = 256;
        memcpy(page_buffer, flash_at(SU7_STAGE_OFFSET + pos), count);
        if (!program_page(SU7_APP_OFFSET + pos)) return 0;
    }
    return image_valid(SU7_APP_OFFSET, manifest);
}

/* Return through ROM reset rather than jumping with UART/clocks/cache live. */
static void run_application(void) {
    su7_manifest manifest;
    if(!manifest_valid(&manifest) || !image_valid(SU7_APP_OFFSET,&manifest)) {
        su7_reply("ERR NO_APP\r\n"); return;
    }
    su7_reply("OK RUN\r\n");
    systick_delay_ms(100);
    SNVS->LPGPR[0]=0;
    __DSB();
    NVIC_SystemReset();
}

static void acknowledge(void)
{
    char reply[40];
    snprintf(reply, sizeof(reply), "OK DATA %lu\r\n", (unsigned long)received);
    su7_reply(reply);
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* YMODEM-1K/CRC: receive a complete frame in RAM before touching Flash.
 * The existing BEGIN/COMMIT envelope retains whole-image CRC32 and recovery.
 * TEST uses exactly the same receiver but never erases or programs Flash. */
static uint8_t ym_packet[1028];
extern LPUART_Type *UARTN[];
static void ym_put(uint8_t value) { uart_putchar(WIRELESS_UART, value); }
static int ym_get(uint8_t *value, uint32_t timeout_ms) {
    uint32_t start = su7_cycles();
    while ((uint32_t)(su7_cycles()-start) < (SystemCoreClock/1000U)*timeout_ms) {
        uint32_t errors = UARTN[WIRELESS_UART]->STAT &
            (LPUART_STAT_OR_MASK | LPUART_STAT_FE_MASK | LPUART_STAT_NF_MASK);
        if (errors) { UARTN[WIRELESS_UART]->STAT = errors; return 0; }
        if (uart_query(WIRELESS_UART,value)) return 1;
    }
    return 0;
}
static uint16_t ym_crc16(const uint8_t *data, uint32_t count) {
    uint16_t crc=0;
    while(count--) {
        crc ^= (uint16_t)*data++ << 8;
        for(unsigned b=0;b<8;b++) crc=(crc & 0x8000U) ? (crc<<1)^0x1021U : crc<<1;
    }
    return crc;
}
static int ym_receive(int test_only) {
    uint8_t expected=0, lead;
    uint16_t previous_crc=0;
    unsigned errors=0;
    int ending=0, saw_eot=0;
    received=0;
    ym_put('C');
    while(errors<10) {
        if(!ym_get(&lead,1000)) {
            errors++; ym_put(expected==0 || ending ? 'C' : 0x15); continue;
        }
        if(lead==0x18) goto failed;
        if(lead==0x04) {
            if(received!=incoming_size) goto failed;
            if(!saw_eot) { saw_eot=1; ym_put(0x15); }
            else { ending=1; ym_put(0x06); ym_put('C'); }
            continue;
        }
        uint32_t length=lead==0x01 ? 128U : lead==0x02 ? 1024U : 0U;
        int valid=length!=0;
        if(valid) {
            for(uint32_t i=0;i<length+4;i++) {
                if(!ym_get(ym_packet+i,250)) { valid=0; break; }
            }
        }
        uint16_t checksum=valid ? ((uint16_t)ym_packet[length+2]<<8)|ym_packet[length+3] : 0;
        if(!valid || (uint8_t)(ym_packet[0]^ym_packet[1])!=0xFF ||
           checksum!=ym_crc16(ym_packet+2,length)) {
            /* Drain a damaged/truncated frame before requesting its retry. */
            while(ym_get(&lead,50)) {}
            errors++; ym_put(0x15); continue;
        }
        uint8_t number=ym_packet[0];
        uint8_t *data=ym_packet+2;
        if(ending) {
            if(number!=0 || data[0]!=0) goto failed;
            ym_put(0x06);
            if(test_only) session_open=0;
            char reply[64];
            snprintf(reply,sizeof(reply),"OK YMODEM %lu\r\n",(unsigned long)received);
            su7_reply(reply);
            return 1;
        }
        if(expected==0 && received==0) {
            if(number!=0 || !data[0]) goto failed;
            uint8_t *zero=memchr(data,0,length);
            if(!zero || zero+1>=data+length) goto failed;
            /* Size must match BEGIN; bound parsing to this frame. */
            data[length-1]=0;
            char *end;
            unsigned long size=strtoul((char *)zero+1,&end,10);
            if(end==(char *)zero+1 || (*end && *end!=' ') || size!=incoming_size) goto failed;
            expected=1; previous_crc=checksum; errors=0;
            ym_put(0x06); ym_put('C'); continue;
        }
        if(number==(uint8_t)(expected-1) && checksum==previous_crc) {
            ym_put(0x06);
            if(received==0) ym_put('C');
            continue;
        }
        if(number!=expected || received>=incoming_size || length!=1024) { errors++; ym_put(0x15); continue; }
        uint32_t count=incoming_size-received;
        if(count>length) count=length;
        if(!test_only) {
            for(uint32_t pos=0;pos<count;pos+=256) {
                uint32_t offset=received+pos;
                uint32_t n=count-pos; if(n>256) n=256;
                memset(page_buffer,0xFF,sizeof(page_buffer));
                memcpy(page_buffer,data+pos,n);
                if((!(offset&4095U) && !erase_sector(SU7_STAGE_OFFSET+offset)) ||
                   !program_page(SU7_STAGE_OFFSET+offset)) goto failed;
            }
        }
        received+=count; expected++; previous_crc=checksum; errors=0;
        ym_put(0x06);
    }
failed:
    session_open=0;
    ym_put(0x18); ym_put(0x18);
    su7_reply("ERR YMODEM\r\n");
    return 0;
}

static void handle_command(void)
{
    if (!strcmp(command, "HELLO") || !strcmp(command, "ENTER")) {
        su7_reply("SU7 BOOT 2 UART8 115200\r\n");
    } else if (!strcmp(command, "CAPS")) {
        su7_reply("OK CAPS YMODEM-1K TEST\r\n");
    } else if (!strncmp(command, "YMODEM TEST ", 12)) {
        unsigned long size; char extra;
        if(sscanf(command+12,"%lu %c",&size,&extra)!=1 || !size || size>SU7_IMAGE_LIMIT) {
            su7_reply("ERR LENGTH\r\n"); return;
        }
        session_open=0; incoming_size=size;
        ym_receive(1);
    } else if (!strcmp(command, "YMODEM")) {
        if(!session_open) { su7_reply("ERR SESSION\r\n"); return; }
        ym_receive(0);
    } else if (!strcmp(command, "RUN")) {
        session_open = 0;
        run_application();
    } else if (!strncmp(command, "BEGIN ", 6)) {
        unsigned long size, crc;
        char extra;
        if (!flash_ready || sscanf(command + 6, "%lu %lx %c", &size, &crc, &extra) != 2 ||
                size <= SU7_VECTOR_OFFSET + 8 || size > SU7_IMAGE_LIMIT) {
            su7_reply("ERR BEGIN\r\n"); return;
        }
        /* Same-session retry is idempotent. A new session starts at zero. */
        if (!session_open || incoming_size != size || incoming_crc != crc) received = 0;
        incoming_size = (uint32_t)size; incoming_crc = (uint32_t)crc;
        session_open = 1;
        char reply[40];
        snprintf(reply, sizeof(reply), "OK BEGIN %lu\r\n", (unsigned long)received);
        su7_reply(reply);
    } else if (!strncmp(command, "DATA ", 5)) {
        char *end;
        uint32_t offset = (uint32_t)strtoul(command + 5, &end, 10);
        if (!session_open || end == command + 5 || *end++ != ' ') {
            su7_reply("ERR SESSION\r\n"); return;
        }
        char *hex = end, *separator = strchr(hex, ' ');
        if (!separator) { su7_reply("ERR PACKET\r\n"); return; }
        uint32_t digits = (uint32_t)(separator - hex);
        if (!digits || digits > 512 || (digits & 1U)) { su7_reply("ERR LENGTH\r\n"); return; }
        uint32_t count = digits / 2;
        if ((offset & 255U) || offset > incoming_size || count > incoming_size - offset ||
                (count != 256 && offset + count != incoming_size)) {
            su7_reply("ERR RANGE\r\n"); return;
        }
        memset(page_buffer, 0xFF, sizeof(page_buffer));
        uint8_t *data = (uint8_t *)page_buffer;
        for (uint32_t i = 0; i < count; i++) {
            int high = hex_digit(hex[i * 2]), low = hex_digit(hex[i * 2 + 1]);
            if (high < 0 || low < 0) { su7_reply("ERR HEX\r\n"); return; }
            data[i] = (uint8_t)((high << 4) | low);
        }
        char *crc_start = separator + 1;
        uint32_t crc = (uint32_t)strtoul(crc_start, &end, 16);
        if (end == crc_start || *end || crc != su7_crc(data, count)) {
            su7_reply("ERR CRC\r\n"); return;
        }
        if (offset < received && offset + count <= received &&
                !memcmp(flash_at(SU7_STAGE_OFFSET + offset), data, count)) {
            acknowledge(); return;
        }
        if (offset != received) { su7_reply("ERR OFFSET\r\n"); return; }
        if ((!(offset & 4095U) && !erase_sector(SU7_STAGE_OFFSET + offset)) ||
                !program_page(SU7_STAGE_OFFSET + offset)) {
            session_open = 0; su7_reply("ERR FLASH\r\n"); return;
        }
        received += count;
        acknowledge();
    } else if (!strcmp(command, "COMMIT")) {
        su7_manifest manifest = { SU7_MAGIC, 2, incoming_size, incoming_crc, 0 };
        if (!session_open || received != incoming_size || !image_valid(SU7_STAGE_OFFSET, &manifest)) {
            /* Allow retries after a previous successful COMMIT response was lost. */
            if (manifest_valid(&manifest) && image_valid(SU7_APP_OFFSET, &manifest) &&
                    manifest.size == incoming_size && manifest.crc == incoming_crc)
                su7_reply("OK COMMIT\r\n");
            else su7_reply("ERR IMAGE\r\n");
            return;
        }
        manifest.header_crc = su7_crc((const uint8_t *)&manifest, 16);
        memset(page_buffer, 0xFF, sizeof(page_buffer));
        memcpy(page_buffer, &manifest, sizeof(manifest));
        if (!erase_sector(SU7_META_OFFSET) || !program_page(SU7_META_OFFSET)) {
            session_open = 0; su7_reply("ERR META\r\n"); return;
        }
        session_open = 0;
        su7_reply("OK INSTALLING\r\n");
        su7_reply(install_image(&manifest) ? "OK COMMIT\r\n" : "ERR INSTALL\r\n");
    } else {
        su7_reply("ERR COMMAND\r\n");
    }
}

void main(void) {
    /* The resident loader is also the reset image. Dispatch to a committed
     * application before board/UART setup; otherwise RUN resets back into
     * this loader forever because the vector hook is never reached. */
    /* Application dispatch runs from SystemInit before SDRAM/cache setup. */
    su7_boot_flag_init();
    int forced=SNVS->LPGPR[0]==SU7_ENTER_MAGIC;
    SNVS->LPGPR[0]=0;
    board_init();
    gpio_init(RTS_PIN,GPI,0,GPIO_PIN_CONFIG);
    uart_init(WIRELESS_UART,WIRELESS_UART_BAUD,WIRELESS_UART_TX,WIRELESS_UART_RX);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    EnableGlobalIRQ(0);
    flash_ready=!flash_init();
    su7_reply(flash_ready ? "SU7 BOOT 2 READY\r\n" : "ERR FLASH_INIT\r\n");
    su7_manifest manifest;
    if(flash_ready && manifest_valid(&manifest)) {
        if(!image_valid(SU7_APP_OFFSET,&manifest)) {
            su7_reply("SU7 RECOVER\r\n");
            if(!install_image(&manifest)) forced=1;
        }
        if(!forced && image_valid(SU7_APP_OFFSET,&manifest)) run_application();
    }
    for(;;) {
        int result=su7_line(command,sizeof(command),2000);
        if(result==1) handle_command();
        else if(result<0) su7_reply("ERR LINE\r\n");
    }
}
