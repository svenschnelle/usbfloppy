#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "generic.h"
#include "device.h"
#include <mass_mal.h>
#include "hw_config.h"
#include "usb_core.h"
#include "usb_init.h"
#include "usb_pwr.h"
#include <string.h>
#include "terminal.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x_usart.h"
#include <errno.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdarg.h>

// Port B
#define PIN_CS		GPIO_Pin_0
#define PIN_TIME	GPIO_Pin_5
#define PIN_TX1		GPIO_Pin_6
#define PIN_DB0		GPIO_Pin_8
#define PIN_DB1		GPIO_Pin_9
#define PIN_DB2		GPIO_Pin_10
#define PIN_DB3		GPIO_Pin_11
#define PIN_DB4		GPIO_Pin_12
#define PIN_DB5		GPIO_Pin_13
#define PIN_DB6		GPIO_Pin_14
#define PIN_DB7		GPIO_Pin_15

// Port A
#define PIN_A0		GPIO_Pin_0
#define PIN_A1		GPIO_Pin_1
#define PIN_A2		GPIO_Pin_2
#define PIN_WR		GPIO_Pin_3
#define PIN_RD		GPIO_Pin_4
#define PIN_DACK	GPIO_Pin_5
#define PIN_TC		GPIO_Pin_6
#define PIN_RESET	GPIO_Pin_7
#define PIN_DRQ		GPIO_Pin_8
#define PIN_INT		GPIO_Pin_9

#define PIN_OUT		PIN_A0|PIN_A1|PIN_A2|PIN_WR|PIN_RD|PIN_DACK|PIN_TC|PIN_RESET
#define PIN_IN		PIN_DRQ|PIN_INT
#define PIN_DBX		PIN_DB0|PIN_DB1|PIN_DB2|PIN_DB3|PIN_DB4|PIN_DB5|PIN_DB6|PIN_DB7

#define FDC_REG_SRA	0x00
#define FDC_REG_SRB	0x01
#define FDC_REG_DOR	0x02
#define FDC_REG_MSR	0x04
#define FDC_REG_DSR	0x04
#define FDC_REG_FIFO	0x05
#define FDC_REG_DIR	0x07
#define FDC_REG_CCR	0x07

#define FDC_MSR_RQM		0x80
#define FDC_MSR_DIO		0x40
#define FDC_MSR_NONDMA		0x20
#define FDC_MSR_CMD_BSY		0x10
#define FDC_MSR_DRV3_BSY	0x08
#define FDC_MSR_DRV2_BSY	0x04
#define FDC_MSR_DRV1_BSY	0x02
#define FDC_MSR_DRV0_BSY	0x01

#define FDC_CMD_SPECIFY		0x03
#define FDC_CMD_WRITE		0x05
#define FDC_CMD_READ_DATA	0x06
#define FDC_CMD_SENSE_INT	0x08
#define FDC_CMD_DUMPREG		0x0e
#define FDC_CMD_VERSION		0x10
#define FDC_CMD_CONFIGURE	0x13
#define FDC_CMD_READ_ID		0x0a
#define FDC_CMD_SEEK		0x0f
#define FDC_CMD_RECALIBRATE	0x07
#define FDC_CMD_RELSEEK		0x8f

#define FDC_TIMEOUT 2000

#define FDC_DATA_RATE_MFM_500KBPS	0
#define FDC_DATA_RATE_MFM_300KBPS	1
#define FDC_DATA_RATE_MFM_250KBPS	2
#define FDC_DATA_RATE_MFM_1MBPS		3

#define FDC_DATA_RATE_FM_250KBPS	0
#define FDC_DATA_RATE_FM_150KBPS	1
#define FDC_DATA_RATE_FM_125KBPS	2

#define FDC_DATA_RATE_300KBPS	0
#define FDC_DATA_RATE_250KBPS	0

#define MOTOR_TIMEOUT 30

static int motor_timer;
static int floppy_dsel;

uint32_t Mass_Memory_Size[2];
uint32_t Mass_Block_Size[2];
uint32_t Mass_Block_Count[2];

struct chs {
	uint8_t c;
	uint8_t h;
	uint8_t s;
} write_start;

struct fdc_st0 {
	uint8_t ds:2;
	uint8_t h:1;
	uint8_t :1;
	uint8_t ec:1;
	uint8_t se:1;
	uint8_t ic:2;
};

struct fdc_st1 {
	uint8_t ma:1;
	uint8_t nw:1;
	uint8_t nd:1;
	uint8_t :1;
	uint8_t or:1;
	uint8_t de:1;
	uint8_t :1;
	uint8_t en:1;
};

struct fdc_st2 {
	uint8_t md:1;
	uint8_t bc:1;
	uint8_t :2;
	uint8_t wc:1;
	uint8_t dd:1;
	uint8_t cm:1;
	uint8_t :1;
};

struct fdc_st3 {
	uint8_t ds:2;
	uint8_t hd:1;
	uint8_t :1;
	uint8_t t0:1;
	uint8_t :1;
	uint8_t wp:1;
	uint8_t :1;
};

struct fdc_st {
	struct fdc_st0 st0;
	struct fdc_st1 st1;
	struct fdc_st2 st2;
};

struct fdc_read_data_cmd {
	uint8_t cmd:6;
	uint8_t mfm:1;
	uint8_t mt:1;

	uint8_t ds:2;
	uint8_t hds:1;
	uint8_t :5;

	struct chs chs;
	uint8_t n;
	uint8_t eot;
	uint8_t gpl;
	uint8_t dtl;
};

struct fdc_read_data_result {
	struct fdc_st st;
	struct chs chs;
	uint8_t n;
};

struct fdc_write_data_cmd {
	uint8_t cmd:6;
	uint8_t mfm:1;
	uint8_t mt:1;

	uint8_t ds:2;
	uint8_t hds:1;
	uint8_t :5;

	struct chs chs;
	uint8_t n;
	uint8_t eot;
	uint8_t gpl;
	uint8_t dtl;
};

struct fdc_sense_int_cmd {
	uint8_t cmd;
};

struct fdc_sense_int_result {
	struct fdc_st0 st0;
	uint8_t pcn;
};

struct fdc_write_data_result {
	struct fdc_st st;
	struct chs chs;
	uint8_t n;
};

struct fdc_readid_cmd {
	uint8_t cmd:6;
	uint8_t mfm:1;
	uint8_t :1;

	uint8_t ds:2;
	uint8_t hds:1;
	uint8_t :5;
};

struct fdc_readid_result {
	struct fdc_st st;
	struct chs chs;
	uint8_t n;
};

struct fdc_seek_cmd {
	uint8_t cmd;

	uint8_t	ds:2;
	uint8_t	hds:1;
	uint8_t :5;

	uint8_t	ncn;
};

struct fdc_recalibrate_cmd {
	uint8_t cmd;

	uint8_t ds:2;
	uint8_t :6;
};

struct fdc_specify_cmd {
	uint8_t cmd;

	uint8_t hut:4;
	uint8_t srt:4;

	uint8_t nd:1;
	uint8_t hlt:7;
};

struct fdc_configure_cmd {
	uint8_t cmd;
	uint8_t null;

	uint8_t fifothr:4;
	uint8_t poll:1;
	uint8_t efifo:1;
	uint8_t eis:1;
	uint8_t :1;
	uint8_t pretrk;
};

struct fdc_format_track_cmd {
	uint8_t cmd:6;
	uint8_t mfm:1;
	uint8_t :1;

	uint8_t ds:2;
	uint8_t hds:1;
	uint8_t :5;

	uint8_t d;
	struct chs chs;
	uint8_t n;
};

struct fdc_format_track_result {
	struct fdc_st st;
	uint8_t undefined[4];
};

struct fdc_sense_drive_status_cmd {
	uint8_t cmd;
	uint8_t ds:2;
	uint8_t hds:1;
};

struct fdc_sense_drive_status_result {
	struct fdc_st3 st3;
};

struct fdc_ctx {
	union {
		struct fdc_read_data_cmd read_data;
		struct fdc_write_data_cmd write_data;
		struct fdc_sense_int_cmd sense_int;
		struct fdc_readid_cmd readid;
		struct fdc_seek_cmd seek;
		struct fdc_recalibrate_cmd recalibrate;
		struct fdc_specify_cmd specify;
		struct fdc_configure_cmd configure;
		struct fdc_format_track_cmd format_track;
		struct fdc_sense_drive_status_cmd sense_drive_status;
	} cmd;
	union {
		struct fdc_read_data_result read_data;
		struct fdc_write_data_result write_data;
		struct fdc_sense_int_result sense_int;
		struct fdc_readid_result readid;
		struct fdc_format_track_result format_track;
		struct fdc_sense_drive_status_result sense_drive_status;
	} result;
	uint8_t	*data;
	int	cmdlen;
	int	datalen;
	int	resultlen;

};

struct gpio_config_data {
	GPIO_TypeDef *gpio;
	GPIO_InitTypeDef config;
};

static struct gpio_config_data gpio_config[] = {
	{ GPIOC, { PIN_LED, GPIO_Speed_50MHz, GPIO_Mode_Out_PP }},
	{ GPIOB, { PIN_DBX|PIN_CS|PIN_TIME, GPIO_Speed_50MHz, GPIO_Mode_Out_PP }},
	{ GPIOA, { PIN_OUT, GPIO_Speed_50MHz, GPIO_Mode_Out_PP }},
	{ GPIOA, { PIN_IN,  GPIO_Speed_50MHz, GPIO_Mode_IPU }},
	{ GPIOB, { PIN_TX1, GPIO_Speed_50MHz, GPIO_Mode_AF_PP }},
};

#define SECTORS_PER_TRACK 15

static uint8_t track_cache[SECTORS_PER_TRACK * 2 * 512];
static int current_track = -1;

size_t strlen(const char *s)
{
	size_t ret = 0;
	while (*s++)
		ret++;
	return ret;
}

void *memset(void *s, int c, size_t n)
{
	void *ret = s;

	while (n--)
		*(char *)s++ = c;
	return ret;
}

void *memcpy(void *d, const void *s, size_t n)
{
	void *ret = d;

	while (n--)
		*(char *)d++ = *(char *)s++;
	return ret;
}

void *memmove(void *d, const void *s, size_t n)
{
	memcpy(d, s, n); // FIXME
	return d;
}

int memcmp(const void *s1, const void *s2, size_t n)
{
	while (n--) {
		char c1 = *(char *)s1++;
		char c2 = *(char *)s2++;
		if (c1 != c2)
			return c1 - c2;
	}
	return 0;
}

void *memchr(const void *s, int c, size_t n)
{
	while (n--) {
		if (*(char *)s++ == (char)c)
			return (void *)s - 1;
	}
	return NULL;
}

int putchar(int c)
{
	if (c == '\n')
		putchar('\r');
	while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
	USART_SendData(USART1, c);
	return (char)c;
}

int puts(const char *s)
{
	int n = 0;

	while (*s) {
		putchar(*s++);
		n++;
	}
	return n;
}

static void printdigit(unsigned int digit)
{
	digit &= 0xf;

	if (digit > 9)
		putchar('a' + digit - 10);
	else
		putchar('0' + digit);
}

static void printhex(unsigned int num)
{
	int i;

	for (i = 32; i > 0; i -= 4) {
		printdigit(num >> (i - 4));
	}
}

static void printdec(int num)
{
	char numstr[16] = { 0 };
	int i = 14, sign = 0;

	if (!num) {
		puts("0");
		return;
	}
	if (num < 0) {
		num = -num;
		sign = 1;
	}

	while (num && i >= 0) {
		int digit = num % 10;
		numstr[i--] = '0' + digit;
		num /= 10;
	}

	if (sign && i > 0)
		numstr[i--] = '-';
	puts(numstr + i + 1);
}

static void printdec_u(unsigned int num)
{
	char numstr[16] = { 0 };
	int i = 14;

	if (!num) {
		puts("0");
		return;
	}

	while (num && i >= 0) {
		int digit = num % 10;
		numstr[i--] = '0' + digit;
		num /= 10;
	}
	puts(numstr + i + 1);
}

int printf(const char *fmt, ...)
{
	va_list ap;
	char c;

	va_start(ap, fmt);

	while (*fmt) {
		c = *fmt++;
		if (c != '%') {
			putchar(c);
			continue;
		}

		switch (*fmt++) {
		case '%':
			putchar('%');
			break;
		case 's':
			puts(va_arg(ap, const char *));
			break;
		case 'p':
		case 'x':
			printhex(va_arg(ap, unsigned int));
			break;
		case 'd':
			printdec(va_arg(ap, int));
			break;
		case 'u':
			printdec_u(va_arg(ap, unsigned int));
			break;
		default:
			break;
		}
	}
	va_end(ap);
	return 0;// FIXME
}

static void prnibble(uint8_t nibble)
{
	if (nibble > 9)
		putchar(0x61 + nibble - 10);
	else
		putchar(0x30 + nibble);
}

static void prbyte(uint8_t byte)
{
	prnibble(byte >> 4);
	prnibble(byte & 0xf);
}

void hexdump(char *prefix, void *buf, size_t len)
{
	for (unsigned int i = 0; i < len; i++) {
		if (!(i & 0xf))
			printf("\n%s %x: ", prefix, i);
		prbyte(((uint8_t *)buf)[i]);
		putchar(' ');
	}
	puts("\n");
}


char *strchr(const char *s, int c)
{
	while (*s) {
		if (*s == c)
			return (char *)s;
		s++;
	}
	return NULL;
}

static void gpio_init(void)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(gpio_config); i++) {
		struct gpio_config_data *config = &gpio_config[i];
		GPIO_Init(config->gpio, &config->config);
	}
	GPIO_SetBits(GPIOA, PIN_DACK|PIN_RD|PIN_WR|PIN_CS);
	GPIO_ResetBits(GPIOA, PIN_TC|PIN_A0|PIN_A1|PIN_A2);
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
}


static RCC_ClocksTypeDef RCC_Clocks;

static void DWT_Delay_Init()
{
	RCC_GetClocksFreq(&RCC_Clocks);
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	DWT->CYCCNT = 0;
}

static void DWT_Delay_us(uint32_t delay)
{
	uint32_t initial_ticks = DWT->CYCCNT;

	delay *= (RCC_Clocks.HCLK_Frequency / 1000000);
	while (DWT->CYCCNT - initial_ticks < delay);
}

void DWT_Delay_ms(uint32_t delay)
{
	uint32_t initial_ticks = DWT->CYCCNT;
	delay *= (RCC_Clocks.HCLK_Frequency / 1000);
	while (DWT->CYCCNT - initial_ticks < delay);
}

static uint32_t DWT_ms_to_ticks(uint32_t ms)
{
	return ms * (RCC_Clocks.HCLK_Frequency / 1000);
}

static uint32_t DWT_GetTick(void)
{
	return DWT->CYCCNT;
}

static int DWT_TimeAfter(uint32_t start, uint32_t ticks)
{
	return (DWT->CYCCNT - start) > ticks;
}

static void fdc_reset(void)
{
	GPIO_SetBits(GPIOA, PIN_RESET);
	DWT_Delay_ms(10);
	GPIO_ResetBits(GPIOA, PIN_RESET);
	DWT_Delay_ms(10);
}

static void fdc_data_mode_in(void)
{
	GPIOB->CRH = 0x88888888;
	GPIOB->BSRR = 0xff00;
}

static void fdc_data_mode_out(void)
{
	GPIOB->CRH = 0x33333333;
}

static void fdc_write_reg(int reg, uint8_t data)
{
	GPIOA->BSRR = PIN_RD | PIN_WR | PIN_CS;
	fdc_data_mode_out();
	GPIOA->BRR = (~reg & 7);
	GPIOA->BSRR = reg & 7;
	GPIOB->BRR = 0xff00;
	GPIOB->BSRR = data << 8;
	DWT_Delay_us(1);
	GPIOB->BRR = PIN_CS;
	DWT_Delay_us(1);
	GPIOA->BRR = PIN_WR;
	DWT_Delay_us(1);
	GPIOA->BSRR = PIN_WR;
	DWT_Delay_us(1);
	GPIOB->BSRR = PIN_CS;
	DWT_Delay_us(1);
}

static uint8_t fdc_read_reg(int reg)
{
	uint8_t data;

	GPIOA->BSRR = PIN_RD | PIN_WR | PIN_CS;
	GPIOA->BRR = (~reg) & 7;
	GPIOA->BSRR = reg & 7;
	GPIOB->BRR = PIN_CS;
	fdc_data_mode_in();
	GPIOA->BRR = PIN_RD;
	DWT_Delay_us(1);
	data = GPIOB->IDR >> 8;
	GPIOA->BSRR = PIN_RD;
	GPIOB->BSRR = PIN_CS;
	return data;
}

static void fdc_print_msr(const char *prefix, uint8_t msr)
{
#if 0
	printf("%s: MSR %x %s%s%s%s%s%s%s%s\n", prefix, msr,
	       msr & FDC_MSR_RQM ? "RQM " : "",
	       msr & FDC_MSR_DIO ? "DIO " : "",
	       msr & FDC_MSR_NONDMA ? "NONDMA " : "",
	       msr & FDC_MSR_CMD_BSY ? "CMD " : "",
	       msr & FDC_MSR_DRV3_BSY ? "DRV3 " : "",
	       msr & FDC_MSR_DRV2_BSY ? "DRV2 " : "",
	       msr & FDC_MSR_DRV1_BSY ? "DRV1 " : "",
	       msr & FDC_MSR_DRV0_BSY ? "DRV0 " : "");
#endif
}

static int fdc_int_pending(void)
{
	return GPIOA->IDR & PIN_INT;
}

static int fdc_wait_interrupt(void)
{
	uint32_t start = DWT_GetTick();
	int timeout = 0;
	uint8_t msr;

	while (!fdc_int_pending()) {
		if (DWT_TimeAfter(start, DWT_ms_to_ticks(500))) {
			printf("%s: timeout\n", __func__);
			timeout = -1;
			break;
		}
	}

	msr = fdc_read_reg(FDC_REG_MSR);
	fdc_print_msr(__func__, msr);
	return timeout;
}

static int fdc_wait_cmd_ready(int tmo)
{
	uint32_t start = DWT_GetTick();
	uint8_t msr;

	msr = fdc_read_reg(FDC_REG_MSR);
	if (msr & FDC_MSR_CMD_BSY) {
		fdc_print_msr(__func__, msr);
		return -3;
	}

	for (;;) {
		msr = fdc_read_reg(FDC_REG_MSR);
		if ((msr & (FDC_MSR_RQM|FDC_MSR_DIO)) == FDC_MSR_RQM)
			return 0;
		if (DWT_TimeAfter(start, DWT_ms_to_ticks(FDC_TIMEOUT))) {
			printf("%s: timeout\n", __func__);
			break;
		}
	}
	fdc_print_msr(__func__, msr);
	return -1;
}

static int fdc_wait_cmd_done(void)
{
	uint32_t start = DWT_GetTick();
	uint8_t msr;

	for (;;) {
		msr = fdc_read_reg(FDC_REG_MSR);
		if (!(msr & (FDC_MSR_CMD_BSY)))
			return 0;
		if (DWT_TimeAfter(start, DWT_ms_to_ticks(FDC_TIMEOUT))) {
			printf("%s: timeout\n", __func__);
			break;
		}
	}
	fdc_print_msr(__func__, msr);
	return -1;
}
#if 0
static int fdc_wait_data_read_ready(int tmo)
{
	int i = tmo;
	uint8_t msr;

	while (i--) {
		msr = fdc_read_reg(FDC_REG_MSR);
		if ((msr & (FDC_MSR_RQM|FDC_MSR_DIO)) == (FDC_MSR_RQM|FDC_MSR_DIO))
			return 0;
		DWT_Delay_us(10);
	}
	fdc_print_msr(__func__, msr);
	return -1;
}
#endif

static int fdc_wait_data_write_ready(int tmo)
{
	uint32_t start = DWT_GetTick();
	uint8_t msr;

	for (;;) {
		msr = fdc_read_reg(FDC_REG_MSR);
		if ((msr & (FDC_MSR_RQM|FDC_MSR_DIO)) == FDC_MSR_RQM)
			return 0;
		if (DWT_TimeAfter(start, DWT_ms_to_ticks(FDC_TIMEOUT))) {
			printf("%s: timeout\n", __func__);
			break;
		}
	}
	fdc_print_msr(__func__, msr);
	return -1;
}

static int fdc_result_phase(uint8_t *out, int len)
{
	uint32_t start = DWT_GetTick();
	uint8_t msr, c;
	int n = 0;

	for (;;) {
		msr = fdc_read_reg(FDC_REG_MSR);
		if (DWT_TimeAfter(start, DWT_ms_to_ticks(FDC_TIMEOUT))) {
			printf("%s: timeout\n", __func__);
			fdc_print_msr(__func__, msr);
			return -1;
		}
		if (!(msr & FDC_MSR_RQM))
			continue;
		if ((msr & (FDC_MSR_RQM|FDC_MSR_DIO|FDC_MSR_CMD_BSY|FDC_MSR_NONDMA)) == FDC_MSR_RQM)
			break;
		if (n < len) {
			c = fdc_read_reg(FDC_REG_FIFO);
			out[n] = c;
//			printf("%s: FIFO %d %x\n", __func__, n, c);
		}
		n++;
	}
//	if (n < len)
//		printf("%s: short read: %d out of %d bytes\n", __func__, n, len);
	return n;
}

static int fdc_data_read_phase(uint8_t *out, int len)
{
	uint32_t start = DWT_GetTick();
	uint16_t idr;
	uint8_t msr;
	int n = 0;

	fdc_data_mode_in();
	GPIOB->BRR = PIN_CS;
	do {
		if (DWT_TimeAfter(start, DWT_ms_to_ticks(2000))) {
			msr = fdc_read_reg(FDC_REG_MSR);
			printf("%s: timeout\n", __func__);
			fdc_print_msr(__func__, msr);
			return -1;
		}
		idr = GPIOA->IDR;
		if (idr & PIN_INT)
			break;
		if (!(idr & PIN_DRQ))
			continue;

		GPIOB->BRR = PIN_TIME;
		GPIOA->BRR = PIN_DACK;
		GPIOA->BRR = PIN_RD;
		DWT_Delay_us(1);
		out[n++] = GPIOB->IDR >> 8;
		if (n >= len)
			GPIOA->BSRR = PIN_TC;
		GPIOA->BSRR = PIN_RD;
		GPIOA->BSRR = PIN_DACK;
		GPIOB->BSRR = PIN_TIME;
	} while(n < len);

	GPIOA->BRR = PIN_TC;
	GPIOB->BSRR = PIN_CS;
	if (n < len)
		printf("%s: short read: %d out of %d bytes\n", __func__, n, len);
	return n;
}

static int fdc_data_write_phase(uint8_t *in, int len)
{
	uint32_t start = DWT_GetTick();
	uint16_t idr;
	uint8_t msr;
	int n = 0;

	fdc_data_mode_out();
	GPIOB->BRR = PIN_CS;
	do {
		if (DWT_TimeAfter(start, DWT_ms_to_ticks(2000))) {
			msr = fdc_read_reg(FDC_REG_MSR);
			printf("%s: timeout\n", __func__);
			fdc_print_msr(__func__, msr);
			return -1;
		}
		idr = GPIOA->IDR;
		if (idr & PIN_INT)
			break;
		if (!(idr & PIN_DRQ))
			continue;

		GPIOB->BRR = PIN_TIME;
		GPIOA->BRR = PIN_DACK;
		uint16_t c = in[n++] << 8;
		GPIOB->BRR = 0xff00;
		GPIOB->BSRR = c;
		GPIOA->BRR = PIN_WR;
//		DWT_Delay_us(1);
		if (n >= len)
			GPIOA->BSRR = PIN_TC;
		GPIOA->BSRR = PIN_WR;
		GPIOA->BSRR = PIN_DACK;
		GPIOB->BSRR = PIN_TIME;
	} while(n < len);

	GPIOA->BRR = PIN_TC;
	GPIOB->BSRR = PIN_CS;
	if (n < len)
		printf("%s: short write: %d out of %d bytes\n", __func__, n, len);
	return n;
}

static int fdc_command(struct fdc_ctx *ctx)
{
	int ret;

//	printf("%s: command %x\n", __func__, command[0]);
	ret = fdc_wait_cmd_ready(FDC_TIMEOUT);
	if (ret)
		return ret;

	for (int i = 0; i < ctx->cmdlen; i++) {
		ret = fdc_wait_data_write_ready(FDC_TIMEOUT);
		if (ret)
			return ret;
		fdc_write_reg(FDC_REG_FIFO, ((uint8_t *)&ctx->cmd)[i]);
	}

	switch (((uint8_t *)&ctx->cmd)[0] & 0xf) {
	case FDC_CMD_READ_DATA:
		ret = fdc_data_read_phase(ctx->data, ctx->datalen);
		break;
	case FDC_CMD_WRITE:
		ret = fdc_data_write_phase(ctx->data, ctx->datalen);
		break;
	default:
		break;
	}

	switch (((uint8_t *)&ctx->cmd)[0] & 0xf) {
	case FDC_CMD_SPECIFY:
	case FDC_CMD_SENSE_INT:
	case FDC_CMD_CONFIGURE:
		break;
	default:
		ret = fdc_wait_interrupt();
		if (ret)
			return ret;
		break;
	}

	ret = fdc_result_phase((uint8_t *)&ctx->result, ctx->resultlen);
	if (ret < 0)
		return ret;

	ret = fdc_wait_cmd_done();
	if (ret)
		return ret;
	return 0;
}

int fdc_sense_int(struct fdc_sense_int_result *result)
{
	struct fdc_ctx ctx = {
		.cmdlen = sizeof(struct fdc_sense_int_cmd),
		.resultlen = sizeof(struct fdc_sense_int_result),
		.cmd.sense_int.cmd = FDC_CMD_SENSE_INT
	};
	int ret;

	ret = fdc_command(&ctx);
	if (ret)
		return ret;
	*result = ctx.result.sense_int;
	return 0;
}

static void print_fdc_st0(const char *prefix, struct fdc_st0 *st0)
{
	static const char *ics[] = { "Normal Execution", "Abnormal Termination", "Invalid Command", "Abnormal Termination (Polling)" };

	if (st0->ic == 0)
		return;

	printf("%s: ST0 IC=%s %s%sHead %d DSEL %d\n", prefix, ics[st0->ic],
	       st0->se ? "Seek End " : "",
	       st0->ec ? "Equipment Check " : "",
	       st0->h,
	       st0->ds);
}

static void print_fdc_status(const char *prefix, struct fdc_st *st)
{
	if (st->st0.ic == 0)
		return;

	print_fdc_st0(prefix, &st->st0);
	printf("%s: ST1 %s %s %s %s %s %s\n", prefix,
	       st->st1.en ? "End of Cyclinder " : "",
	       st->st1.de ? "Data Error " : "",
	       st->st1.or ? "Over/Underrun " : "",
	       st->st1.nd ? "No Data " : "",
	       st->st1.nw ? "Not Writable " : "",
	       st->st1.ma ? "Missing Address Mark " : "");
	printf("%s: ST2 %s%s%s%s%s\n", prefix,
	       st->st2.cm ? "Control Mark " : "",
	       st->st2.dd ? "Data Error in Data Field " : "",
	       st->st2.wc ? "Wrong Cylinder " : "",
	       st->st2.bc ? "Bad Cylinder " : "",
	       st->st2.md ? "Missing DAM " : "");
}

void fdc_readid(void)
{
	static const int sector_sizes[8] = { 128, 256, 512, 1024, 2048, 4096, 8192, 16384 };
	struct fdc_ctx ctx = {
		.cmdlen = sizeof(struct fdc_readid_cmd),
		.resultlen = sizeof(struct fdc_readid_result),
		.cmd.readid.cmd = FDC_CMD_READ_ID,
		.cmd.readid.mfm = 1
	};
	struct fdc_sense_int_result status;
	int ret;

	ret = fdc_command(&ctx);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return;
	}

	ret = fdc_sense_int(&status);
	if (ret) {
		printf("%s: fdc_sense_int: %d\n", __func__, ret);
		return;
	}

	print_fdc_status(__func__, &ctx.result.readid.st);

	printf("%s: Cyl %d Head %d Sector %d %d bytes/sector\n", __func__,
	       ctx.result.readid.chs.c,
	       ctx.result.readid.chs.h,
	       ctx.result.readid.chs.s,
	       sector_sizes[ctx.result.readid.n & 7]);
}

void fdc_dsel(int dsel)
{
	static const uint8_t values[] = { 0x0c, 0x1c, 0x2d, 0x4e, 0x8f };
	if (dsel)
		motor_timer = MOTOR_TIMEOUT;
	if (floppy_dsel == dsel)
		return;
	floppy_dsel = dsel;

	fdc_write_reg(FDC_REG_DOR, values[dsel]);
	if (dsel)
		DWT_Delay_ms(500);
}

void fdc_seek(int track)
{
	struct fdc_ctx ctx = {
		.cmdlen = sizeof(struct fdc_seek_cmd),
		.cmd.seek.cmd = FDC_CMD_SEEK,
		.cmd.seek.ncn = track
	};
	struct fdc_sense_int_result status;
	int ret;

	ret = fdc_command(&ctx);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return;
	}

	fdc_wait_interrupt();

	ret = fdc_sense_int(&status);
	if (ret) {
		printf("%s: fdc_sense_int: %d\n", __func__, ret);
		return;
	}
	print_fdc_st0(__func__, &status.st0);
//	printf("%s: pcn=%x\n", __func__, status.pcn);
}


void fdc_recalibrate(void)
{
	struct fdc_ctx ctx = {
		.cmdlen = sizeof(struct fdc_recalibrate_cmd),
		.cmd.recalibrate.cmd = FDC_CMD_RECALIBRATE
	};
	struct fdc_sense_int_result status;
	int ret;

	ret = fdc_command(&ctx);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return;
	}

	ret = fdc_sense_int(&status);
	if (ret) {
		printf("%s: fdc_sense_int: %d\n", __func__, ret);
		return;
	}
	print_fdc_st0(__func__, &status.st0);
	printf("%s: pcn=%x\n", __func__, status.pcn);
}

void fdc_specify(int srt, int hut, int hlt, int nd)
{
	struct fdc_ctx ctx = {
		.cmdlen = sizeof(struct fdc_specify_cmd),
		.cmd.specify.cmd = FDC_CMD_SPECIFY,
		.cmd.specify.srt = srt,
		.cmd.specify.hut = hut,
		.cmd.specify.hlt = hlt,
		.cmd.specify.nd = nd
	};
	struct fdc_sense_int_result status;
	int ret;

	ret = fdc_command(&ctx);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return;
	}

	ret = fdc_sense_int(&status);
	if (ret) {
		printf("%s: fdc_sense_int: %d\n", __func__, ret);
		return;
	}
	print_fdc_st0(__func__, &status.st0);
	printf("%s: pcn=%x\n", __func__, status.pcn);
}

void fdc_configure(int eis, int efifo, int poll, int threshold, int pretrk)
{
	struct fdc_ctx ctx = {
		.cmdlen = sizeof(struct fdc_configure_cmd),
		.cmd.configure.cmd = FDC_CMD_CONFIGURE,
		.cmd.configure.eis = eis,
		.cmd.configure.efifo = efifo,
		.cmd.configure.poll = poll,
		.cmd.configure.fifothr = threshold,
		.cmd.configure.pretrk = pretrk
	};
	struct fdc_sense_int_result status;
	int ret;

	ret = fdc_command(&ctx);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return;
	}

	if (fdc_int_pending()) {
		ret = fdc_sense_int(&status);
		if (ret) {
			printf("%s: fdc_sense_int: %d\n", __func__, ret);
			return;
		}
		print_fdc_st0(__func__, &status.st0);
		printf("%s: pcn=%x\n", __func__, status.pcn);
	}
}

int fdc_read(struct chs *chs, int last_sector, uint8_t *out, int len)
{
	struct fdc_ctx ctx = { .cmd.read_data.cmd = FDC_CMD_READ_DATA,
		.cmdlen = sizeof(struct fdc_read_data_cmd),
		.resultlen = sizeof(struct fdc_read_data_result),
		.data = out,
		.datalen = len,
		.cmd.read_data.mt = 1,
		.cmd.read_data.mfm = 1,
		.cmd.read_data.hds = chs->h,
		.cmd.read_data.chs = *chs,
		.cmd.read_data.n = 2,
		.cmd.read_data.eot = last_sector,
		.cmd.read_data.gpl = 0x2a,
		.cmd.read_data.dtl = 0xff
	};
	struct fdc_sense_int_result status;
	int ret;

	ret = fdc_command(&ctx);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return ret;
	}

	print_fdc_status(__func__, &ctx.result.read_data.st);
//	printf("%s: C %d H %d S %d N %d\n", __func__,
//	       ctx.result.read_data.chs.c,
//	       ctx.result.read_data.chs.h,
//	       ctx.result.read_data.chs.s,
//	       ctx.result.read_data.n);

	if (fdc_int_pending()) {
		ret = fdc_sense_int(&status);
		if (ret) {
			printf("%s: fdc_sense_int: %d\n", __func__, ret);
			return -1;
		}
		print_fdc_st0(__func__, &status.st0);
		printf("%s: pcn=%x\n", __func__, status.pcn);
	}
	return 0;
}

int fdc_write(struct chs *chs, int last_sector, uint8_t *in, int len)
{
	struct fdc_ctx ctx = {
		.cmdlen = sizeof(struct fdc_write_data_cmd),
		.resultlen = sizeof(struct fdc_write_data_result),
		.data = in,
		.datalen = len,
		.cmd.write_data.cmd = FDC_CMD_WRITE,
		.cmd.write_data.mfm = 1,
		.cmd.write_data.hds = chs->h,
		.cmd.write_data.chs = *chs,
		.cmd.write_data.n = 2,
		.cmd.write_data.eot = last_sector,
		.cmd.write_data.gpl = 0x2a,
		.cmd.write_data.dtl = 0xff
	};
	struct fdc_sense_int_result status;
	int ret;

	ret = fdc_command(&ctx);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return ret;
	}

	print_fdc_status(__func__, &ctx.result.write_data.st);
//	printf("%s: C %d H %d S %d N %d\n", __func__,
//	       ctx.result.write_data.chs.c,
//	       ctx.result.write_data.chs.h,
//	       ctx.result.write_data.chs.s,
//	       ctx.result.write_data.n);

	if (fdc_int_pending()) {
		ret = fdc_sense_int(&status);
		if (ret) {
			printf("%s: fdc_sense_int: %d\n", __func__, ret);
			return ret;
		}
		print_fdc_st0(__func__, &status.st0);
		printf("%s: pcn=%x\n", __func__, status.pcn);
	}
	return 0;
}

static void uart_init(void)
{
	USART_InitTypeDef init;
	USART_ClockInitTypeDef usart_clock_init;

	USART_ClockStructInit(&usart_clock_init);
	USART_ClockInit(USART1, &usart_clock_init);

	USART_StructInit(&init);
	init.USART_BaudRate = 115200;
	USART_Init(USART1, &init);
	USART_Cmd(USART1, ENABLE);
}

int main()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | \
			       RCC_APB2Periph_GPIOB | \
			       RCC_APB2Periph_GPIOC | \
			       RCC_APB2Periph_AFIO | \
			       RCC_APB2Periph_USART1, ENABLE);

	SystemInit();
	Set_System();
	Set_USBClock();
	DWT_Delay_Init();

	gpio_init();
	uart_init();

	fdc_reset();
	fdc_write_reg(FDC_REG_DOR, 0x0c);
	fdc_write_reg(FDC_REG_DSR, FDC_DATA_RATE_MFM_500KBPS);

	DWT_Delay_ms(1);

	puts("\n\n**** Boot done ****\n");
	struct fdc_sense_int_result status;
	fdc_sense_int(&status);
	print_fdc_st0(__func__, &status.st0);
	fdc_sense_int(&status);
	print_fdc_st0(__func__, &status.st0);
	fdc_sense_int(&status);
	print_fdc_st0(__func__, &status.st0);
	fdc_sense_int(&status);
	print_fdc_st0(__func__, &status.st0);
	fdc_dsel(2);
	fdc_configure(1, 0, 0, 15, 0);
//	fdc_specify(0, 0, 0, 0);
	fdc_recalibrate();

	motor_timer = MOTOR_TIMEOUT;

	USB_Interrupts_Config(ENABLE);
	USB_Init();

	while (1) {
//		GPIO_ToggleBits(GPIOC, PIN_LED);
		DWT_Delay_ms(100);

		__disable_irq();
		if (motor_timer > 0) {
			if (--motor_timer == 0) {
				fdc_dsel(0);
				motor_timer = -1;
			}
		}
		__enable_irq();
	}
}

uint16_t MAL_Init(uint8_t lun)
{

	if (lun)
		return MAL_FAIL;

	for (int i = 0; i < 2; i++) {
		Mass_Block_Count[i] = 15 * 2 * 80;
		Mass_Block_Size[i] = 512;
		Mass_Memory_Size[i] = Mass_Block_Size[i] * Mass_Block_Count[i];
	}
	return MAL_OK;
}

uint16_t MAL_GetStatus (uint8_t lun)
{
	if (lun)
		return MAL_FAIL;
	return MAL_OK;
}

static void lba_to_chs(int lba, struct chs *chs)
{
	chs->h = (lba % (SECTORS_PER_TRACK * 2)) / SECTORS_PER_TRACK;
	chs->c = lba / (SECTORS_PER_TRACK * 2);
	chs->s = lba % SECTORS_PER_TRACK + 1;
}

uint16_t MAL_StartWrite(uint8_t lun, int start_lba, int num_sectors)
{
	lba_to_chs(start_lba, &write_start);
	printf("%s: o %x, C %d, H %d, S %d\n", __func__, start_lba, \
	       write_start.c, write_start.h, write_start.s);
	fdc_dsel(2);
	return 0;
}

int MAL_Write(uint8_t lun, uint32_t offset, uint32_t *buf, uint16_t _len, int last)
{
	struct chs chs;
	int ret, len;

	lba_to_chs(offset / 512, &chs);

	current_track = -1;
	printf("%s: o %x, C %d, H %d, S %d, Len %d\n", __func__, (int)offset,
	       chs.c, chs.h, chs.s, _len);
	fdc_dsel(2);

	memcpy(&track_cache[offset % (512 * 15)], buf, _len);

	if (chs.s == SECTORS_PER_TRACK || last) {
		printf("%s: writing C %d H %d S %d - %d%s\n", __func__,
		       write_start.c, write_start.h, write_start.s, chs.s, last ? " LAST" : "");

		len = (chs.s - write_start.s + 1) * 512;
		ret = fdc_write(&write_start, chs.s, track_cache + ((write_start.s - 1) * 512), len);
		lba_to_chs((offset / 512) + 1, &write_start);
		return ret;
	}
	return 0;
}

int MAL_Read(uint8_t lun, uint32_t offset, uint32_t *buf, uint16_t len)
{
	struct chs chs;
	int ret;

	lba_to_chs(offset / 512, &chs);
//	printf("%s: o %x, C %d, H %d, S %d, Len %d\n", __func__, (int)offset, c, h, s, len);

	fdc_dsel(2);
	if (current_track != chs.c) {
		chs.s = 1;
		chs.h = 0;
		ret = fdc_read(&chs, SECTORS_PER_TRACK, track_cache, sizeof(track_cache));
		if (ret)
			return ret;
		current_track = chs.c;
	}

	memcpy(buf, &track_cache[offset % (512 * 15 * 2)], len);
	return 0;
}
