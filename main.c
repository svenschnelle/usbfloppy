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
#define FDC_CMD_READ		0x06
#define FDC_CMD_SENSE_INT	0x08
#define FDC_CMD_DUMPREG		0x0e
#define FDC_CMD_VERSION		0x10
#define FDC_CMD_CONFIGURE	0x13
#define FDC_CMD_READ_ID_MFM	0x4a
#define FDC_CMD_READ_ID_FM	0x0a
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
	int c;
	int h;
	int s;
} write_start;

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
		printf("%s: short read: %d out of %d bytes\n", __func__, n, len);
	return n;
}

static int fdc_command(const uint8_t *command, int len,
		       uint8_t *data, int dlen,
		       uint8_t *result, int rlen)
{
	int ret;

//	printf("%s: command %x\n", __func__, command[0]);
	ret = fdc_wait_cmd_ready(FDC_TIMEOUT);
	if (ret)
		return ret;

	for (int i = 0; i < len; i++) {
		ret = fdc_wait_data_write_ready(FDC_TIMEOUT);
		if (ret)
			return ret;
		fdc_write_reg(FDC_REG_FIFO, command[i]);
	}

	switch (command[0] & 0xf) {
	case FDC_CMD_READ:
		ret = fdc_data_read_phase(data, dlen);
		break;
	case FDC_CMD_WRITE:
		ret = fdc_data_write_phase(data, dlen);
		break;
	default:
		break;
	}

	switch (command[0]) {
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

	ret = fdc_result_phase(result, rlen);
	if (ret < 0)
		return ret;

	ret = fdc_wait_cmd_done();
	if (ret)
		return ret;
	return 0;
}

int fdc_sense_int(uint8_t *status)
{
	static const uint8_t cmd[] = { FDC_CMD_SENSE_INT };

	status[0] = 0;
	status[1] = 0;
	return fdc_command(cmd, sizeof(cmd), NULL, 0, status, 2);
}

static void print_fdc_st0(const char *prefix, uint8_t st0)
{
	static const char *ics[] = { "Normal Execution", "Abnormal Termination", "Invalid Command", "Abnormal Termination (Polling)" };

	if ((st0 >> 6) == 0)
		return;

	printf("%s: ST0 %x IC=%s %s%sHead %d DSEL %d\n", prefix, st0, ics[st0 >> 6],
	       st0 & 0x20 ? "Seek End " : "",
	       st0 & 0x10 ? "Equipment Check " : "",
	       (st0 >> 2) & 1,
	       st0 & 3);
}

static void print_fdc_status(const char *prefix, uint8_t st0, uint8_t st1, uint8_t st2)
{
	if ((st0 >> 6) == 0)
		return;

	print_fdc_st0(prefix, st0);
	printf("%s: ST1 %x %s %s %s %s %s %s\n", prefix, st1,
	       st1 & 0x80 ? "End of Cyclinder " : "",
	       st1 & 0x20 ? "Data Error " : "",
	       st1 & 0x10 ? "Over/Underrun " : "",
	       st1 & 0x04 ? "No Data " : "",
	       st1 & 0x02 ? "Not Writable " : "",
	       st1 & 0x01 ? "Missing Address Mark " : "");
	printf("%s: ST2 %x %s%s%s%s%s\n", prefix, st2,
	       st2 & 0x40 ? "Control Mark " : "",
	       st2 & 0x20 ? "Data Error in Data Field " : "",
	       st2 & 0x10 ? "Wrong Cylinder " : "",
	       st2 & 0x02 ? "Bad Cylinder " : "",
	       st2 & 0x01 ? "Missing DAM " : "");
}

void fdc_readid(void)
{
	static const int sector_sizes[8] = { 128, 256, 512, 1024, 2048, 4096, 8192, 16384 };
	static const uint8_t cmd[] = { FDC_CMD_READ_ID_MFM, 0 };
	uint8_t status[2];
	uint8_t regs[7];
	int ret;

	ret = fdc_command(cmd, sizeof(cmd), NULL, 0, regs, sizeof(regs));
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return;
	}

	ret = fdc_sense_int(status);
	if (ret) {
		printf("%s: fdc_sense_int: %d\n", __func__, ret);
		return;
	}

	print_fdc_status(__func__, regs[0], regs[1], regs[2]);

	printf("%s: Cyl %d Head %d Sector %d %d bytes/sector\n", __func__,
	       regs[3], regs[4], regs[5],
	       sector_sizes[regs[6] & 7]);
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
	uint8_t cmd[] = { FDC_CMD_SEEK, 0, track };
	uint8_t status[2];
	int ret;

	ret = fdc_command(cmd, sizeof(cmd), NULL, 0, NULL, 0);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return;
	}

	fdc_wait_interrupt();

	ret = fdc_sense_int(status);
	if (ret) {
		printf("%s: fdc_sense_int: %d\n", __func__, ret);
		return;
	}
	print_fdc_st0(__func__, status[0]);
//	printf("%s: pcn=%x\n", __func__, status[1]);
}

void fdc_relseek(int num)
{
	uint8_t cmd[] = { FDC_CMD_RELSEEK, 0, 0 };
	uint8_t status[2];
	int ret;

	if (num < 0) {
		cmd[0] |= 0x40;
		num = -num;
	}
	cmd[2] = num;

	ret = fdc_command(cmd, sizeof(cmd), NULL, 0, NULL, 0);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return;
	}

	ret = fdc_sense_int(status);
	if (ret) {
		printf("%s: fdc_sense_int: %d\n", __func__, ret);
		return;
	}
	printf("status=%x, pcn=%x\n", status[0], status[1]);
}

void fdc_recalibrate(void)
{
	static const uint8_t cmd[] = { FDC_CMD_RECALIBRATE, 0 };
	uint8_t status[2];
	int ret;

	ret = fdc_command(cmd, sizeof(cmd), NULL, 0, NULL, 0);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return;
	}

	ret = fdc_sense_int(status);
	if (ret) {
		printf("%s: fdc_sense_int: %d\n", __func__, ret);
		return;
	}
	print_fdc_st0(__func__, status[0]);
	printf("%s: pcn=%x\n", __func__, status[1]);
}

void fdc_specify(int srt, int hut, int hlt, int nd)
{
	uint8_t cmd[] = { FDC_CMD_SPECIFY, 0, 0 };
	uint8_t status[2];
	int ret;

	cmd[1] = ((srt & 0xf) << 4) | (hut & 0x0f);
	cmd[2] = (hlt << 1) | (nd & 1);

	ret = fdc_command(cmd, sizeof(cmd), NULL, 0, NULL, 0);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return;
	}

	ret = fdc_sense_int(status);
	if (ret) {
		printf("%s: fdc_sense_int: %d\n", __func__, ret);
		return;
	}
	print_fdc_st0(__func__, status[0]);
	printf("%s: pcn=%x\n", __func__, status[1]);
}

void fdc_configure(int eis, int efifo, int poll, int threshold, int pretrk)
{
	uint8_t cmd[] = { FDC_CMD_CONFIGURE, 0, 0, 0 };
	uint8_t status[2];
	int ret;

	if (eis)
		cmd[2] |= 0x40;
	if (efifo)
		cmd[2] |= 0x20;
	if (poll)
		cmd[2] |= 0x10;
	cmd[2] |= (threshold & 0xf);
	cmd[3] = pretrk;

	ret = fdc_command(cmd, sizeof(cmd), NULL, 0, NULL, 0);
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return;
	}

	if (fdc_int_pending()) {
		ret = fdc_sense_int(status);
		if (ret) {
			printf("%s: fdc_sense_int: %d\n", __func__, ret);
			return;
		}
		print_fdc_st0(__func__, status[0]);
		printf("%s: pcn=%x\n", __func__, status[1]);
	}
}

int fdc_read(struct chs *chs, int last_sector, uint8_t *out, int len)
{
	uint8_t cmd[9], result[7], status[2];
	int ret;

	cmd[0] = FDC_CMD_READ | 0x40 | 0x80;	// MFM + MT
	cmd[1] = ((chs->h & 1) << 2);		// DSH
	cmd[2] = chs->c;			// C
	cmd[3] = (chs->h & 1);			// H
	cmd[4] = chs->s;			// R
	cmd[5] = 2;				// N 512 bytes
	cmd[6] = last_sector;			// EOT
	cmd[7] = 0x2a;				// GPL
	cmd[8] = 0xff;				// DTL

	ret = fdc_command(cmd, sizeof(cmd), out, len, result, sizeof(result));
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return ret;
	}

	print_fdc_status(__func__, result[0], result[1], result[2]);
//	printf("%s: C %d H %d S %d N %d\n", __func__,
//	       result[3], result[4], result[5], result[6]);

	if (fdc_int_pending()) {
		ret = fdc_sense_int(status);
		if (ret) {
			printf("%s: fdc_sense_int: %d\n", __func__, ret);
			return -1;
		}
		print_fdc_st0(__func__, status[0]);
		printf("%s: pcn=%x\n", __func__, status[1]);
	}
	return 0;
}

int fdc_write(struct chs *chs, int last_sector, uint8_t *in, int len)
{
	uint8_t cmd[9], result[7], status[2];
	int ret;

	cmd[0] = FDC_CMD_WRITE | 0x40;		// MFM + MT
	cmd[1] = ((chs->h & 1) << 2);		// DSH
	cmd[2] = chs->c;			// C
	cmd[3] = (chs->h & 1);			// H
	cmd[4] = chs->s;			// R
	cmd[5] = 2;				// N 512 bytes
	cmd[6] = last_sector;			// EOT
	cmd[7] = 0x2a;				// GPL
	cmd[8] = 0xff;				// DTL

	ret = fdc_command(cmd, sizeof(cmd), in, len, result, sizeof(result));
	if (ret) {
		printf("%s: fdc_command: %d\n", __func__, ret);
		return ret;
	}

	print_fdc_status(__func__, result[0], result[1], result[2]);
	printf("%s: C %d H %d S %d N %d\n", __func__,
	       result[3], result[4], result[5], result[6]);

	if (fdc_int_pending()) {
		ret = fdc_sense_int(status);
		if (ret) {
			printf("%s: fdc_sense_int: %d\n", __func__, ret);
			return ret;
		}
		print_fdc_st0(__func__, status[0]);
		printf("%s: pcn=%x\n", __func__, status[1]);
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
	uint8_t status[2];
	fdc_sense_int(status);
	print_fdc_st0(__func__, status[0]);
	fdc_sense_int(status);
	print_fdc_st0(__func__, status[0]);
	fdc_sense_int(status);
	print_fdc_st0(__func__, status[0]);
	fdc_sense_int(status);
	print_fdc_st0(__func__, status[0]);
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
