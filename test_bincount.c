#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>
#include <stdio.h>
#include <errno.h>

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOA, GPIOB, GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART2);

	/* Enable SPI1 Periph and gpio clocks */
	rcc_periph_clock_enable(RCC_SPI1);
}

static void spi_setup(void) {

  /* Configure GPIOs: SS=PA3, SCK=PA5, MISO=PA6 and MOSI=PA7 */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO7 );

  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
          GPIO6);

  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
  spi_reset(SPI1);

  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/8 of peripheral clock frequency
   * Clock polarity: Idle Low
   * Clock phase: Data valid on 1st clock pulse
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
  spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_disable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);

  /* Enable SPI1 periph. */
  spi_enable(SPI1);
}


static void gpio_setup(void)
{
	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO3);
}

#define MCP_IODIRA 0x00
#define MCP_IODIRB 0x01
#define MCP_IOCONA 0x0A
#define MCP_GPPUA  0x0C
#define MCP_GPPUB  0x0D
#define MCP_GPIOA  0x12
#define MCP_GPIOB  0x13

static void mcp_write(uint8_t addr, uint8_t data) {
	gpio_clear(GPIOA, GPIO3);
	spi_xfer(SPI1, 0x40);
	spi_xfer(SPI1, addr);
	spi_xfer(SPI1, data);
	gpio_set(GPIOA, GPIO3);
}

static void mcp_setup(void) {
	mcp_write(MCP_IOCONA, 0x28); // I/O Control Register: BANK=0, SEQOP=1, HAEN=1 (Enable Addressing)
	mcp_write(MCP_IODIRA, 0x00); // GPIOA as output
	mcp_write(MCP_IODIRB, 0xFF); // GPIOB as input
	mcp_write(MCP_GPPUB,  0xFF); // Enable Pull-up Resistor on GPIOB
	mcp_write(MCP_GPIOA,  0x00); // Reset Output on GPIOA
}

int main(void)
{
	clock_setup();
	gpio_setup();
	gpio_set(GPIOA, GPIO3);
	gpio_set(GPIOC, GPIO13);
	spi_setup();
	mcp_setup();

	/* Binary Counter Test (on GPIOA) */
	uint8_t counter=0;
	while (1) {
		mcp_write(MCP_GPIOA, counter++);
	}

	return 0;
}
