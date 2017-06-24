#include "stm32f4xx_hal.h"
#include "ili9488.h"

extern SPI_HandleTypeDef SpiHandle;

/**
 * @brief  Orientation
 * @note   Used private
 */
typedef enum {
	ILI9488_Landscape,
	ILI9488_Portrait
} ILI9488_Orientation;

/**
 * @brief  LCD options
 * @note   Used private
 */
typedef struct {
	uint16_t width;
	uint16_t height;
	ILI9488_Orientation orientation; // 1 = portrait; 0 = landscape
} ILI931_Options_t;

/* Pin definitions */
#define ILI9488_RST_SET				HAL_GPIO_WritePin(ILI9488_RST_PORT, ILI9488_RST_PIN, GPIO_PIN_SET)
#define ILI9488_RST_RESET			HAL_GPIO_WritePin(ILI9488_RST_PORT, ILI9488_RST_PIN, GPIO_PIN_RESET)
#define ILI9488_CS_SET				HAL_GPIO_WritePin(ILI9488_CS_PORT, ILI9488_CS_PIN, GPIO_PIN_SET)
#define ILI9488_CS_RESET			HAL_GPIO_WritePin(ILI9488_CS_PORT, ILI9488_CS_PIN, GPIO_PIN_RESET)
#define ILI9488_WRX_SET				HAL_GPIO_WritePin(ILI9488_WRX_PORT, ILI9488_WRX_PIN, GPIO_PIN_SET)
#define ILI9488_WRX_RESET			HAL_GPIO_WritePin(ILI9488_WRX_PORT, ILI9488_WRX_PIN, GPIO_PIN_RESET)

/* Private defines */
#define ILI9488_RESET				0x01
#define ILI9488_SLEEP_OUT			0x11
#define ILI9488_GAMMA				0x26
#define ILI9488_DISPLAY_OFF			0x28
#define ILI9488_DISPLAY_ON			0x29
#define ILI9488_COLUMN_ADDR			0x2A
#define ILI9488_PAGE_ADDR			0x2B
#define ILI9488_GRAM				0x2C
#define ILI9488_MAC					0x36
#define ILI9488_PIXEL_FORMAT		0x3A
#define ILI9488_WDB					0x51
#define ILI9488_WCD					0x53
#define ILI9488_RGB_INTERFACE		0xB0
#define ILI9488_FRC					0xB1
#define ILI9488_BPC					0xB5
#define ILI9488_DFC					0xB6
#define ILI9488_POWER1				0xC0
#define ILI9488_POWER2				0xC1
#define ILI9488_VCOM1				0xC5
#define ILI9488_VCOM2				0xC7
#define ILI9488_POWERA				0xCB
#define ILI9488_POWERB				0xCF
#define ILI9488_PGAMMA				0xE0
#define ILI9488_NGAMMA				0xE1
#define ILI9488_DTCA				0xE8
#define ILI9488_DTCB				0xEA
#define ILI9488_POWER_SEQ			0xED
#define ILI9488_3GAMMA_EN			0xF2
#define ILI9488_INTERFACE			0xF6
#define ILI9488_PRC					0xF7

#define Delay(x)        HAL_Delay(10 * (x))

/* Pin functions */
uint16_t ILI9488_x;
uint16_t ILI9488_y;
ILI931_Options_t ILI9488_Opts;
uint8_t ILI9488_INT_CalledFromPuts = 0;

/* Private functions */
void ILI9488_InitLCD(void);
void ILI9488_SendData(uint8_t data);
void ILI9488_SendMultipleData(uint8_t *pData, uint32_t len);
void ILI9488_SendCommand(uint8_t data);
void ILI9488_Delay(volatile unsigned int delay);
void ILI9488_SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void ILI9488_INT_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color);

void ILI9488_Test(void)
{
}

void ILI9488_InitPins(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the CS Clock */
  ILI9488_CS_CLK_ENABLE();

  /* Configure the GPIO pin */
  GPIO_InitStruct.Pin = ILI9488_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;

  HAL_GPIO_Init(ILI9488_CS_PORT, &GPIO_InitStruct);

  /* Enable the WRX Clock */
  ILI9488_WRX_CLK_ENABLE();

  /* Configure the GPIO pin */
  GPIO_InitStruct.Pin = ILI9488_WRX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;

  HAL_GPIO_Init(ILI9488_WRX_PORT, &GPIO_InitStruct);

  /* Enable the RST Clock */
  ILI9488_RST_CLK_ENABLE();

  /* Configure the GPIO pin */
  GPIO_InitStruct.Pin = ILI9488_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

  HAL_GPIO_Init(ILI9488_RST_PORT, &GPIO_InitStruct);
}

void ILI9488_Init(void)
{
	ILI9488_InitPins();

	/* CS high */
	ILI9488_CS_SET;

	/* Init SPI */
	//SPI_Init(ILI9488_SPI, ILI9488_SPI_PINS);

	/* Init DMA for SPI */
	//SPI_DMA_Init(ILI9488_SPI);

	/* Init LCD */
	ILI9488_InitLCD();

	/* Set default settings */
	ILI9488_x = ILI9488_y = 0;
	ILI9488_Opts.width = ILI9488_WIDTH;
	ILI9488_Opts.height = ILI9488_HEIGHT;
	ILI9488_Opts.orientation = ILI9488_Portrait;
//	ILI9488_Opts.orientation = ILI9488_Landscape;

	/* Fill with white color */
//	ILI9488_Fill(ILI9488_COLOR_CYAN);
//	ILI9488_Fill(0xFFFFFF);
//        ILI9488_INT_Fill(0, 0, 320 - 1, 480, ILI9488_COLOR_RED);
}

void ILI9488_InitLCD(void)
{
	/* Force reset */
	ILI9488_RST_RESET;
	ILI9488_Delay(20000);
	ILI9488_RST_SET;

	/* Delay for RST response */
	ILI9488_Delay(20000);

	/* Software reset */
	ILI9488_SendCommand(ILI9488_RESET);
	ILI9488_Delay(50000);

#define SPI4_CMD_8bit   ILI9488_SendCommand
#define SPI4_DAT_8bit  ILI9488_SendData
	SPI4_CMD_8bit(0xE0);
	SPI4_DAT_8bit(0x00);
	SPI4_DAT_8bit(0x03);
	SPI4_DAT_8bit(0x0C);
	SPI4_DAT_8bit(0x09);
	SPI4_DAT_8bit(0x17);
	SPI4_DAT_8bit(0x09);
	SPI4_DAT_8bit(0x3E);
	SPI4_DAT_8bit(0x89);
	SPI4_DAT_8bit(0x49);
	SPI4_DAT_8bit(0x08);
	SPI4_DAT_8bit(0x0D);
	SPI4_DAT_8bit(0x0A);
	SPI4_DAT_8bit(0x13);
	SPI4_DAT_8bit(0x15);
	SPI4_DAT_8bit(0x0f);

	SPI4_CMD_8bit(0xE1);
	SPI4_DAT_8bit(0x00);
	SPI4_DAT_8bit(0x11);
	SPI4_DAT_8bit(0x15);
	SPI4_DAT_8bit(0x03);
	SPI4_DAT_8bit(0x0F);
	SPI4_DAT_8bit(0x05);
	SPI4_DAT_8bit(0x2D);
	SPI4_DAT_8bit(0x34);
	SPI4_DAT_8bit(0x41);
	SPI4_DAT_8bit(0x02);
	SPI4_DAT_8bit(0x0B);
	SPI4_DAT_8bit(0x0A);
	SPI4_DAT_8bit(0x33);
	SPI4_DAT_8bit(0x37);
	SPI4_DAT_8bit(0x0f);

	SPI4_CMD_8bit(0xC0);
	SPI4_DAT_8bit(0x17);
	SPI4_DAT_8bit(0x15);

	SPI4_CMD_8bit(0xC1);
	SPI4_DAT_8bit(0x41);

	SPI4_CMD_8bit(0xC5);
	SPI4_DAT_8bit(0x00);
	SPI4_DAT_8bit(0x12); // VCOM
	SPI4_DAT_8bit(0x80);

	SPI4_CMD_8bit(0x36);
	SPI4_DAT_8bit(0xC8);

	SPI4_CMD_8bit(0x3A);  //Interface Mode Control
	SPI4_DAT_8bit(0x66);  //6-6-6

	SPI4_CMD_8bit(0XB0);  //Interface Mode Control
	SPI4_DAT_8bit(0x00);

	SPI4_CMD_8bit(0xB4);
	SPI4_DAT_8bit(0x02);

	SPI4_CMD_8bit(0xB6);  //mcu-interface
	SPI4_DAT_8bit(0x02);
	SPI4_DAT_8bit(0x22);
	SPI4_DAT_8bit(0x3B);


	SPI4_CMD_8bit(0xE9);
	SPI4_DAT_8bit(0x00);

	SPI4_CMD_8bit(0XF7);
	SPI4_DAT_8bit(0xA9);
	SPI4_DAT_8bit(0x51);
	SPI4_DAT_8bit(0x2C);
	SPI4_DAT_8bit(0x82);

	SPI4_CMD_8bit(0x11); //Exit Sleep
	Delay(15);
	SPI4_CMD_8bit(0x29); //Display on
	Delay(10);
}

//void ili9488_set_orientation(uint8_t flags)
//{
//	/* Flip X/Y and reverse X orientation and set BGR mode*/
//	uint8_t madctl = 0x68;
//
//	/* Pretend the display is in landscape mode by default to match other display drivers */
//	//flags ^= ILI9488_SWITCH_XY | ILI9488_FLIP_X;
//
//	if (flags & ILI9488_FLIP_X) {
//		madctl &= ~(1 << 6);
//	}
//
//	if (flags & ILI9488_FLIP_Y) {
//		madctl |= 1 << 7;
//	}
//
//	if (flags & ILI9488_SWITCH_XY) {
//		madctl &= ~(1 << 5);
//	}
//
//	ili9488_send_command(ILI9488_CMD_MEMORY_ACCESS_CONTROL);
//	ili9488_send_byte(madctl);
//	ili9488_wait_for_send_done();
//	ili9488_deselect_chip();
//}

void ILI9488_DisplayOn(void) {
	ILI9488_SendCommand(ILI9488_DISPLAY_ON);
}

void ILI9488_DisplayOff(void) {
	ILI9488_SendCommand(ILI9488_DISPLAY_OFF);
}

void ILI9488_SendCommand(uint8_t data) {
	ILI9488_WRX_RESET;
	ILI9488_CS_RESET;
	//SPI_Send(ILI9488_SPI, data);
	HAL_SPI_Transmit(&SpiHandle, (uint8_t*)&data, 1, 5000);
	ILI9488_CS_SET;
}

void ILI9488_SendData(uint8_t data) {
	ILI9488_WRX_SET;
	ILI9488_CS_RESET;
//	SPI_Send(ILI9488_SPI, data);
	HAL_SPI_Transmit(&SpiHandle, (uint8_t*)&data, 1, 5000);
	ILI9488_CS_SET;
}

void ILI9488_SendMultipleData(uint8_t *pData, uint32_t len) {
	ILI9488_WRX_SET;
	ILI9488_CS_RESET;
	HAL_SPI_Transmit(&SpiHandle, (uint8_t*)pData, len, 5000);
	ILI9488_CS_SET;
}

void ILI9488_DrawPixel(uint16_t x, uint16_t y, uint32_t color) {
	ILI9488_SetCursorPosition(x, y, x, y);

	ILI9488_SendCommand(ILI9488_GRAM);
#if 0	// RGB-565
	ILI9488_SendData((color >> 8) & 0xFF);
	ILI9488_SendData(color & 0xFF);
#else	// RGB-666
	ILI9488_SendData((color >> 16) & 0xFF);
	ILI9488_SendData((color >> 8) & 0xFF);
	ILI9488_SendData(color & 0xFF);
#endif
}


void ILI9488_SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	ILI9488_SendCommand(ILI9488_COLUMN_ADDR);
	ILI9488_SendData((x1 >> 8) & 0xFF);
	ILI9488_SendData(x1 & 0xFF);
	ILI9488_SendData((x2 >> 8) & 0xFF);
	ILI9488_SendData(x2 & 0xFF);

	ILI9488_SendCommand(ILI9488_PAGE_ADDR);
	ILI9488_SendData((y1 >> 8) & 0xFF);
	ILI9488_SendData(y1 & 0xFF);
	ILI9488_SendData((y2 >> 8) & 0xFF);
	ILI9488_SendData(y2 & 0xFF);
}

void ILI9488_Fill(uint32_t color) {
	/* Fill entire screen */
	ILI9488_INT_Fill(0, 0, ILI9488_Opts.width - 1, ILI9488_Opts.height, color);
}

void ILI9488_INT_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color)
{
#if 1
  for (uint16_t y = y0; y < y1; y++) {
    for (uint16_t x = x0; x < x1; x++) {
      ILI9488_DrawPixel(x, y, color);
    }
  }
#else
	uint32_t pixels_count;

	/* Set cursor position */
	ILI9488_SetCursorPosition(x0, y0, x1, y1);

	/* Set command for GRAM data */
	ILI9488_SendCommand(ILI9488_GRAM);

	/* Calculate pixels count */
	pixels_count = (x1 - x0 + 1) * (y1 - y0 + 1);

	/* Send everything */
	ILI9488_CS_RESET;
	ILI9488_WRX_SET;

	/* Go to 16-bit SPI mode */
	SPI_SetDataSize(ILI9488_SPI, SPI_DataSize_16b);

	/* Send first 65535 bytes, SPI MUST BE IN 16-bit MODE */
	SPI_DMA_SendHalfWord(ILI9488_SPI, color, (pixels_count > 0xFFFF) ? 0xFFFF : pixels_count);
//	SPI_DMA_SendWord(ILI9488_SPI, color, (pixels_count > 0xFFFF) ? 0xFFFF : pixels_count);
	/* Wait till done */
	while (SPI_DMA_Working(ILI9488_SPI));

	/* Check again */
	if (pixels_count > 0xFFFF) {
		/* Send remaining data */
		SPI_DMA_SendHalfWord(ILI9488_SPI, color, pixels_count - 0xFFFF);
//		SPI_DMA_SendWord(ILI9488_SPI, color, pixels_count - 0xFFFF);
		/* Wait till done */
		while (SPI_DMA_Working(ILI9488_SPI));
	}

	ILI9488_CS_SET;

	/* Go back to 8-bit SPI mode */
	SPI_SetDataSize(ILI9488_SPI, SPI_DataSize_8b);
#endif
}

void ILI9488_Delay(volatile unsigned int delay) {
	for (; delay != 0; delay--);
}

void ILI9488_Rotate(ILI9488_Orientation_t orientation) {
	ILI9488_SendCommand(ILI9488_MAC);
	if (orientation == ILI9488_Orientation_Portrait_1) {
		ILI9488_SendData(0x58);
	} else if (orientation == ILI9488_Orientation_Portrait_2) {
		ILI9488_SendData(0x88);
	} else if (orientation == ILI9488_Orientation_Landscape_1) {
		ILI9488_SendData(0x28);
	} else if (orientation == ILI9488_Orientation_Landscape_2) {
		ILI9488_SendData(0xE8);
	}

	if (orientation == ILI9488_Orientation_Portrait_1 || orientation == ILI9488_Orientation_Portrait_2) {
		ILI9488_Opts.width = ILI9488_WIDTH;
		ILI9488_Opts.height = ILI9488_HEIGHT;
		ILI9488_Opts.orientation = ILI9488_Portrait;
	} else {
		ILI9488_Opts.width = ILI9488_HEIGHT;
		ILI9488_Opts.height = ILI9488_WIDTH;
		ILI9488_Opts.orientation = ILI9488_Landscape;
	}
}

void ILI9488_Puts(uint16_t x, uint16_t y, char *str, FontDef_t *font, uint32_t foreground, uint32_t background) {
	uint16_t startX = x;

	/* Set X and Y coordinates */
	ILI9488_x = x;
	ILI9488_y = y;

	while (*str) {
		/* New line */
		if (*str == '\n') {
			ILI9488_y += font->FontHeight + 1;
			/* if after \n is also \r, than go to the left of the screen */
			if (*(str + 1) == '\r') {
				ILI9488_x = 0;
				str++;
			} else {
				ILI9488_x = startX;
			}
			str++;
			continue;
		} else if (*str == '\r') {
			str++;
			continue;
		}

		/* Put character to LCD */
		ILI9488_Putc(ILI9488_x, ILI9488_y, *str++, font, foreground, background);
	}
}

void ILI9488_PutString(uint16_t x, uint16_t y, char *str, FontDef_t *font, uint32_t color) {
	uint16_t startX = x;

	/* Set X and Y coordinates */
	ILI9488_x = x;
	ILI9488_y = y;

	while (*str) {
		/* New line */
		if (*str == '\n') {
			ILI9488_y += font->FontHeight + 1;
			/* if after \n is also \r, than go to the left of the screen */
			if (*(str + 1) == '\r') {
				ILI9488_x = 0;
				str++;
			} else {
				ILI9488_x = startX;
			}
			str++;
			continue;
		} else if (*str == '\r') {
			str++;
			continue;
		}

		/* Put character to LCD */
		ILI9488_PutChar(ILI9488_x, ILI9488_y, *str++, font, color);
	}
}

void ILI9488_GetStringSize(char *str, FontDef_t *font, uint16_t *width, uint16_t *height) {
	uint16_t w = 0;
	*height = font->FontHeight;
	while (*str++) {
		w += font->FontWidth;
	}
	*width = w;
}

void ILI9488_Putc(uint16_t x, uint16_t y, char c, FontDef_t *font, uint32_t foreground, uint32_t background) {
	uint32_t i, b, j;
	/* Set coordinates */
	ILI9488_x = x;
	ILI9488_y = y;

	if ((ILI9488_x + font->FontWidth) > ILI9488_Opts.width) {
		/* If at the end of a line of display, go to new line and set x to 0 position */
		ILI9488_y += font->FontHeight;
		ILI9488_x = 0;
	}

	/* Draw rectangle for background */
	ILI9488_INT_Fill(ILI9488_x, ILI9488_y, ILI9488_x + font->FontWidth, ILI9488_y + font->FontHeight, background);

	/* Draw font data */
	for (i = 0; i < font->FontHeight; i++) {
		b = font->data[(c - 32) * font->FontHeight + i];
		for (j = 0; j < font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				ILI9488_DrawPixel(ILI9488_x + j, (ILI9488_y + i), foreground);
			}
		}
	}

	/* Set new pointer */
	ILI9488_x += font->FontWidth;
}

void ILI9488_PutChar(uint16_t x, uint16_t y, char c, FontDef_t *font, uint32_t color) {
	uint32_t i, b, j;
	/* Set coordinates */
	ILI9488_x = x;
	ILI9488_y = y;

	if ((ILI9488_x + font->FontWidth) > ILI9488_Opts.width) {
		/* If at the end of a line of display, go to new line and set x to 0 position */
		ILI9488_y += font->FontHeight;
		ILI9488_x = 0;
	}

	/* Draw font data */
	for (i = 0; i < font->FontHeight; i++) {
		b = font->data[(c - 32) * font->FontHeight + i];
		for (j = 0; j < font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				ILI9488_DrawPixel(ILI9488_x + j, (ILI9488_y + i), color);
			}
		}
	}

	/* Set new pointer */
	ILI9488_x += font->FontWidth;
}

void ILI9488_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color) {
	/* Code by dewoller: https://github.com/dewoller */

	int16_t dx, dy, sx, sy, err, e2;
	uint16_t tmp;

	/* Check for overflow */
	if (x0 >= ILI9488_Opts.width) {
		x0 = ILI9488_Opts.width - 1;
	}
	if (x1 >= ILI9488_Opts.width) {
		x1 = ILI9488_Opts.width - 1;
	}
	if (y0 >= ILI9488_Opts.height) {
		y0 = ILI9488_Opts.height - 1;
	}
	if (y1 >= ILI9488_Opts.height) {
		y1 = ILI9488_Opts.height - 1;
	}

	/* Check correction */
	if (x0 > x1) {
		tmp = x0;
		x0 = x1;
		x1 = tmp;
	}
	if (y0 > y1) {
		tmp = y0;
		y0 = y1;
		y1 = tmp;
	}

	dx = x1 - x0;
	dy = y1 - y0;

	/* Vertical or horizontal line */
	if (dx == 0 || dy == 0) {
		ILI9488_INT_Fill(x0, y0, x1, y1, color);
		return;
	}

	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	while (1) {
		ILI9488_DrawPixel(x0, y0, color);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}
}

void ILI9488_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color) {
	ILI9488_DrawLine(x0, y0, x1, y0, color); //Top
	ILI9488_DrawLine(x0, y0, x0, y1, color);	//Left
	ILI9488_DrawLine(x1, y0, x1, y1, color);	//Right
	ILI9488_DrawLine(x0, y1, x1, y1, color);	//Bottom
}

void ILI9488_DrawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color) {
	uint16_t tmp;

	/* Check correction */
	if (x0 > x1) {
		tmp = x0;
		x0 = x1;
		x1 = tmp;
	}
	if (y0 > y1) {
		tmp = y0;
		y0 = y1;
		y1 = tmp;
	}

	/* Fill rectangle */
	ILI9488_INT_Fill(x0, y0, x1, y1, color);

	/* CS HIGH back */
	ILI9488_CS_SET;
}

void ILI9488_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    ILI9488_DrawPixel(x0, y0 + r, color);
    ILI9488_DrawPixel(x0, y0 - r, color);
    ILI9488_DrawPixel(x0 + r, y0, color);
    ILI9488_DrawPixel(x0 - r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ILI9488_DrawPixel(x0 + x, y0 + y, color);
        ILI9488_DrawPixel(x0 - x, y0 + y, color);
        ILI9488_DrawPixel(x0 + x, y0 - y, color);
        ILI9488_DrawPixel(x0 - x, y0 - y, color);

        ILI9488_DrawPixel(x0 + y, y0 + x, color);
        ILI9488_DrawPixel(x0 - y, y0 + x, color);
        ILI9488_DrawPixel(x0 + y, y0 - x, color);
        ILI9488_DrawPixel(x0 - y, y0 - x, color);
    }
}

void ILI9488_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    ILI9488_DrawPixel(x0, y0 + r, color);
    ILI9488_DrawPixel(x0, y0 - r, color);
    ILI9488_DrawPixel(x0 + r, y0, color);
    ILI9488_DrawPixel(x0 - r, y0, color);
    ILI9488_DrawLine(x0 - r, y0, x0 + r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ILI9488_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
        ILI9488_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);

        ILI9488_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
        ILI9488_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
    }
}

/**
  * @brief  Draws a bitmap picture loaded in the STM32 MCU internal memory.
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pBmp: Pointer to Bmp picture address
  * @retval None
  */
void ILI9488_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pBmp)
{
  uint32_t height = 0, width  = 0;

  /* Read bitmap width */
  width = *(uint16_t *) (pBmp + 18);
  width |= (*(uint16_t *) (pBmp + 20)) << 16;

  /* Read bitmap height */
  height = *(uint16_t *) (pBmp + 22);
  height |= (*(uint16_t *) (pBmp + 24)) << 16;

  printf("h: %d, w: %d\r\n", width, height);

  uint32_t index = 0, size = 0;
  /* Read bitmap size */
  size = *(volatile uint16_t *) (pBmp + 2);
  size |= (*(volatile uint16_t *) (pBmp + 4)) << 16;
  /* Get bitmap data address offset */
  index = *(volatile uint16_t *) (pBmp + 10);
  index |= (*(volatile uint16_t *) (pBmp + 12)) << 16;
  printf("size: %d, index: %d\r\n", size, index);
  size = (size - index) / 2;
  pBmp += index;

#if 0
  ILI9488_SetCursorPosition(Xpos, Ypos, Xpos + width - 1, Ypos + height - 1);
  uint8_t *p0 = pBmp;
  for (int i = 0; i < width; i++) {
	  for (int j = 0; j < height; j++) {
		uint32_t color = (*p0++) << 16 | (*p0++) << 8 | (*p0++);
		ILI9488_DrawPixel(Xpos + i, Ypos + j, color);
	  }
  }
  ILI9488_SetCursorPosition(0, 0, ILI9488_WIDTH, ILI9488_HEIGHT);
  return;
#endif

//  SetDisplayWindow(Xpos, Ypos, width, height);
  ILI9488_SetCursorPosition(Xpos, Ypos, Xpos + width - 1, Ypos + height - 1);

  ILI9488_SendCommand(0x36);
  ILI9488_SendData(0x48);

  ILI9488_SendCommand(ILI9488_GRAM);
#if 0	// RGB888
  ILI9488_SendMultipleData(pBmp, size * 2);
#elif 1	// RGB-888/666
  uint8_t *p = pBmp;
  for (int i = 0; i < height * width; i++)
  {
	ILI9488_SendData(*p++);
	ILI9488_SendData(*p++);
	ILI9488_SendData(*p++);
  }
#else	// RGB-565
  uint16_t *p = (uint16_t *)pBmp;
  for (int i = 0; i < height * width; i++)
  {
	ILI9488_SendData(((*p >> 11) & 0x1F) << 2);
	ILI9488_SendData(((*p >> 5) & 0x3F) << 2);
	ILI9488_SendData((*p & 0x1F) << 2);
    p++;
  }
#endif

	SPI4_CMD_8bit(0x36);
	SPI4_DAT_8bit(0xC8);

//  SetDisplayWindow(0, 0, BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  ILI9488_SetCursorPosition(0, 0, ILI9488_WIDTH, ILI9488_HEIGHT);
}
