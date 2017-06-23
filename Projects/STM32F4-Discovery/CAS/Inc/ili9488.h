#ifndef ILI9488_H
#define ILI9488_H

#include "fonts.h"

/**
 * @defgroup ILI9488_Macros
 * @brief    Library defines
 * @{
 */

/**
 * @brief  This SPI pins are used on STM32F429-Discovery board
 */
#ifndef ILI9488_SPI
#define ILI9488_SPI           SPI1
#define ILI9488_SPI_PINS      SPI_PinsPack_1
#endif

/**
 * @brief  CS PIN for SPI, used as on STM32F429-Discovery board
 */
#ifndef ILI9488_CS_PIN
#define ILI9488_CS_PORT       GPIOC
#define ILI9488_CS_PIN        GPIO_PIN_2
#define ILI9488_CS_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define ILI9488_CS_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()

#endif

/**
 * @brief  WRX PIN for data/command, used as on STM32F429-Discovery board
 */
#ifndef ILI9488_WRX_PIN
#define ILI9488_WRX_PORT      GPIOD
#define ILI9488_WRX_PIN       GPIO_PIN_13
#define ILI9488_WRX_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define ILI9488_WRX_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()
#endif

/**
 * @brief  RESET for LCD
 */
#ifndef ILI9488_RST_PIN
#define ILI9488_RST_PORT      GPIOD
#define ILI9488_RST_PIN       GPIO_PIN_12
#define ILI9488_RST_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define ILI9488_RST_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()
#endif

/* LCD settings */
#define ILI9488_WIDTH        320
#define ILI9488_HEIGHT       480
#define ILI9488_PIXEL        153600

/* Colors */
#define ILI9488_COLOR_WHITE			0xFFFF
#define ILI9488_COLOR_BLACK			0x0000
#define ILI9488_COLOR_RED       0xF800
#define ILI9488_COLOR_GREEN			0x07E0
#define ILI9488_COLOR_GREEN2		0xB723
#define ILI9488_COLOR_BLUE			0x001F
#define ILI9488_COLOR_BLUE2			0x051D
#define ILI9488_COLOR_YELLOW		0xFFE0
#define ILI9488_COLOR_ORANGE		0xFBE4
#define ILI9488_COLOR_CYAN			0x07FF
#define ILI9488_COLOR_MAGENTA		0xA254
#define ILI9488_COLOR_GRAY			0x7BEF
#define ILI9488_COLOR_BROWN			0xBBCA

/* Transparent background, only for strings and chars */
#define ILI9488_TRANSPARENT			0x80000000

/**
 * @}
 */

/**
 * @defgroup ILI9488_Typedefs
 * @brief    Library Typedefs
 * @{
 */


/**
 * @brief  Possible orientations for LCD
 */
typedef enum {
	ILI9488_Orientation_Portrait_1,  /*!< Portrait orientation mode 1 */
	ILI9488_Orientation_Portrait_2,  /*!< Portrait orientation mode 2 */
	ILI9488_Orientation_Landscape_1, /*!< Landscape orientation mode 1 */
	ILI9488_Orientation_Landscape_2  /*!< Landscape orientation mode 2 */
} ILI9488_Orientation_t;

/**
 * @}
 */

/**
 * @defgroup ILI9488_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes ILI9488 LCD with LTDC peripheral
 *         It also initializes external SDRAM
 * @param  None
 * @retval None
 */
void ILI9488_Init(void);

/**
 * @brief  Draws single pixel to LCD
 * @param  x: X position for pixel
 * @param  y: Y position for pixel
 * @param  color: Color of pixel
 * @retval None
 */
void ILI9488_DrawPixel(uint16_t x, uint16_t y, uint32_t color);

/**
 * @brief  Fills entire LCD with color
 * @param  color: Color to be used in fill
 * @retval None
 */
void ILI9488_Fill(uint32_t color);

/**
 * @brief  Rotates LCD to specific orientation
 * @param  orientation: LCD orientation. This parameter can be a value of @ref ILI9488_Orientation_t enumeration
 * @retval None
 */
void ILI9488_Rotate(ILI9488_Orientation_t orientation);

/**
 * @brief  Puts single character to LCD
 * @param  x: X position of top left corner
 * @param  y: Y position of top left corner
 * @param  c: Character to be displayed
 * @param  *font: Pointer to @ref FontDef_t used font
 * @param  foreground: Color for char
 * @param  background: Color for char background
 * @retval None
 */
void ILI9488_Putc(uint16_t x, uint16_t y, char c, FontDef_t* font, uint32_t foreground, uint32_t background);

void ILI9488_PutChar(uint16_t x, uint16_t y, char c, FontDef_t* font, uint32_t color);

/**
 * @brief  Puts string to LCD
 * @param  x: X position of top left corner of first character in string
 * @param  y: Y position of top left corner of first character in string
 * @param  *str: Pointer to first character
 * @param  *font: Pointer to @ref FontDef_t used font
 * @param  foreground: Color for string
 * @param  background: Color for string background
 * @retval None
 */
void ILI9488_Puts(uint16_t x, uint16_t y, char* str, FontDef_t *font, uint32_t foreground, uint32_t background);

void ILI9488_PutString(uint16_t x, uint16_t y, char* str, FontDef_t *font, uint32_t color);

/**
 * @brief  Gets width and height of box with text
 * @param  *str: Pointer to first character
 * @param  *font: Pointer to @ref FontDef_t used font
 * @param  *width: Pointer to variable to store width
 * @param  *height: Pointer to variable to store height
 * @retval None
 */
void ILI9488_GetStringSize(char* str, FontDef_t* font, uint16_t* width, uint16_t* height);

/**
 * @brief  Draws line to LCD
 * @param  x0: X coordinate of starting point
 * @param  y0: Y coordinate of starting point
 * @param  x1: X coordinate of ending point
 * @param  y1: Y coordinate of ending point
 * @param  color: Line color
 * @retval None
 */
void ILI9488_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color);

/**
 * @brief  Draws rectangle on LCD
 * @param  x0: X coordinate of top left point
 * @param  y0: Y coordinate of top left point
 * @param  x1: X coordinate of bottom right point
 * @param  y1: Y coordinate of bottom right point
 * @param  color: Rectangle color
 * @retval None
 */
void ILI9488_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color);

/**
 * @brief  Draws filled rectangle on LCD
 * @param  x0: X coordinate of top left point
 * @param  y0: Y coordinate of top left point
 * @param  x1: X coordinate of bottom right point
 * @param  y1: Y coordinate of bottom right point
 * @param  color: Rectangle color
 * @retval None
 */
void ILI9488_DrawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color);

/**
 * @brief  Draws circle on LCD
 * @param  x0: X coordinate of center circle point
 * @param  y0: Y coordinate of center circle point
 * @param  r: Circle radius
 * @param  color: Circle color
 * @retval None
 */
void ILI9488_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color);

/**
 * @brief  Draws filled circle on LCD
 * @param  x0: X coordinate of center circle point
 * @param  y0: Y coordinate of center circle point
 * @param  r: Circle radius
 * @param  color: Circle color
 * @retval None
 */
void ILI9488_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color);

/**
 * @brief   Enables display
 * @note    After initialization, LCD is enabled and you don't need to call this function
 * @param   None
 * @retval  None
 */
void ILI9488_DisplayOn(void);

/**
 * @brief   Disables display
 * @param   None
 * @retval  None
 */
void ILI9488_DisplayOff(void);

void ILI9488_Test(void);
void ILI9488_Init(void);
void ILI9488_InitPins(void);
void ILI9488_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pBmp);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif

