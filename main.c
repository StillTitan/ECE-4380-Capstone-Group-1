/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Apartment Energy Monitor (RTOS + AP Mode)
  *
  * Features:
  *  - STM32 Nucleo-F411RE
  *  - 2x INA219 (current/voltage sensors on USB loads)
  *  - SSD1306 0.96" OLED (I2C)
  *  - ESP-01S as Wi-Fi Access Point (SoftAP)
  *  - Custom "RTOS":
  *      - 1ms SysTick tick (interrupt-driven)
  *      - Task table + scheduler for 3 periodic tasks
  *      - Inter-task communication via a simple queue
  *  - OLED shows CH1, CH2, and total Watts + Wh
  *
  * ESP-01S AP:
  *  - SSID:     EnergyMonitor_AP
  *  - Password: password123
  *
  * Your phone can join that Wi-Fi network (no internet required).
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Simple task control block for RTOS */
typedef struct {
  void     (*taskFunc)(void);  // Task function
  uint32_t periodTicks;        // Period in ticks (1 tick = 1 ms)
  uint32_t nextRelease;        // Next release time (tick)
} RTOS_Task_t;

/* INA219 channel data */
typedef struct {
  float v_V;       // latest voltage
  float i_A;       // latest current
  float p_W;       // latest power
  float p_W_filt;  // filtered power
  float E_Wh;      // accumulated energy
} ChannelData_t;

/* Sample structure passed between tasks via queue */
typedef struct {
  float ch1_W;
  float ch2_W;
  float total_W;
  float total_Wh;
} EnergySample_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* === INA219 definitions === */
#define INA219_ADDR1     (0x40 << 1)   // INA219 #1 (A0 = 0)
#define INA219_ADDR2     (0x41 << 1)   // INA219 #2 (A0 = 1)

#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT_V      0x01
#define INA219_REG_BUS_V        0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

/* Calibration: assumes ~0.1 ohm shunt, Current_LSB = 100 uA, Power_LSB = 2 mW */
static uint16_t INA219_CalibrationValue = 4096;
static float INA219_Current_LSB = 0.0001f;  // 100 µA/LSB
static float INA219_Power_LSB   = 0.002f;   // 2 mW/LSB

/* === OLED (SSD1306) definitions === */
#define OLED_ADDR   (0x3C << 1)
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_PAGES  (OLED_HEIGHT / 8)

/* Smoothing factor for exponential moving average */
static const float alpha = 0.2f;

/* === ESP-01S Access Point configuration === */
#define ESP_AP_SSID     "EnergyMonitor_AP"
#define ESP_AP_PASSWORD "password123"
#define ESP_AP_CHANNEL  5   // Wi-Fi channel
#define ESP_AP_ENC      3   // WPA2_PSK (for AT+CWSAP)

/* === RTOS constants === */
#define RTOS_MAX_TASKS  4
#define RTOS_QUEUE_SIZE 8
#define RTOS_TICK_HZ    1000u   // 1 tick = 1 ms

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//I2C_HandleTypeDef hi2c1;
//UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* === RTOS globals === */
volatile uint32_t gRTOS_Tick = 0;               // incremented in SysTick callback
static RTOS_Task_t gRTOS_Tasks[RTOS_MAX_TASKS];
static uint8_t gRTOS_TaskCount = 0;

typedef struct {
  EnergySample_t buf[RTOS_QUEUE_SIZE];
  uint8_t head;   // write index
  uint8_t tail;   // read index
  uint8_t count;  // number of items
} RTOS_Queue_t;

static RTOS_Queue_t gEnergyQueue;

/* INA219 channels */
ChannelData_t ch1 = {0}, ch2 = {0};

/* OLED framebuffer */
static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

//static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* Debug UART helpers */
void debug_print(const char *s);
void debug_println(const char *s);

/* I2C scanner (debug) */
void I2C_Scan(void);

/* INA219 helpers */
HAL_StatusTypeDef INA_WriteReg(uint16_t addr, uint8_t reg, uint16_t val);
HAL_StatusTypeDef INA_ReadReg(uint16_t addr, uint8_t reg, uint16_t *val);
void INA_Init(uint16_t addr);
void INA_Read(uint16_t addr, float *voltage, float *current, float *power);

/* OLED helpers */
void OLED_Cmd(uint8_t cmd);
void OLED_Init(void);
void OLED_ClearBuffer(void);
void OLED_Update(void);
void OLED_DrawChar(uint8_t x, uint8_t y, char c);
void OLED_DrawString(uint8_t x, uint8_t y, const char *s);

/* ESP-01S helpers (AP only) */
void ESP_SendRaw(const char *s);
void ESP_StartAccessPoint(void);

/* RTOS API */
void RTOS_Init(void);
int  RTOS_AddPeriodicTask(void (*func)(void), uint32_t periodMs);
void RTOS_RunScheduler(void);
void RTOS_Queue_Push(EnergySample_t *s);
int  RTOS_Queue_Pop(EnergySample_t *s);

/* Application tasks */
void Task_SampleINA(void);
void Task_UpdateOLED(void);
void Task_Status(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* === Debug UART === */
void debug_print(const char *s) {
  HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}
void debug_println(const char *s) {
  debug_print(s);
  debug_print("\r\n");
}

/* === I2C Scanner (optional) === */
void I2C_Scan(void) {
  debug_println("Scanning I2C bus...");
  char msg[64];

  for (uint8_t addr = 1; addr < 127; addr++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
      snprintf(msg, sizeof(msg), " - Found device at 0x%02X", addr);
      debug_println(msg);
    }
  }
  debug_println("I2C scan complete.\r\n");
}

/* === INA219 === */
HAL_StatusTypeDef INA_WriteReg(uint16_t addr, uint8_t reg, uint16_t val) {
  uint8_t data[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
  return HAL_I2C_Master_Transmit(&hi2c1, addr, data, 3, HAL_MAX_DELAY);
}

HAL_StatusTypeDef INA_ReadReg(uint16_t addr, uint8_t reg, uint16_t *val) {
  uint8_t buf[2];
  HAL_StatusTypeDef res;

  res = HAL_I2C_Master_Transmit(&hi2c1, addr, &reg, 1, HAL_MAX_DELAY);
  if (res != HAL_OK) return res;

  res = HAL_I2C_Master_Receive(&hi2c1, addr, buf, 2, HAL_MAX_DELAY);
  if (res != HAL_OK) return res;

  *val = (buf[0] << 8) | buf[1];
  return HAL_OK;
}

void INA_Init(uint16_t addr) {
  /* Calibration */
  INA_WriteReg(addr, INA219_REG_CALIBRATION, INA219_CalibrationValue);

  /* Config: 32V range, 320mV shunt, 12-bit bus+shunt, continuous mode */
  uint16_t config = 0;
  config |= (1 << 13);   // BRNG = 1 (32V)
  config |= (3 << 11);   // PG   = 3 (320mV)
  config |= (3 << 7);    // BADC = 3 (12-bit)
  config |= (3 << 3);    // SADC = 3 (12-bit)
  config |= 7;           // Mode = shunt+bus, continuous

  INA_WriteReg(addr, INA219_REG_CONFIG, config);
}

void INA_Read(uint16_t addr, float *voltage, float *current, float *power) {
  uint16_t raw;
  int16_t sraw;

  /* Bus voltage */
  INA_ReadReg(addr, INA219_REG_BUS_V, &raw);
  *voltage = ((raw >> 3) * 0.004f); // 4mV/bit, bits 0–2 are flags

  /* Current */
  INA_ReadReg(addr, INA219_REG_CURRENT, &raw);
  sraw = (int16_t)raw;
  *current = (float)sraw * INA219_Current_LSB;

  /* Power */
  INA_ReadReg(addr, INA219_REG_POWER, &raw);
  *power = (float)raw * INA219_Power_LSB;
}

/* === Minimal 5x7 glyphs for digits and a few letters === */
static void getGlyph5x7(char c, uint8_t out[5]) {
  memset(out, 0x00, 5);

  switch (c) {
    case '0': { uint8_t g[5] = {0x3E,0x51,0x49,0x45,0x3E}; memcpy(out,g,5); break; }
    case '1': { uint8_t g[5] = {0x00,0x42,0x7F,0x40,0x00}; memcpy(out,g,5); break; }
    case '2': { uint8_t g[5] = {0x42,0x61,0x51,0x49,0x46}; memcpy(out,g,5); break; }
    case '3': { uint8_t g[5] = {0x21,0x41,0x45,0x4B,0x31}; memcpy(out,g,5); break; }
    case '4': { uint8_t g[5] = {0x18,0x14,0x12,0x7F,0x10}; memcpy(out,g,5); break; }
    case '5': { uint8_t g[5] = {0x27,0x45,0x45,0x45,0x39}; memcpy(out,g,5); break; }
    case '6': { uint8_t g[5] = {0x3C,0x4A,0x49,0x49,0x30}; memcpy(out,g,5); break; }
    case '7': { uint8_t g[5] = {0x01,0x71,0x09,0x05,0x03}; memcpy(out,g,5); break; }
    case '8': { uint8_t g[5] = {0x36,0x49,0x49,0x49,0x36}; memcpy(out,g,5); break; }
    case '9': { uint8_t g[5] = {0x06,0x49,0x49,0x29,0x1E}; memcpy(out,g,5); break; }
    case '.': { uint8_t g[5] = {0x00,0x60,0x60,0x00,0x00}; memcpy(out,g,5); break; }
    case 'C': { uint8_t g[5] = {0x3C,0x42,0x41,0x41,0x22}; memcpy(out,g,5); break; }
    case 'H': { uint8_t g[5] = {0x7F,0x08,0x08,0x08,0x7F}; memcpy(out,g,5); break; }
    case 'T': { uint8_t g[5] = {0x01,0x01,0x7F,0x01,0x01}; memcpy(out,g,5); break; }
    case 'o': { uint8_t g[5] = {0x38,0x44,0x44,0x44,0x38}; memcpy(out,g,5); break; }
    case 't': { uint8_t g[5] = {0x04,0x3F,0x44,0x40,0x20}; memcpy(out,g,5); break; }
    case 'W': { uint8_t g[5] = {0x7E,0x20,0x18,0x20,0x7E}; memcpy(out,g,5); break; }
    case 'h': { uint8_t g[5] = {0x7F,0x08,0x04,0x04,0x78}; memcpy(out,g,5); break; }
    case ':': { uint8_t g[5] = {0x00,0x36,0x36,0x00,0x00}; memcpy(out,g,5); break; }
    case ' ': default: { uint8_t g[5] = {0x00,0x00,0x00,0x00,0x00}; memcpy(out,g,5); break; }
  }
}

/* === OLED === */
void OLED_Cmd(uint8_t cmd) {
  uint8_t data[2] = {0x00, cmd};
  HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, data, 2, HAL_MAX_DELAY);
}

void OLED_Init(void) {
  HAL_Delay(100);
  OLED_Cmd(0xAE);
  OLED_Cmd(0x20); OLED_Cmd(0x00); // horizontal mode
  OLED_Cmd(0xB0);
  OLED_Cmd(0xC8);
  OLED_Cmd(0x00);
  OLED_Cmd(0x10);
  OLED_Cmd(0x40);
  OLED_Cmd(0x81); OLED_Cmd(0x7F);
  OLED_Cmd(0xA1);
  OLED_Cmd(0xA6);
  OLED_Cmd(0xA8); OLED_Cmd(0x3F);
  OLED_Cmd(0xA4);
  OLED_Cmd(0xD3); OLED_Cmd(0x00);
  OLED_Cmd(0xD5); OLED_Cmd(0x80);
  OLED_Cmd(0xD9); OLED_Cmd(0xF1);
  OLED_Cmd(0xDA); OLED_Cmd(0x12);
  OLED_Cmd(0xDB); OLED_Cmd(0x40);
  OLED_Cmd(0x8D); OLED_Cmd(0x14);
  OLED_Cmd(0xAF);
}

void OLED_ClearBuffer(void) {
  memset(oled_buffer, 0x00, sizeof(oled_buffer));
}

void OLED_Update(void) {
  for (uint8_t page = 0; page < OLED_PAGES; page++) {
    OLED_Cmd(0xB0 + page);
    OLED_Cmd(0x00);
    OLED_Cmd(0x10);
    uint8_t control = 0x40;
    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, control, 1,
                      &oled_buffer[OLED_WIDTH * page],
                      OLED_WIDTH,
                      HAL_MAX_DELAY);
  }
}

void OLED_DrawChar(uint8_t x, uint8_t y, char c) {
  if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

  uint8_t glyph[5];
  getGlyph5x7(c, glyph);

  uint8_t page = y / 8;
  uint8_t bit_in_page = y % 8;

  for (uint8_t col = 0; col < 5; col++) {
    uint8_t line = glyph[col];
    uint16_t idx = page * OLED_WIDTH + x + col;
    if (idx >= sizeof(oled_buffer)) continue;
    oled_buffer[idx] |= (line << bit_in_page);
  }
}

void OLED_DrawString(uint8_t x, uint8_t y, const char *s) {
  while (*s && x < (OLED_WIDTH - 6)) {
    OLED_DrawChar(x, y, *s++);
    x += 6; // 5px char + 1px space
  }
}

/* === ESP-01S: Soft Access Point === */

void ESP_SendRaw(const char *s) {
  HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

/* Start ESP-01S as an Access Point:
 *  SSID: ESP_AP_SSID
 *  PASS: ESP_AP_PASSWORD
 */
void ESP_StartAccessPoint(void) {
  char cmd[128];

  debug_println("Initializing ESP-01S as Access Point...");

  ESP_SendRaw("AT\r\n");
  HAL_Delay(1000);

  /* Set Wi-Fi mode to SoftAP (2) */
  ESP_SendRaw("AT+CWMODE=2\r\n");
  HAL_Delay(1000);

  /* Configure AP: AT+CWSAP="ssid","pwd",channel,enc */
  snprintf(cmd, sizeof(cmd),
           "AT+CWSAP=\"%s\",\"%s\",%d,%d\r\n",
           ESP_AP_SSID, ESP_AP_PASSWORD,
           ESP_AP_CHANNEL, ESP_AP_ENC);

  debug_print("Starting AP SSID: ");
  debug_println(ESP_AP_SSID);
  ESP_SendRaw(cmd);
  HAL_Delay(2000);

  debug_println("ESP AP ready. Connect your phone to:");
  debug_print("SSID: "); debug_println(ESP_AP_SSID);
  debug_print("PASS: "); debug_println(ESP_AP_PASSWORD);
  debug_println("");
}

/* === RTOS implementation === */

/* Initialize RTOS structures */
void RTOS_Init(void) {
  gRTOS_TaskCount = 0;
  gRTOS_Tick = 0;
  gEnergyQueue.head = gEnergyQueue.tail = gEnergyQueue.count = 0;
}

/* Add a periodic task. periodMs is in milliseconds (ticks) */
int RTOS_AddPeriodicTask(void (*func)(void), uint32_t periodMs) {
  if (gRTOS_TaskCount >= RTOS_MAX_TASKS) return -1;

  uint32_t now = gRTOS_Tick;
  gRTOS_Tasks[gRTOS_TaskCount].taskFunc    = func;
  gRTOS_Tasks[gRTOS_TaskCount].periodTicks = periodMs;
  gRTOS_Tasks[gRTOS_TaskCount].nextRelease = now + periodMs;

  gRTOS_TaskCount++;
  return 0;
}

/* Simple FIFO queue: producer/consumer (not used in ISR, so no mutex) */
void RTOS_Queue_Push(EnergySample_t *s) {
  if (gEnergyQueue.count >= RTOS_QUEUE_SIZE) {
    // drop oldest to make room
    gEnergyQueue.tail = (gEnergyQueue.tail + 1) % RTOS_QUEUE_SIZE;
    gEnergyQueue.count--;
  }
  gEnergyQueue.buf[gEnergyQueue.head] = *s;
  gEnergyQueue.head = (gEnergyQueue.head + 1) % RTOS_QUEUE_SIZE;
  gEnergyQueue.count++;
}

int RTOS_Queue_Pop(EnergySample_t *s) {
  if (gEnergyQueue.count == 0) return -1; // empty queue
  *s = gEnergyQueue.buf[gEnergyQueue.tail];
  gEnergyQueue.tail = (gEnergyQueue.tail + 1) % RTOS_QUEUE_SIZE;
  gEnergyQueue.count--;
  return 0;
}

/* SysTick interrupt hook: integrates interrupts with scheduler.
 * HAL calls this every 1 ms.
 */
void HAL_SYSTICK_Callback(void) {
  gRTOS_Tick++;
}

/* RTOS scheduler: called from main loop, runs ready tasks cooperatively. */
void RTOS_RunScheduler(void) {
  uint32_t now = gRTOS_Tick;

  for (uint8_t i = 0; i < gRTOS_TaskCount; i++) {
    RTOS_Task_t *t = &gRTOS_Tasks[i];
    if ((int32_t)(now - t->nextRelease) >= 0) {
      // "context switch" into this task
      t->taskFunc();
      t->nextRelease += t->periodTicks;
    }
  }
}

/* === Application tasks === */

/* Task 1: Sample INA219s, filter, integrate Wh, push snapshot into queue */
void Task_SampleINA(void) {
  const float dt_ms = 100.0f;  // must match registration period (100 ms)
  float v, i, p;

  /* Channel 1 */
  INA_Read(INA219_ADDR1, &v, &i, &p);
  ch1.v_V = v;
  ch1.i_A = i;
  ch1.p_W = p;
  if (ch1.p_W_filt == 0.0f) ch1.p_W_filt = p;
  else ch1.p_W_filt = alpha * p + (1.0f - alpha) * ch1.p_W_filt;
  ch1.E_Wh += ch1.p_W_filt * (dt_ms / 3600000.0f);

  /* Channel 2 */
  INA_Read(INA219_ADDR2, &v, &i, &p);
  ch2.v_V = v;
  ch2.i_A = i;
  ch2.p_W = p;
  if (ch2.p_W_filt == 0.0f) ch2.p_W_filt = p;
  else ch2.p_W_filt = alpha * p + (1.0f - alpha) * ch2.p_W_filt;
  ch2.E_Wh += ch2.p_W_filt * (dt_ms / 3600000.0f);

  /* Push snapshot to queue (inter-task communication) */
  EnergySample_t s;
  s.ch1_W    = ch1.p_W_filt;
  s.ch2_W    = ch2.p_W_filt;
  s.total_W  = ch1.p_W_filt + ch2.p_W_filt;
  s.total_Wh = ch1.E_Wh + ch2.E_Wh;
  RTOS_Queue_Push(&s);

  /* Optional debug (conflicts with ESP UART usage in practice) */
  // char msg[128];
  // snprintf(msg, sizeof(msg),
  //          "SampleINA: CH1=%.2fW CH2=%.2fW Tot=%.2fW\r\n",
  //          s.ch1_W, s.ch2_W, s.total_W);
  // debug_print(msg);
}

/* Task 2: Update OLED display using latest sample from queue */
void Task_UpdateOLED(void) {
  char line[32];
  EnergySample_t s;
  int hasSample = (RTOS_Queue_Pop(&s) == 0);

  float ch1W  = hasSample ? s.ch1_W    : ch1.p_W_filt;
  float ch2W  = hasSample ? s.ch2_W    : ch2.p_W_filt;
  float totW  = hasSample ? s.total_W  : (ch1.p_W_filt + ch2.p_W_filt);
  float totWh = hasSample ? s.total_Wh : (ch1.E_Wh + ch2.E_Wh);

  OLED_ClearBuffer();

  snprintf(line, sizeof(line), "CH1: %4.2fW %4.2fWh", ch1W, ch1.E_Wh);
  OLED_DrawString(0, 0, line);

  snprintf(line, sizeof(line), "CH2: %4.2fW %4.2fWh", ch2W, ch2.E_Wh);
  OLED_DrawString(0, 16, line);

  snprintf(line, sizeof(line), "Tot: %4.2fW %4.2fWh", totW, totWh);
  OLED_DrawString(0, 32, line);

  OLED_Update();
}

/* Task 3: Status / heartbeat (e.g., debug counter) */
void Task_Status(void) {
  static uint32_t counter = 0;
  char msg[64];
  snprintf(msg, sizeof(msg), "Status tick: %lu\r\n", (unsigned long)counter++);
  debug_print(msg);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  debug_println("Smart Apartment Energy Monitor (RTOS + AP) Boot");

  I2C_Scan();               // optional debug
  INA_Init(INA219_ADDR1);
  INA_Init(INA219_ADDR2);

  OLED_Init();
  OLED_ClearBuffer();
  OLED_Update();

  //ESP_StartAccessPoint();   // ESP-01S AP: EnergyMonitor_AP / password123

  /* Initialize RTOS and register tasks */
  RTOS_Init();
  RTOS_AddPeriodicTask(Task_SampleINA, 100);   // 100 ms
  RTOS_AddPeriodicTask(Task_UpdateOLED, 200);  // 200 ms
  RTOS_AddPeriodicTask(Task_Status,    1000);  // 1000 ms

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    RTOS_RunScheduler();  // Run ready tasks (cooperative)
    HAL_Delay(1);         // small idle to avoid tight spin
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* === CubeMX-style init functions below === */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    // Trap here
  }
}

