/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */

typedef struct
{
  uint8_t  bus_ok;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t rx_id;
  uint32_t rx_dlc;
  uint8_t  rx_data[8];
} DbgCan_t;

typedef struct
{
  int16_t  c_x10;
  uint16_t adc_raw;
} DbgTemp_t;

typedef struct
{
  uint8_t  spi_ok;
  uint8_t  pwm3x;
  uint8_t  cfg_ok;
  uint8_t  fault;
  uint16_t reg[7];
} DbgDrv_t;

typedef struct
{
  uint8_t  parity_ok;
  uint8_t  nomag;
  int16_t  deg_x10;
  uint16_t angle;
  uint16_t raw;
  uint32_t count;
  uint32_t err_count;

  uint32_t zero_count;

} DbgEnc_t;

typedef struct
{
  uint8_t  offset_ok;
  int32_t  a_ma;
  int32_t  b_ma;
  int32_t  c_ma;
  int32_t  sum_ma;
  uint16_t raw[3];
  uint16_t offset[3];
} DbgCur_t;

typedef struct
{
  uint8_t  drive_on;
  uint16_t duty_a;
  uint16_t duty_b;
  uint16_t duty_c;
  uint16_t age_ms;
  uint32_t cmd_count;
  uint32_t reject_count;
  uint32_t lost_count;
} DbgCmd_t;

typedef struct
{
  uint8_t  mode;
  uint8_t  pole_pairs;
  uint16_t offset;
  uint16_t e_angle;
  uint16_t e_angle_enc;
  int16_t  id_ma;
  int16_t  iq_ma;
  int16_t  id_ref_ma;
  int16_t  iq_ref_ma;
  int16_t  vd_mv;
  int16_t  vq_mv;
  uint16_t vbus_mv;
  int16_t  rpm;
  int32_t  enc_total;
  uint16_t isr_cycles;
  uint16_t isr_max;
} DbgFoc_t;

typedef struct
{
  int32_t  pos_cnt;
  int16_t  pos_mrad;
  int16_t  vel_x10;
  int16_t  tau_mnm;
  int16_t  p_des_mrad;
  int16_t  v_des_x10;
  int16_t  kp_x10;
  int16_t  kd_x100;
  int16_t  tff_mnm;
  uint32_t cmd_count;
} DbgImp_t;

typedef struct
{
  uint8_t  trip;
  int16_t  therm_pct;
  uint8_t  drv_enabled;
  int32_t  peak_ma;
  int32_t  trip_ma;
  int16_t  ilim_ma;
  uint32_t trip_count;
  uint32_t nfault_count;
  uint32_t clear_count;
} DbgSaf_t;

typedef struct
{
  uint32_t   uptime_ms;
  DbgCan_t   can;
  DbgTemp_t  temp;
  DbgDrv_t   drv;
  DbgEnc_t   enc;
  DbgCur_t   cur;
  DbgCmd_t   cmd;
  DbgSaf_t   saf;
  DbgFoc_t   foc;
  DbgImp_t   imp;
} Dbg_t;

/* USER CODE END PTD */

/* USER CODE BEGIN PD */

#ifndef LED_Pin
#define LED_Pin        GPIO_PIN_15
#define LED_GPIO_Port  GPIOA
#endif

#define NTC_SERIES_R    10000.0f
#define NTC_R25         10000.0f
#define NTC_B           4100.0f
#define NTC_T25_K       298.15f
#define ADC_FULLSCALE   4095.0f
#define TEMP_INVALID    (-32768)

#define DRV_REG_DCR     0x020U
#define DRV_REG_OCP     0x250U
#define DRV_REG_CSA     0x2C3U

#define PWM_HZ          25000U
#define PWM_ARR         3399U

#define ADC_TRIG_CCR4   (PWM_ARR - 150U)

#define CUR_MA_PER_LSB_X100   2014
#define CUR_CAL_SAMPLES       64U

#define CMD_MAGIC           0xA5U
#define CMD_STOP            0U
#define CMD_DRIVE           1U
#define CMD_CLEAR_TRIP      2U

#define CMD_TEST_TRIP       0xF0U
#define PWM_MAX_TEST        425U
#define CMD_TIMEOUT_MS      500U

#define TRIP_NONE           0U
#define TRIP_NFAULT         1U
#define TRIP_OVERCUR        2U
#define TRIP_THERMAL        3U
#define TRIP_ESTOP          4U
#define TRIP_BOARD_HOT      5U

#define CUR_TRIP_MA         10000
#define CUR_TRIP_LSB        ((CUR_TRIP_MA * 100) / CUR_MA_PER_LSB_X100)
#define CUR_TRIP_N          4U

#define I_RATED_MA          2490
#define I_RATED_LSB         ((I_RATED_MA * 100) / CUR_MA_PER_LSB_X100)
#define ISQ_RATED           (3 * I_RATED_LSB * I_RATED_LSB)
#define THERM_TAU_TICKS     100
#define THERM_TRIP_PCT      120
#define THERM_CLEAR_PCT     100

#define THERM_AMB_C         30
#define THERM_RISE_K        68
#define THERM_TREF_C        66
#define CU_ALPHA            0.00393f

#define BOARD_TRIP_C        80
#define BOARD_CLEAR_C       70
#define BOARD_DERATE_C      70
#define WIND_DERATE_C       100
#define BOARD_HOT_N         3U

#define A_PER_LSB           0.02014f
#define MA_PER_A            1000.0f

#define FOC_DT              (1.0f / (float)PWM_HZ)

#define VBUS_MV_DEFAULT     25200U

#define POLE_PAIRS_DEFAULT  21U

#define FOC_VMAX_FRAC       (0.90f / 1.7320508f)

#define FOC_MODE_IDLE       0U
#define FOC_MODE_ALIGN      1U
#define FOC_MODE_OPENLOOP   2U
#define FOC_MODE_CURRENT    3U
#define FOC_CMD_LEARN_OFS   4U
#define FOC_CMD_SET_VBUS    5U
#define FOC_CMD_SET_PP      6U
#define FOC_CMD_SET_GAIN    7U
#define FOC_CMD_SET_DIR     8U
#define FOC_CMD_STEP_V      9U
#define FOC_CMD_STEP_I     10U
#define FOC_CMD_AUTOCAL    11U
#define FOC_CMD_SAVE_CAL   12U
#define FOC_CMD_ARM_TRIPCAP 13U
#define FOC_CMD_SET_CAPSRC 14U

#define FOC_CMD_IMPEDANCE  15U
#define FOC_CMD_ARM_CAP    16U
#define FOC_CMD_SET_NODE   17U
#define FOC_CMD_SET_LIMIT  18U

#define FOC_CMD_SET_TLM    19U
#define FOC_TIMEOUT_MS      500U

#define IMP_LOST_HOLD_MS   2000U
#define IMP_LOST_REL_MS    3000U

#define FOC_IMAX_MA         3000

#define FOC_ALIGN_MV        2000

#define CAP_N               201U

#define CAL_FLASH_PAGE      63U
#define CAL_FLASH_ADDR      0x0801F800U
#define CAL_MAGIC           0x4B524331U

#define COUNTS_PER_REV      16384.0f
#define RAD_PER_COUNT       (6.28318531f / COUNTS_PER_REV)
#define RADS_PER_CPS        RAD_PER_COUNT

#define ENC_V_MAX_RADS      100.0f
#define ENC_MAX_STEP_CNT    ((int32_t)(ENC_V_MAX_RADS * COUNTS_PER_REV / 6.28318531f / (float)PWM_HZ) + 1)
#define ENC_ZERO_NEAR_CNT   (8 * ENC_MAX_STEP_CNT)

#define ENC_ZERO_RUN_MAX    (PWM_HZ)

#define OBS_WN              2513.0f
#define OBS_K1              (2.0f * OBS_WN)
#define OBS_K2              (OBS_WN * OBS_WN)

#define KT_NM_PER_A         0.30f

#define MIT_P_MAX           95.5f
#define MIT_V_MAX           100.0f
#define MIT_KP_MAX          500.0f
#define MIT_KD_MAX          5.0f

#define MIT_T_MAX           3.0f
#define MIT_I_MAX           10.0f

static volatile float   g_p_max        = MIT_P_MAX;
static volatile float   g_v_max        = MIT_V_MAX;
static volatile float   g_kp_max       = MIT_KP_MAX;
static volatile float   g_kd_max       = MIT_KD_MAX;
static volatile float   g_t_max        = MIT_T_MAX;
static volatile float   g_i_max        = MIT_I_MAX;
static volatile float   g_vmax_frac    = FOC_VMAX_FRAC;
static volatile int32_t g_hi_duty_cnt  = (int32_t)((float)PWM_ARR * 0.80f);
static volatile float   g_obs_k1       = OBS_K1;
static volatile float   g_obs_k2       = OBS_K2;
static volatile float   g_extrap_max   = 0.025f;
static volatile float   g_acc_max      = 3000.0f;
static volatile float   g_harm_max_v   = 1.0f;
static volatile float   g_harm_min     = 2.0f;
static volatile float   g_l_h          = 0.00078f;
static volatile float   g_ke           = 0.196f;
static volatile float   g_ntc_b        = NTC_B;
static volatile int32_t g_align_mv     = 2000;

#define CANID_ESTOP         0x080U
#define CANID_IMP(id)       (0x100U + (uint32_t)(id))
#define CANID_CFG(id)       (0x120U + (uint32_t)(id))
#define CANID_DUTY(id)      (0x140U + (uint32_t)(id))
#define CANID_FB(id)        (0x200U + (uint32_t)(id))
#define CANID_TLM(id, n)    (0x300U + ((uint32_t)(id) * 0x10U) + (uint32_t)(n))

#define NODE_ID_DEFAULT     1U
#define NODE_ID_MAX         15U

#define LIM_KP              20.0f
#define LIM_KD              0.15f

#define FOC_MODE_IMPEDANCE  4U

#define FOC_KP_DEFAULT      3.52f
#define FOC_KI_DEFAULT      8040.0f

#define FOC_L_H             0.00078f
#define FOC_KE_VSRAD        0.196f
#define FOC_CMD_SET_DECOUP  20U

#define HARM_MU             0.0005f
#define HARM_MAX_V          1.0f
#define HARM_MIN_RADS       2.0f
#define FOC_CMD_SET_HARM    21U

/* USER CODE END PD */

/* USER CODE BEGIN PM */

/* USER CODE END PM */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_TxHeaderTypeDef TxHeader2;
FDCAN_TxHeaderTypeDef TxHeader3;
FDCAN_TxHeaderTypeDef TxHeader4;
FDCAN_TxHeaderTypeDef TxHeader5;
FDCAN_TxHeaderTypeDef TxHeaderMit;
uint8_t TxData2[8];
uint8_t TxData3[8];
uint8_t TxData4[8];
uint8_t TxData5[8];
uint8_t MitFb[8];
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];

volatile Dbg_t dbg = {0};

DMA_HandleTypeDef hdma_adc1;
static volatile uint16_t adc1_dma[3];

static int32_t cur_off_acc[3];

static uint32_t led_tick = 0;
volatile uint8_t rx_led_hold = 0;

static volatile uint8_t  cur_over_run  = 0U;
static volatile int32_t  cur_peak_lsb  = 0;
static volatile uint8_t  saf_clear_req = 0U;

static volatile int32_t  isq_acc  = 0;
static volatile int32_t  isq_n    = 0;
static int32_t therm_x100000 = 0;
static uint8_t therm_seeded  = 0U;
static uint8_t board_hot_run = 0U;

static float sin_tab[1024];

static volatile uint8_t  foc_mode   = FOC_MODE_IDLE;
static volatile int16_t  foc_p1     = 0;
static volatile int16_t  foc_p2     = 0;
static volatile uint16_t foc_age_ms = 0;
static volatile uint16_t foc_pp     = POLE_PAIRS_DEFAULT;
static volatile uint16_t foc_offset = 0;
static volatile uint16_t foc_vbus_mv = VBUS_MV_DEFAULT;
static volatile uint8_t  foc_learn_req = 0U;

static inline float foc_vmax_v(void)
{
  return (float)foc_vbus_mv * (0.001f * g_vmax_frac);
}

static int64_t  foc_ol_acc = 0;
static int32_t  foc_ol_inc = 0;
static volatile int32_t foc_ol_turns = 0;

static volatile int8_t  foc_dir = -1;
static volatile int16_t foc_dec_x1000 = 0;

static volatile uint8_t foc_harm_on = 0U;
static volatile float   foc_harm_mu = HARM_MU;
static float foc_h6c = 0.0f, foc_h6s = 0.0f;
static float foc_h12c = 0.0f, foc_h12s = 0.0f;
static float    foc_kp = FOC_KP_DEFAULT;
static float    foc_ki = FOC_KI_DEFAULT;

#define FOC_CMD_SET_PARAM   22U

#define FOC_CMD_SAVE_PARAM  23U
#define PRM_IMAX_MA          1U
#define PRM_KT_X1E4          2U
#define PRM_IRATED_MA        3U
#define PRM_THERM_TRIP       4U
#define PRM_THERM_CLEAR      5U
#define PRM_THERM_TAU_S      6U
#define PRM_TIMEOUT_MS       7U
#define PRM_VBUS_MV          8U
#define PRM_LIM_KP_X10       9U
#define PRM_LIM_KD_X1000    10U
#define PRM_HARM_ON         11U
#define PRM_HARM_MU_X1E6    12U
#define PRM_CUR_KP_MV       13U
#define PRM_CUR_KI          14U
#define PRM_DECOUP          15U
#define PRM_ENC_OFFSET      16U

#define PRM_P_MAX_X100      17U
#define PRM_V_MAX_X10       18U
#define PRM_KP_MAX_X10      19U
#define PRM_KD_MAX_X1000    20U
#define PRM_T_MAX_X1000     21U
#define PRM_I_MAX_MA        22U
#define PRM_VMAX_FRAC_X1000 23U
#define PRM_HI_DUTY_X1000   24U
#define PRM_OBS_HZ          25U
#define PRM_EXTRAP_X10      26U
#define PRM_ACC_MAX         27U
#define PRM_HARM_MAX_MV     28U
#define PRM_HARM_MIN_X100   29U
#define PRM_L_UH            30U
#define PRM_KE_X1E4         31U
#define PRM_NTC_B           32U
#define PRM_ALIGN_MV        33U
#define PRM_THERM_AMB_C     34U
#define PRM_THERM_RISE_K    35U
#define PRM_THERM_TREF_C    36U
#define PRM_LOST_HOLD_MS    37U
#define PRM_LOST_REL_MS     38U

#define PRM_ENC_C1          39U
#define PRM_ENC_S1          40U
#define PRM_ENC_C2          41U
#define PRM_ENC_S2          42U

#define PRM_BOARD_TRIP_C    43U
#define PRM_BOARD_CLEAR_C   44U
#define PRM_WIND_DERATE_C   45U
#define PRM_BOARD_DERATE_C  46U
#define PRM_N               47U
#define PRM_EXT_WORDS        8U
#define PRM_IDLE_ONLY(id)   ((((id) >= PRM_P_MAX_X100) && ((id) <= PRM_I_MAX_MA)) || ((id) == PRM_OBS_HZ))

typedef struct { int16_t def; int16_t lo; int16_t hi; } prm_def_t;
static const prm_def_t PRM_TBL[PRM_N] = {
  {0, 0, 0},
  {(int16_t)FOC_IMAX_MA,               500,  8000},
  {(int16_t)(KT_NM_PER_A * 10000.0f), 1000, 10000},
  {(int16_t)I_RATED_MA,                500,  6500},
  {(int16_t)THERM_TRIP_PCT,            101,   250},
  {(int16_t)THERM_CLEAR_PCT,            50,   240},
  {(int16_t)(THERM_TAU_TICKS / 10),      1,   600},
  {(int16_t)FOC_TIMEOUT_MS,            500,  2000},
  {(int16_t)VBUS_MV_DEFAULT,         12000, 30000},
  {(int16_t)(LIM_KP * 10.0f),            0,  1000},
  {(int16_t)(LIM_KD * 1000.0f),          0,  1000},
  {0,                                    0,     1},
  {(int16_t)(HARM_MU * 1000000.0f),  -5000,  5000},
  {(int16_t)(FOC_KP_DEFAULT * 1000.0f),  1, 32767},
  {(int16_t)FOC_KI_DEFAULT,              1, 32767},
  {0,                                    0,  1000},
  {0,                                    0, 16383},
  {(int16_t)(MIT_P_MAX * 100.0f),      100, 32767},
  {(int16_t)(MIT_V_MAX * 10.0f),       100, 10000},
  {(int16_t)(MIT_KP_MAX * 10.0f),       10, 32767},
  {(int16_t)(MIT_KD_MAX * 1000.0f),     10, 32767},
  {(int16_t)(MIT_T_MAX * 1000.0f),     100, 32767},
  {(int16_t)(MIT_I_MAX * 1000.0f),    1000, 32767},
  {900,                                500,   950},
  {800,                                500,   950},
  {400,                                 50,  2000},
  {250,                                 30,  1000},
  {3000,                                 0, 32767},
  {1000,                                 0,  5000},
  {200,                                  0, 10000},
  {780,                                 10, 32767},
  {1960,                                10, 32767},
  {(int16_t)NTC_B,                    1000, 10000},
  {2000,                               200,  6000},
  {THERM_AMB_C,                          0,    60},
  {THERM_RISE_K,                        10,   200},
  {THERM_TREF_C,                        20,   200},
  {(int16_t)IMP_LOST_HOLD_MS,             0, 10000},
  {(int16_t)IMP_LOST_REL_MS,              0, 10000},

  {0,                                 -900,   900},
  {0,                                 -900,   900},
  {0,                                 -900,   900},
  {0,                                 -900,   900},

  {BOARD_TRIP_C,                        40,   120},
  {BOARD_CLEAR_C,                       30,   119},
  {WIND_DERATE_C,                       40,   200},
  {BOARD_DERATE_C,                      30,   120},
};

static volatile int32_t g_imax_ma        = FOC_IMAX_MA;
static volatile float   g_kt             = KT_NM_PER_A;
static volatile int16_t g_irated_ma      = I_RATED_MA;
static volatile int32_t g_isq_rated      = ISQ_RATED;
static volatile int16_t g_therm_trip     = THERM_TRIP_PCT;
static volatile int16_t g_therm_clear_set = THERM_CLEAR_PCT;
static volatile int16_t g_therm_clear    = THERM_CLEAR_PCT;
static volatile int32_t g_therm_tau      = THERM_TAU_TICKS;
static volatile int16_t g_therm_amb      = THERM_AMB_C;
static volatile int16_t g_therm_rise     = THERM_RISE_K;
static volatile int16_t g_therm_tref     = THERM_TREF_C;
static volatile int16_t g_board_trip     = BOARD_TRIP_C;
static volatile int16_t g_board_clear_set = BOARD_CLEAR_C;
static volatile int16_t g_board_clear    = BOARD_CLEAR_C;
static volatile int16_t g_wind_derate    = WIND_DERATE_C;
static volatile int16_t g_board_derate   = BOARD_DERATE_C;

static volatile int32_t g_ilim_ma        = FOC_IMAX_MA;
static volatile uint16_t g_foc_timeout_ms = FOC_TIMEOUT_MS;
static volatile uint16_t g_lost_hold_ms   = IMP_LOST_HOLD_MS;
static volatile uint16_t g_lost_rel_ms    = IMP_LOST_REL_MS;
static volatile float   g_lim_kp         = LIM_KP;
static volatile float   g_lim_kd         = LIM_KD;

static volatile int16_t g_enc_h[4]       = {0, 0, 0, 0};
static volatile float   g_enc_k[4]       = {0.0f, 0.0f, 0.0f, 0.0f};
static volatile uint8_t g_enc_on         = 0U;

static int16_t prm_clamp(uint8_t id, int32_t v)
{
  if (v < PRM_TBL[id].lo) { v = PRM_TBL[id].lo; }
  if (v > PRM_TBL[id].hi) { v = PRM_TBL[id].hi; }
  return (int16_t)v;
}

static void prm_set(uint8_t id, int32_t v)
{
  int16_t x;
  if ((id == 0U) || (id >= PRM_N)) { return; }
  x = prm_clamp(id, v);
  switch (id)
  {
    case PRM_IMAX_MA:      g_imax_ma = x; break;
    case PRM_KT_X1E4:      g_kt = (float)x * 0.0001f; break;
    case PRM_IRATED_MA:
    {
      int32_t lsb = ((int32_t)x * 100) / CUR_MA_PER_LSB_X100;
      g_irated_ma = x;
      g_isq_rated = 3 * lsb * lsb;
      break;
    }
    case PRM_THERM_TRIP:   g_therm_trip = x; break;
    case PRM_THERM_CLEAR:  g_therm_clear_set = x; break;
    case PRM_THERM_TAU_S:  g_therm_tau = (int32_t)x * 10; break;
    case PRM_TIMEOUT_MS:   g_foc_timeout_ms = (uint16_t)x; break;
    case PRM_VBUS_MV:      foc_vbus_mv = (uint16_t)x; break;
    case PRM_LIM_KP_X10:   g_lim_kp = (float)x * 0.1f; break;
    case PRM_LIM_KD_X1000: g_lim_kd = (float)x * 0.001f; break;
    case PRM_HARM_ON:      foc_harm_on = (x != 0) ? 1U : 0U; break;
    case PRM_HARM_MU_X1E6: foc_harm_mu = (float)x * 1e-6f; break;
    case PRM_CUR_KP_MV:    foc_kp = (float)x * 0.001f; break;
    case PRM_CUR_KI:       foc_ki = (float)x; break;
    case PRM_DECOUP:       foc_dec_x1000 = x; break;
    case PRM_ENC_OFFSET:   foc_offset = (uint16_t)(x & 0x3FFF); dbg.foc.offset = foc_offset; break;
    case PRM_P_MAX_X100:   g_p_max = (float)x * 0.01f; break;
    case PRM_V_MAX_X10:    g_v_max = (float)x * 0.1f; break;
    case PRM_KP_MAX_X10:   g_kp_max = (float)x * 0.1f; break;
    case PRM_KD_MAX_X1000: g_kd_max = (float)x * 0.001f; break;
    case PRM_T_MAX_X1000:  g_t_max = (float)x * 0.001f; break;
    case PRM_I_MAX_MA:     g_i_max = (float)x * 0.001f; break;
    case PRM_VMAX_FRAC_X1000: g_vmax_frac = ((float)x * 0.001f) / 1.7320508f; break;
    case PRM_HI_DUTY_X1000:   g_hi_duty_cnt = (int32_t)(((float)PWM_ARR * (float)x) * 0.001f); break;
    case PRM_OBS_HZ:
    {
      float wn = 6.28318531f * (float)x;
      g_obs_k1 = 2.0f * wn;
      g_obs_k2 = wn * wn;
      break;
    }
    case PRM_EXTRAP_X10:   g_extrap_max = (float)x * 0.0001f; break;
    case PRM_ACC_MAX:      g_acc_max = (float)x; break;
    case PRM_HARM_MAX_MV:  g_harm_max_v = (float)x * 0.001f; break;
    case PRM_HARM_MIN_X100: g_harm_min = (float)x * 0.01f; break;
    case PRM_L_UH:         g_l_h = (float)x * 1e-6f; break;
    case PRM_KE_X1E4:      g_ke = (float)x * 0.0001f; break;
    case PRM_NTC_B:        g_ntc_b = (float)x; break;
    case PRM_ALIGN_MV:     g_align_mv = x; break;
    case PRM_THERM_AMB_C:  g_therm_amb = x; break;
    case PRM_THERM_RISE_K: g_therm_rise = x; break;
    case PRM_THERM_TREF_C: g_therm_tref = x; break;
    case PRM_LOST_HOLD_MS: g_lost_hold_ms = (uint16_t)x; break;
    case PRM_LOST_REL_MS:  g_lost_rel_ms = (uint16_t)x; break;
    case PRM_BOARD_TRIP_C:  g_board_trip = x; break;
    case PRM_BOARD_CLEAR_C: g_board_clear_set = x; break;
    case PRM_WIND_DERATE_C: g_wind_derate = x; break;
    case PRM_BOARD_DERATE_C: g_board_derate = x; break;
    case PRM_ENC_C1:
    case PRM_ENC_S1:
    case PRM_ENC_C2:
    case PRM_ENC_S2:
    {
      uint8_t i = (uint8_t)(id - PRM_ENC_C1);
      g_enc_h[i] = x;
      g_enc_k[i] = (float)x * (65536.0f / 3600.0f);
      g_enc_on = ((g_enc_h[0] | g_enc_h[1] | g_enc_h[2] | g_enc_h[3]) != 0) ? 1U : 0U;
      break;
    }
    default: break;
  }
  g_therm_clear = (g_therm_clear_set < g_therm_trip) ? g_therm_clear_set : (int16_t)(g_therm_trip - 1);
  g_board_clear = (g_board_clear_set < g_board_trip) ? g_board_clear_set : (int16_t)(g_board_trip - 1);
}

static int16_t prm_get(uint8_t id)
{
  float f = 0.0f;
  switch (id)
  {
    case PRM_IMAX_MA:      return (int16_t)g_imax_ma;
    case PRM_KT_X1E4:      f = g_kt * 10000.0f; break;
    case PRM_IRATED_MA:    return g_irated_ma;
    case PRM_THERM_TRIP:   return g_therm_trip;
    case PRM_THERM_CLEAR:  return g_therm_clear_set;
    case PRM_THERM_TAU_S:  return (int16_t)(g_therm_tau / 10);
    case PRM_TIMEOUT_MS:   return (int16_t)g_foc_timeout_ms;
    case PRM_VBUS_MV:      return (foc_vbus_mv > 32767U) ? (int16_t)32767 : (int16_t)foc_vbus_mv;
    case PRM_LIM_KP_X10:   f = g_lim_kp * 10.0f; break;
    case PRM_LIM_KD_X1000: f = g_lim_kd * 1000.0f; break;
    case PRM_HARM_ON:      return (int16_t)foc_harm_on;
    case PRM_HARM_MU_X1E6: f = foc_harm_mu * 1000000.0f; break;
    case PRM_CUR_KP_MV:    f = foc_kp * 1000.0f; break;
    case PRM_CUR_KI:       f = foc_ki; break;
    case PRM_DECOUP:       return foc_dec_x1000;
    case PRM_ENC_OFFSET:   return (int16_t)foc_offset;
    case PRM_P_MAX_X100:   f = g_p_max * 100.0f; break;
    case PRM_V_MAX_X10:    f = g_v_max * 10.0f; break;
    case PRM_KP_MAX_X10:   f = g_kp_max * 10.0f; break;
    case PRM_KD_MAX_X1000: f = g_kd_max * 1000.0f; break;
    case PRM_T_MAX_X1000:  f = g_t_max * 1000.0f; break;
    case PRM_I_MAX_MA:     f = g_i_max * 1000.0f; break;
    case PRM_VMAX_FRAC_X1000: f = g_vmax_frac * 1.7320508f * 1000.0f; break;
    case PRM_HI_DUTY_X1000:   f = ((float)g_hi_duty_cnt * 1000.0f) / (float)PWM_ARR; break;
    case PRM_OBS_HZ:       f = (0.5f * g_obs_k1) / 6.28318531f; break;
    case PRM_EXTRAP_X10:   f = g_extrap_max * 10000.0f; break;
    case PRM_ACC_MAX:      f = g_acc_max; break;
    case PRM_HARM_MAX_MV:  f = g_harm_max_v * 1000.0f; break;
    case PRM_HARM_MIN_X100: f = g_harm_min * 100.0f; break;
    case PRM_L_UH:         f = g_l_h * 1e6f; break;
    case PRM_KE_X1E4:      f = g_ke * 10000.0f; break;
    case PRM_NTC_B:        f = g_ntc_b; break;
    case PRM_ALIGN_MV:     return (int16_t)g_align_mv;
    case PRM_THERM_AMB_C:  return g_therm_amb;
    case PRM_THERM_RISE_K: return g_therm_rise;
    case PRM_THERM_TREF_C: return g_therm_tref;
    case PRM_LOST_HOLD_MS: return (int16_t)g_lost_hold_ms;
    case PRM_LOST_REL_MS:  return (int16_t)g_lost_rel_ms;
    case PRM_BOARD_TRIP_C:  return g_board_trip;
    case PRM_BOARD_CLEAR_C: return g_board_clear_set;
    case PRM_WIND_DERATE_C: return g_wind_derate;
    case PRM_BOARD_DERATE_C: return g_board_derate;
    case PRM_ENC_C1:
    case PRM_ENC_S1:
    case PRM_ENC_C2:
    case PRM_ENC_S2:       return g_enc_h[id - PRM_ENC_C1];
    default:               return 0;
  }
  if (f >  32767.0f) { f =  32767.0f; }
  if (f < -32768.0f) { f = -32768.0f; }
  return (int16_t)((f >= 0.0f) ? (f + 0.5f) : (f - 0.5f));
}

static void prm_defaults_as(uint8_t skip_idle_only)
{
  uint8_t id;
  for (id = 1U; id < PRM_N; id++)
  {
    if (id == PRM_ENC_OFFSET) { continue; }
    if ((skip_idle_only != 0U) && PRM_IDLE_ONLY(id)) { continue; }
    prm_set(id, PRM_TBL[id].def);
  }
}
static void prm_defaults(void)
{
  prm_defaults_as(0U);
}
static float    foc_int_d = 0.0f;
static float    foc_int_q = 0.0f;

static uint16_t enc_prev = 0U;
static uint8_t  enc_prev_ok = 0U;
static volatile int32_t enc_total = 0;

static float    obs_x    = 0.0f;
static float    obs_v    = 0.0f;
static int32_t  obs_prev = 0;

static volatile int32_t pos_zero = 0;
static volatile uint8_t pos_zero_req = 0U;

static volatile uint8_t node_id = NODE_ID_DEFAULT;
static volatile int32_t lim_lo_cnt = 0;
static volatile int32_t lim_hi_cnt = 0;
static volatile uint8_t tlm_div = 1U;

static volatile float imp_p_des = 0.0f;
static volatile uint8_t  imp_lost    = 0U;
static volatile float    imp_lost_k  = 1.0f;
static uint16_t          imp_lost_ms = 0U;
static volatile float imp_v_des = 0.0f;
static volatile float imp_kp    = 0.0f;
static volatile float imp_kd    = 0.0f;
static volatile float imp_tff   = 0.0f;

#define IMP_EXTRAP_MIN_S    0.002f
#define IMP_EXTRAP_MAX_S    0.025f
#define IMP_ACC_MAX         3000.0f
static volatile float imp_p_ext   = 0.0f;
static volatile float imp_v_ext   = 0.0f;
static volatile float imp_a_est   = 0.0f;
static volatile float imp_v_last  = 0.0f;
static volatile float imp_ext_t   = 0.0f;
static volatile float imp_ext_max = IMP_EXTRAP_MAX_S;

static volatile uint8_t  cap_src = 0U;
static volatile uint16_t cap_div = 1U;
static volatile uint16_t cap_cnt = 0U;
static int16_t  cap_buf[CAP_N];
static volatile uint16_t cap_idx   = 0U;
static volatile uint8_t  cap_state = 0U;

static volatile uint8_t  foc_coast      = 0U;
static volatile uint16_t foc_coast_ms   = 0U;
static volatile uint8_t foc_autocal_req = 0U;
static volatile uint8_t cal_save_req    = 0U;
static volatile uint8_t prm_save_req    = 0U;
static volatile uint8_t prm_save_count  = 0U;
static volatile uint8_t cal_valid       = 0U;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void read_temperature(void)
{
  uint32_t raw;

  if (HAL_ADC_Start(&hadc2) != HAL_OK)
  {
    return;
  }
  if (HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc2);
    dbg.temp.c_x10 = TEMP_INVALID;
    return;
  }
  raw = HAL_ADC_GetValue(&hadc2);
  HAL_ADC_Stop(&hadc2);

  dbg.temp.adc_raw = (uint16_t)raw;

  if ((raw == 0U) || (raw >= (uint32_t)ADC_FULLSCALE))
  {
    dbg.temp.c_x10 = TEMP_INVALID;
    return;
  }

  float r_ntc = NTC_SERIES_R * (float)raw / (ADC_FULLSCALE - (float)raw);

  float inv_t = (1.0f / NTC_T25_K) + (logf(r_ntc / NTC_R25) / g_ntc_b);
  float t_c   = (1.0f / inv_t) - 273.15f;

  dbg.temp.c_x10 = (int16_t)(t_c * 10.0f);
}

static uint16_t drv_xfer(uint16_t frame)
{
  uint16_t rx = 0;

  HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&frame, (uint8_t *)&rx, 1, 10);
  HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_SET);

  return rx;
}

static uint16_t drv_read(uint8_t addr)
{
  return drv_xfer((uint16_t)(0x8000U | ((uint16_t)(addr & 0x0FU) << 11))) & 0x07FFU;
}

static void drv_write(uint8_t addr, uint16_t data)
{
  (void)drv_xfer((uint16_t)(((uint16_t)(addr & 0x0FU) << 11) | (data & 0x07FFU)));
}

static inline uint16_t spi3_xfer16(uint16_t tx)
{
  uint32_t guard = 4000U;

  *(volatile uint16_t *)&SPI3->DR = tx;
  while ((SPI3->SR & SPI_SR_RXNE) == 0U)
  {

    if (--guard == 0U)
    {
      return 0xFFFFU;
    }
  }
  return *(volatile uint16_t *)&SPI3->DR;
}

static uint16_t mt6816_read_raw(void)
{
  uint16_t hi, lo;

  HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET);
  hi = spi3_xfer16(0x8300U);
  HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);

  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();

  HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET);
  lo = spi3_xfer16(0x8400U);
  HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);

  return (uint16_t)(((hi & 0x00FFU) << 8) | (lo & 0x00FFU));
}

static void enc_sample(void)
{
  uint16_t raw = mt6816_read_raw();
  uint16_t ang = (uint16_t)(raw >> 2);
  static uint16_t enc_zero_run = 0U;

  dbg.enc.raw = raw;

  if (raw == 0xFFFFU)
  {
    dbg.enc.parity_ok = 0U;
    dbg.enc.err_count++;
    return;
  }

  if (raw == 0x0000U)
  {
    uint8_t plausible = 1U;

    dbg.enc.zero_count++;

    if (enc_prev_ok != 0U)
    {
      int32_t d = (int32_t)enc_prev;
      if (d > 8192) { d -= 16384; }
      if (d < 0)    { d = -d; }
      if (d > ENC_ZERO_NEAR_CNT) { plausible = 0U; }
    }

    if (plausible == 0U)
    {

      enc_zero_run = 0U;
      dbg.enc.parity_ok = 0U;
      dbg.enc.err_count++;
    }
    else if (++enc_zero_run >= (uint16_t)ENC_ZERO_RUN_MAX)
    {

      enc_zero_run = 0U;
      dbg.enc.parity_ok = 0U;
      dbg.enc.err_count++;
    }

    return;
  }
  enc_zero_run = 0U;

  if ((__builtin_popcount(raw) & 1) != 0)
  {
    dbg.enc.parity_ok = 0U;
    dbg.enc.err_count++;
    return;
  }
  dbg.enc.parity_ok = 1U;

  dbg.enc.angle = ang;
  dbg.enc.nomag = (uint8_t)((raw >> 1) & 0x01U);

  if (enc_prev_ok != 0U)
  {
    int32_t diff = (int32_t)ang - (int32_t)enc_prev;
    if (diff >  8192) { diff -= 16384; }
    if (diff < -8192) { diff += 16384; }

    enc_total += (foc_dir >= 0) ? diff : -diff;
  }
  enc_prev    = ang;
  enc_prev_ok = 1U;

  dbg.enc.count++;
}

static uint8_t can_add(FDCAN_TxHeaderTypeDef *h, uint8_t *d)
{
  uint32_t pri = __get_PRIMASK();
  uint8_t  ok;

  __disable_irq();
  ok = (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, h, d) == HAL_OK) ? 1U : 0U;
  __set_PRIMASK(pri);

  return ok;
}

static void can_filters_update(void)
{
  FDCAN_FilterTypeDef f;
  f.IdType       = FDCAN_STANDARD_ID;
  f.FilterType   = FDCAN_FILTER_DUAL;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterIndex  = 0;
  f.FilterID1    = CANID_ESTOP;
  f.FilterID2    = CANID_IMP(node_id);
  (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
  f.FilterIndex  = 1;
  f.FilterID1    = CANID_CFG(node_id);
  f.FilterID2    = CANID_DUTY(node_id);
  (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
}

static void can_ids_update(void)
{
  TxHeader.Identifier    = CANID_TLM(node_id, 0);
  TxHeader2.Identifier   = CANID_TLM(node_id, 1);
  TxHeader3.Identifier   = CANID_TLM(node_id, 2);
  TxHeader4.Identifier   = CANID_TLM(node_id, 3);
  TxHeader5.Identifier   = CANID_TLM(node_id, 4);
  TxHeaderMit.Identifier = CANID_FB(node_id);
  can_filters_update();
}

#define CAN_TX_KEEP_FOR_REPLY  1U
static uint8_t can_send(FDCAN_TxHeaderTypeDef *h, uint8_t *d)
{
  uint32_t t0 = HAL_GetTick();

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) <= CAN_TX_KEEP_FOR_REPLY)
  {
    if ((HAL_GetTick() - t0) > 5U)
    {
      return 0U;
    }
  }
  return can_add(h, d);
}

static void pwm_set(uint32_t a, uint32_t b, uint32_t c)
{
  if (a > PWM_ARR) { a = PWM_ARR; }
  if (b > PWM_ARR) { b = PWM_ARR; }
  if (c > PWM_ARR) { c = PWM_ARR; }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, a);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, b);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, c);
}

static void read_currents(void)
{

  uint32_t r[3];

  r[0] = adc1_dma[0];
  r[1] = adc1_dma[1];
  r[2] = adc1_dma[2];

  dbg.cur.raw[0] = (uint16_t)r[0];
  dbg.cur.raw[1] = (uint16_t)r[1];
  dbg.cur.raw[2] = (uint16_t)r[2];

  if (dbg.cur.offset_ok == 0U)
  {
    return;
  }

  if (dbg.saf.drv_enabled == 0U)
  {
    dbg.cur.a_ma   = 0;
    dbg.cur.b_ma   = 0;
    dbg.cur.c_ma   = 0;
    dbg.cur.sum_ma = 0;
    return;
  }

  dbg.cur.a_ma = ((int32_t)r[0] - (int32_t)dbg.cur.offset[0]) * CUR_MA_PER_LSB_X100 / 100;
  dbg.cur.b_ma = ((int32_t)r[1] - (int32_t)dbg.cur.offset[1]) * CUR_MA_PER_LSB_X100 / 100;
  dbg.cur.c_ma = ((int32_t)r[2] - (int32_t)dbg.cur.offset[2]) * CUR_MA_PER_LSB_X100 / 100;
  dbg.cur.sum_ma = dbg.cur.a_ma + dbg.cur.b_ma + dbg.cur.c_ma;
}

static void calibrate_current_offset(void)
{
  uint32_t acc[3] = {0U, 0U, 0U};

  for (uint32_t n = 0U; n < CUR_CAL_SAMPLES; n++)
  {
    read_currents();
    acc[0] += dbg.cur.raw[0];
    acc[1] += dbg.cur.raw[1];
    acc[2] += dbg.cur.raw[2];
    HAL_Delay(1);
  }

  for (uint8_t i = 0U; i < 3U; i++)
  {
    dbg.cur.offset[i] = (uint16_t)(acc[i] / CUR_CAL_SAMPLES);
    cur_off_acc[i]    = (int32_t)dbg.cur.offset[i] * 256;
  }
  dbg.cur.offset_ok = 1U;
}

static void track_current_offset(void)
{

  if ((dbg.cur.offset_ok == 0U) ||
      (dbg.cmd.drive_on != 0U) ||
      (foc_mode != FOC_MODE_IDLE) ||
      (foc_coast != 0U) ||
      (dbg.saf.trip != TRIP_NONE) ||
      (dbg.saf.drv_enabled == 0U))
  {
    return;
  }

  for (uint8_t i = 0U; i < 3U; i++)
  {
    int32_t target = (int32_t)dbg.cur.raw[i] * 256;
    cur_off_acc[i] += (target - cur_off_acc[i]) / 64;
    dbg.cur.offset[i] = (uint16_t)(cur_off_acc[i] / 256);
  }
}

static void foc_build_tables(void)
{
  for (uint32_t i = 0U; i < 1024U; i++)
  {
    sin_tab[i] = sinf((float)i * (2.0f * 3.14159265f / 1024.0f));
  }
}

static inline float fsin_u16(uint16_t a)
{
  return sin_tab[a >> 6];
}

static inline float fcos_u16(uint16_t a)
{
  return sin_tab[(uint16_t)(((a >> 6) + 256U) & 1023U)];
}

static inline int32_t enc_corr_u16(uint16_t mech14)
{
  uint16_t a1 = (uint16_t)(mech14 << 2);
  uint16_t a2 = (uint16_t)(mech14 << 3);
  float d = (g_enc_k[0] * fcos_u16(a1)) + (g_enc_k[1] * fsin_u16(a1))
          + (g_enc_k[2] * fcos_u16(a2)) + (g_enc_k[3] * fsin_u16(a2));

  return (int32_t)d;
}

static inline uint16_t foc_elec_angle(uint16_t mech14)
{
  uint32_t rel = (uint32_t)((mech14 - foc_offset) & 0x3FFFU);
  uint16_t e   = (uint16_t)((rel * foc_pp) << 2);

  if (foc_dir < 0) { e = (uint16_t)(0U - (uint32_t)e); }

  if (g_enc_on != 0U)
  {
    e = (uint16_t)((uint32_t)e + (uint32_t)enc_corr_u16(mech14));
  }
  return e;
}

static uint16_t foc_offset_at(uint16_t mech14)
{
  int32_t k = 0;

  if ((g_enc_on != 0U) && (foc_pp != 0U))
  {
    float c = (float)enc_corr_u16(mech14) / (4.0f * (float)foc_pp * (float)foc_dir);
    k = (int32_t)((c >= 0.0f) ? (c + 0.5f) : (c - 0.5f));
  }
  return (uint16_t)(((int32_t)mech14 + k) & 0x3FFF);
}

static inline void foc_reset(void)
{
  foc_int_d = 0.0f;
  foc_int_q = 0.0f;
}

static inline void obs_step(void)
{
  int32_t now = enc_total;
  int32_t d   = now - obs_prev;

  obs_prev = now;

  obs_x += ((obs_v - (g_obs_k1 * obs_x)) * FOC_DT) - (float)d;
  obs_v += -(g_obs_k2 * obs_x) * FOC_DT;
}

static inline float pos_rad(void)
{
  return (float)(enc_total - pos_zero) * RAD_PER_COUNT;
}

static inline float vel_rads(void)
{
  return obs_v * RADS_PER_CPS;
}

static uint16_t mit_pack(float x, float xmin, float xmax, uint8_t bits)
{
  float span = xmax - xmin;
  float v;

  if (x < xmin) { x = xmin; }
  if (x > xmax) { x = xmax; }

  v = (x - xmin) * (float)((1U << bits) - 1U) / span;
  return (uint16_t)v;
}

static float mit_unpack(uint16_t v, float xmin, float xmax, uint8_t bits)
{
  float span = xmax - xmin;

  return ((float)v * span / (float)((1U << bits) - 1U)) + xmin;
}

static void mit_send_feedback(void)
{
  uint16_t p_i = mit_pack(pos_rad(),  -g_p_max, g_p_max, 16);
  uint16_t v_i = mit_pack(vel_rads(), -g_v_max, g_v_max, 12);
  uint16_t i_i = mit_pack((float)dbg.foc.iq_ma * 0.001f, -g_i_max, g_i_max, 12);

  MitFb[0] = (uint8_t)(node_id | ((dbg.saf.trip != TRIP_NONE) ? 0x80U : 0x00U));
  MitFb[1] = (uint8_t)(p_i >> 8);
  MitFb[2] = (uint8_t)(p_i);
  MitFb[3] = (uint8_t)(v_i >> 4);
  MitFb[4] = (uint8_t)(((v_i & 0x0FU) << 4) | (i_i >> 8));
  MitFb[5] = (uint8_t)(i_i);
  MitFb[6] = 0U;
  MitFb[7] = 0U;

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U)
  {
    (void)can_add(&TxHeaderMit, MitFb);
  }
}

static void enc_and_state(void)
{
  enc_sample();
  obs_step();
  dbg.foc.e_angle_enc = foc_elec_angle(dbg.enc.angle);
  dbg.foc.enc_total   = enc_total;

  if (pos_zero_req != 0U)
  {
    pos_zero_req = 0U;
    pos_zero     = enc_total;
  }

  {
    float p  = pos_rad();
    float v  = vel_rads();
    float pm = p * 1000.0f;

    if (pm >  32000.0f) { pm =  32000.0f; }
    if (pm < -32000.0f) { pm = -32000.0f; }

    dbg.imp.pos_cnt  = enc_total - pos_zero;
    dbg.imp.pos_mrad = (int16_t)pm;
    dbg.imp.vel_x10  = (int16_t)(v * 10.0f);
    dbg.foc.rpm      = (int16_t)(v * (60.0f / 6.28318531f));
  }
}

static uint8_t foc_hi_phase = 3U;

#define FOC_HI_DUTY_CNT   ((int32_t)((float)PWM_ARR * 0.80f))

static void foc_step(const int32_t *dlsb)
{
  uint8_t  mode = foc_mode;
  int16_t  ref_q_ma = foc_p1;
  int16_t  ref_d_ma = foc_p2;
  uint16_t th;
  float    s, c;
  float    ia, ib, ic, ialpha, ibeta, id, iq;
  float    vd = 0.0f, vq = 0.0f;
  float    valpha, vbeta, va, vb, vc, vhi, vlo, vmid, k;
  float    mag, vmax;
  int32_t  da, db, dc;

  if (foc_learn_req != 0U)
  {
    foc_learn_req = 0U;
    foc_offset    = foc_offset_at(dbg.enc.angle);
    dbg.foc.offset = foc_offset;
  }

  if (mode == FOC_MODE_IDLE)
  {

    dbg.imp.tau_mnm = 0;

    if (foc_coast != 0U)
    {

      mode     = FOC_MODE_CURRENT;
      ref_q_ma = 0;
      ref_d_ma = 0;
    }
    else
    {
      pwm_set(0U, 0U, 0U);
      dbg.foc.vd_mv = 0;
      dbg.foc.vq_mv = 0;
      foc_hi_phase = 3U;
      return;
    }
  }

  if (mode == FOC_MODE_IMPEDANCE)
  {
    float p    = pos_rad();
    float v    = vel_rads();
    float pdes;
    float vdes;
    float tau;
    float iq_a;
    float kp_eff;
    float tff_eff;

    if (imp_ext_t < imp_ext_max)
    {
      imp_ext_t += FOC_DT;
      imp_v_ext += imp_a_est * FOC_DT;
      imp_p_ext += (imp_v_des + imp_v_ext) * FOC_DT;
    }
    pdes = imp_p_des + imp_p_ext;
    vdes = imp_v_des + imp_v_ext;
    {
      float k = imp_lost_k;
      kp_eff  = imp_kp * k;
      tff_eff = imp_tff * k;
      if (imp_lost != 0U) { vdes = 0.0f; }
    }

    if (lim_hi_cnt > lim_lo_cnt)
    {
      float lo = (float)lim_lo_cnt * RAD_PER_COUNT;
      float hi = (float)lim_hi_cnt * RAD_PER_COUNT;

      if (pdes < lo) { pdes = lo; }
      if (pdes > hi) { pdes = hi; }

      tau = (kp_eff * (pdes - p)) + (imp_kd * (vdes - v)) + tff_eff;

      if (p > hi)
      {
        tau += (-g_lim_kp * (p - hi)) - (g_lim_kd * v);
      }
      else if (p < lo)
      {
        tau += (-g_lim_kp * (p - lo)) - (g_lim_kd * v);
      }
    }
    else
    {
      tau = (kp_eff * (pdes - p)) + (imp_kd * (vdes - v)) + tff_eff;
    }

    if (tau >  g_t_max) { tau =  g_t_max; }
    if (tau < -g_t_max) { tau = -g_t_max; }

    iq_a = tau / g_kt;

    dbg.imp.tau_mnm = (int16_t)(tau * 1000.0f);
    ref_q_ma = (int16_t)(iq_a * 1000.0f);
    ref_d_ma = 0;

    if (ref_q_ma >  g_imax_ma) { ref_q_ma = (int16_t) g_imax_ma; }
    if (ref_q_ma < -g_imax_ma) { ref_q_ma = (int16_t)-g_imax_ma; }

    mode = FOC_MODE_CURRENT;
  }

  switch (mode)
  {
    case FOC_MODE_ALIGN:
      th = (uint16_t)foc_p2;
      break;

    case FOC_MODE_OPENLOOP:
      foc_ol_acc  += (int64_t)foc_ol_inc;
      th           = (uint16_t)((uint64_t)foc_ol_acc >> 16);

      foc_ol_turns = (int32_t)(foc_ol_acc >> 24);
      break;

    case FOC_MODE_CURRENT:
    default:
      th = dbg.foc.e_angle_enc;
      break;
  }
  dbg.foc.e_angle = th;

  s = fsin_u16(th);
  c = fcos_u16(th);

  ia = (float)dlsb[0] * A_PER_LSB;
  ib = (float)dlsb[1] * A_PER_LSB;
  ic = (float)dlsb[2] * A_PER_LSB;

  if (foc_hi_phase == 0U)      { ia = -(ib + ic); }
  else if (foc_hi_phase == 1U) { ib = -(ia + ic); }
  else if (foc_hi_phase == 2U) { ic = -(ia + ib); }

  ialpha = ((2.0f * ia) - ib - ic) * (1.0f / 3.0f);
  ibeta  = (ib - ic) * (1.0f / 1.7320508f);

  id =  (ialpha * c) + (ibeta * s);
  iq = -(ialpha * s) + (ibeta * c);

  dbg.foc.id_ma = (int16_t)(id * MA_PER_A);
  dbg.foc.iq_ma = (int16_t)(iq * MA_PER_A);

  if (cap_state == 1U)
  {
    cap_cnt++;
    if (cap_cnt >= cap_div)
    {
      int16_t v;

      cap_cnt = 0U;
      switch (cap_src)
      {
        case 1U:  v = dbg.foc.iq_ma;    break;
        case 3U:  v = dbg.imp.vel_x10;  break;
        case 4U:  v = dbg.imp.pos_mrad; break;
        case 5U:  v = dbg.imp.tau_mnm;  break;
        default:  v = dbg.foc.id_ma;    break;
      }
      cap_buf[cap_idx] = v;
      cap_idx++;
      if (cap_idx >= CAP_N)
      {
        cap_state = 2U;
      }
    }
  }

  switch (mode)
  {
    case FOC_MODE_ALIGN:
      vd = (float)foc_p1 * 0.001f;
      vq = 0.0f;
      break;

    case FOC_MODE_OPENLOOP:
      vd = 0.0f;
      vq = (float)foc_p1 * 0.001f;
      break;

    case FOC_MODE_CURRENT:
    default:
    {
      {
        int32_t L = g_ilim_ma;
        if (ref_q_ma >  L) { ref_q_ma = (int16_t) L; }
        if (ref_q_ma < -L) { ref_q_ma = (int16_t)-L; }
        if (ref_d_ma >  L) { ref_d_ma = (int16_t) L; }
        if (ref_d_ma < -L) { ref_d_ma = (int16_t)-L; }
      }
      float ref_q = (float)ref_q_ma * 0.001f;
      float ref_d = (float)ref_d_ma * 0.001f;
      float ed = ref_d - id;
      float eq = ref_q - iq;

      dbg.foc.id_ref_ma = ref_d_ma;
      dbg.foc.iq_ref_ma = ref_q_ma;

      foc_int_d += foc_ki * ed * FOC_DT;
      foc_int_q += foc_ki * eq * FOC_DT;

      vmax = foc_vmax_v();
      if (foc_int_d >  vmax) { foc_int_d =  vmax; }
      if (foc_int_d < -vmax) { foc_int_d = -vmax; }
      if (foc_int_q >  vmax) { foc_int_q =  vmax; }
      if (foc_int_q < -vmax) { foc_int_q = -vmax; }

      vd = (foc_kp * ed) + foc_int_d;
      vq = (foc_kp * eq) + foc_int_q;

      if (foc_harm_on != 0U)
      {
        uint16_t th6  = (uint16_t)(th * 6U);
        uint16_t th12 = (uint16_t)(th * 12U);
        float c6  = fcos_u16(th6),  s6  = fsin_u16(th6);
        float c12 = fcos_u16(th12), s12 = fsin_u16(th12);
        float verr = iq - ref_q;
        float sp   = vel_rads();

        if ((sp > g_harm_min) || (sp < -g_harm_min))
        {
          foc_h6c  += foc_harm_mu * verr * c6;
          foc_h6s  += foc_harm_mu * verr * s6;
          foc_h12c += foc_harm_mu * verr * c12;
          foc_h12s += foc_harm_mu * verr * s12;

          if (foc_h6c  >  g_harm_max_v) { foc_h6c  =  g_harm_max_v; }
          if (foc_h6c  < -g_harm_max_v) { foc_h6c  = -g_harm_max_v; }
          if (foc_h6s  >  g_harm_max_v) { foc_h6s  =  g_harm_max_v; }
          if (foc_h6s  < -g_harm_max_v) { foc_h6s  = -g_harm_max_v; }
          if (foc_h12c >  g_harm_max_v) { foc_h12c =  g_harm_max_v; }
          if (foc_h12c < -g_harm_max_v) { foc_h12c = -g_harm_max_v; }
          if (foc_h12s >  g_harm_max_v) { foc_h12s =  g_harm_max_v; }
          if (foc_h12s < -g_harm_max_v) { foc_h12s = -g_harm_max_v; }
        }

        vq -= (foc_h6c * c6) + (foc_h6s * s6)
            + (foc_h12c * c12) + (foc_h12s * s12);
      }

      if (foc_dec_x1000 != 0)
      {
        float we = vel_rads() * (float)foc_pp;
        float kk = (float)foc_dec_x1000 * 0.001f;
        float psi = g_ke / (float)foc_pp;

        vd += -kk * we * g_l_h * iq;
        vq +=  kk * we * ((g_l_h * id) + psi);
      }
      break;
    }
  }

  vmax = foc_vmax_v();
  mag  = sqrtf((vd * vd) + (vq * vq));
  if (mag > vmax)
  {
    float sc = vmax / mag;
    vd *= sc;
    vq *= sc;
  }

  dbg.foc.vd_mv = (int16_t)(vd * 1000.0f);
  dbg.foc.vq_mv = (int16_t)(vq * 1000.0f);

  valpha = (vd * c) - (vq * s);
  vbeta  = (vd * s) + (vq * c);

  va = valpha;
  vb = (-0.5f * valpha) + (0.8660254f * vbeta);
  vc = (-0.5f * valpha) - (0.8660254f * vbeta);

  vhi = va; if (vb > vhi) { vhi = vb; } if (vc > vhi) { vhi = vc; }
  vlo = va; if (vb < vlo) { vlo = vb; } if (vc < vlo) { vlo = vc; }
  vmid = 0.5f * (vhi + vlo);

  k = (float)PWM_ARR / ((float)foc_vbus_mv * 0.001f);

  da = (int32_t)((0.5f * (float)PWM_ARR) + ((va - vmid) * k));
  db = (int32_t)((0.5f * (float)PWM_ARR) + ((vb - vmid) * k));
  dc = (int32_t)((0.5f * (float)PWM_ARR) + ((vc - vmid) * k));

  {
    int32_t dm = da; uint8_t hp = 0U;
    if (db > dm) { dm = db; hp = 1U; }
    if (dc > dm) { dm = dc; hp = 2U; }
    foc_hi_phase = (dm > g_hi_duty_cnt) ? hp : 3U;
  }

  if (da < 0) { da = 0; } if (da > (int32_t)ADC_TRIG_CCR4) { da = (int32_t)ADC_TRIG_CCR4; }
  if (db < 0) { db = 0; } if (db > (int32_t)ADC_TRIG_CCR4) { db = (int32_t)ADC_TRIG_CCR4; }
  if (dc < 0) { dc = 0; } if (dc > (int32_t)ADC_TRIG_CCR4) { dc = (int32_t)ADC_TRIG_CCR4; }

  pwm_set((uint32_t)da, (uint32_t)db, (uint32_t)dc);
}

static uint8_t cal_load(void)
{
  uint64_t dw = *(volatile uint64_t *)CAL_FLASH_ADDR;

  if ((uint32_t)dw != CAL_MAGIC)
  {
    return 0U;
  }

  foc_offset = (uint16_t)((dw >> 32) & 0x3FFFU);
  foc_pp     = (uint16_t)((dw >> 46) & 0x007FU);
  foc_dir    = (((dw >> 53) & 1U) != 0U) ? (int8_t)-1 : (int8_t)1;

  {
    uint8_t n = (uint8_t)((dw >> 54) & 0x0FU);
    if ((n >= 1U) && (n <= NODE_ID_MAX)) { node_id = n; }
  }

  {
    uint64_t dw1 = *(volatile uint64_t *)(CAL_FLASH_ADDR + 8U);
    lim_lo_cnt = (int32_t)(uint32_t)(dw1 & 0xFFFFFFFFU);
    lim_hi_cnt = (int32_t)(uint32_t)(dw1 >> 32);
  }

  {
    uint64_t dw2   = *(volatile uint64_t *)(CAL_FLASH_ADDR + 16U);
    uint16_t kp_mv = (uint16_t)(dw2 & 0xFFFFU);
    uint16_t ki_v  = (uint16_t)((dw2 >> 16) & 0xFFFFU);
    uint16_t dec   = (uint16_t)((dw2 >> 32) & 0xFFFFU);

    if ((kp_mv != 0xFFFFU) && (kp_mv != 0U)) { foc_kp = (float)kp_mv * 0.001f; }
    if ((ki_v  != 0xFFFFU) && (ki_v  != 0U)) { foc_ki = (float)ki_v; }
    if (dec <= 1000U) { foc_dec_x1000 = (int16_t)dec; }
  }

  {
    uint8_t w, k;
    for (w = 0U; w < 3U; w++)
    {
      uint64_t d = *(volatile uint64_t *)(CAL_FLASH_ADDR + 24U + (8U * (uint32_t)w));
      for (k = 0U; k < 4U; k++)
      {
        uint16_t raw = (uint16_t)((d >> (16U * k)) & 0xFFFFU);
        if (raw != 0xFFFFU) { prm_set((uint8_t)(1U + (4U * w) + k), (int16_t)raw); }
      }
    }
  }
  {
    uint8_t w, k;
    for (w = 0U; w < PRM_EXT_WORDS; w++)
    {
      uint64_t d = *(volatile uint64_t *)(CAL_FLASH_ADDR + 48U + (8U * (uint32_t)w));
      for (k = 0U; k < 4U; k++)
      {
        uint8_t id = (uint8_t)(PRM_P_MAX_X100 + (4U * w) + k);
        uint16_t raw = (uint16_t)((d >> (16U * k)) & 0xFFFFU);
        if ((id < PRM_N) && (raw != 0xFFFFU)) { prm_set(id, (int16_t)raw); }
      }
    }
  }
  return 1U;
}

static uint8_t cal_save_as(uint8_t params_only);
static uint8_t cal_save(void)
{
  return cal_save_as(0U);
}

static uint8_t cal_save_as(uint8_t params_only)
{
  FLASH_EraseInitTypeDef er = {0};
  uint32_t page_err = 0U;
  uint8_t  ok = 0U;
  uint64_t dw  = (uint64_t)CAL_MAGIC
               | ((uint64_t)(foc_offset & 0x3FFFU) << 32)
               | ((uint64_t)(foc_pp     & 0x007FU) << 46)
               | ((uint64_t)((foc_dir < 0) ? 1U : 0U) << 53)
               | ((uint64_t)(node_id    & 0x000FU) << 54);
  uint64_t dw1 = ((uint64_t)(uint32_t)lim_lo_cnt)
               | ((uint64_t)(uint32_t)lim_hi_cnt << 32);
  if (params_only != 0U)
  {
    uint64_t old0 = *(volatile uint64_t *)CAL_FLASH_ADDR;
    if ((uint32_t)old0 != CAL_MAGIC) { return 0U; }
    dw  = (old0 & ~((uint64_t)0x3FFFU << 32)) | ((uint64_t)(foc_offset & 0x3FFFU) << 32);
    dw1 = *(volatile uint64_t *)(CAL_FLASH_ADDR + 8U);
  }
  uint64_t dw2 = ((uint64_t)(uint16_t)(int16_t)(foc_kp * 1000.0f))
               | ((uint64_t)(uint16_t)(int16_t)foc_ki << 16)
               | ((uint64_t)(uint16_t)foc_dec_x1000 << 32);
  uint64_t dwp[3];
  uint64_t dwx[PRM_EXT_WORDS];
  {
    uint8_t w, k;
    for (w = 0U; w < PRM_EXT_WORDS; w++)
    {
      dwx[w] = 0U;
      for (k = 0U; k < 4U; k++)
      {
        uint8_t id = (uint8_t)(PRM_P_MAX_X100 + (4U * w) + k);
        uint16_t v = (id < PRM_N) ? (uint16_t)prm_get(id) : 0xFFFFU;
        dwx[w] |= ((uint64_t)v) << (16U * k);
      }
    }
  }
  {
    uint8_t w, k;
    for (w = 0U; w < 3U; w++)
    {
      dwp[w] = 0U;
      for (k = 0U; k < 4U; k++)
      {
        dwp[w] |= ((uint64_t)(uint16_t)prm_get((uint8_t)(1U + (4U * w) + k))) << (16U * k);
      }
    }
  }

  HAL_FLASH_Unlock();

  er.TypeErase = FLASH_TYPEERASE_PAGES;
  er.Banks     = FLASH_BANK_1;
  er.Page      = CAL_FLASH_PAGE;
  er.NbPages   = 1U;

  if (HAL_FLASHEx_Erase(&er, &page_err) == HAL_OK)
  {
    if ((HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR, dw) == HAL_OK) &&
        (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR + 8U, dw1) == HAL_OK) &&
        (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR + 16U, dw2) == HAL_OK) &&
        (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR + 24U, dwp[0]) == HAL_OK) &&
        (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR + 32U, dwp[1]) == HAL_OK) &&
        (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR + 40U, dwp[2]) == HAL_OK))
    {
      uint8_t w;
      ok = 1U;
      for (w = 0U; w < PRM_EXT_WORDS; w++)
      {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR + 48U + (8U * (uint32_t)w), dwx[w]) != HAL_OK)
        {
          ok = 0U;
          break;
        }
      }
    }
  }

  HAL_FLASH_Lock();
  if (ok != 0U) { prm_save_count++; }
  return ok;
}

static void foc_autocal(void)
{
  foc_reset();
  foc_p1   = g_align_mv;
  foc_p2   = 16384;
  foc_mode = FOC_MODE_ALIGN;
  HAL_Delay(400);

  foc_p2 = 0;
  HAL_Delay(700);

  foc_offset     = foc_offset_at(dbg.enc.angle);
  dbg.foc.offset = foc_offset;

  foc_mode = FOC_MODE_IDLE;
  pwm_set(0U, 0U, 0U);
}

static void safety_trip(uint8_t reason, int32_t lsb)
{
  if (dbg.saf.trip != TRIP_NONE)
  {
    return;
  }

  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
  dbg.saf.drv_enabled = 0U;

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);

  dbg.cmd.duty_a   = 0U;
  dbg.cmd.duty_b   = 0U;
  dbg.cmd.duty_c   = 0U;
  dbg.cmd.drive_on = 0U;

  foc_mode      = FOC_MODE_IDLE;
  foc_coast     = 0U;
  dbg.foc.mode  = FOC_MODE_IDLE;
  foc_int_d     = 0.0f;
  foc_int_q     = 0.0f;

  if (cap_state == 3U)
  {
    cap_state = 2U;
  }

  dbg.foc.vd_mv   = 0;
  dbg.foc.vq_mv   = 0;
  dbg.imp.tau_mnm = 0;

  dbg.saf.trip_ma = lsb * CUR_MA_PER_LSB_X100 / 100;
  dbg.saf.trip    = reason;
  dbg.saf.trip_count++;
}

static uint8_t drv_configure(void)
{
  for (uint8_t a = 0U; a <= 6U; a++)
  {
    dbg.drv.reg[a] = drv_read(a);
  }

  dbg.drv.spi_ok = ((dbg.drv.reg[5] == 0x159U) && (dbg.drv.reg[6] == 0x283U)) ? 1U : 0U;
  dbg.drv.pwm3x  = 0U;
  dbg.drv.cfg_ok = 0U;

  if (dbg.drv.spi_ok == 0U)
  {
    return 0U;
  }

  drv_write(0x02, DRV_REG_DCR);
  drv_write(0x05, DRV_REG_OCP);
  drv_write(0x06, DRV_REG_CSA);

  dbg.drv.reg[2] = drv_read(0x02);
  dbg.drv.reg[5] = drv_read(0x05);
  dbg.drv.reg[6] = drv_read(0x06);

  dbg.drv.pwm3x  = (((dbg.drv.reg[2] >> 5) & 0x3U) == 0x1U) ? 1U : 0U;
  dbg.drv.cfg_ok = ((dbg.drv.reg[2] == DRV_REG_DCR) &&
                    (dbg.drv.reg[5] == DRV_REG_OCP) &&
                    (dbg.drv.reg[6] == DRV_REG_CSA)) ? 1U : 0U;

  return dbg.drv.cfg_ok;
}

static void nfault_arm(void)
{
  GPIO_InitTypeDef gi = {0};

  gi.Pin  = DRV_FAULT_Pin;
  gi.Mode = GPIO_MODE_IT_FALLING;
  gi.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DRV_FAULT_GPIO_Port, &gi);

  __HAL_GPIO_EXTI_CLEAR_IT(DRV_FAULT_Pin);
  NVIC_ClearPendingIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

static uint8_t safety_clear(void)
{
  if (dbg.saf.trip == TRIP_NONE)
  {
    return 1U;
  }

  if ((dbg.saf.trip == TRIP_THERMAL) && (dbg.saf.therm_pct >= g_therm_clear))
  {
    return 0U;
  }
  if ((dbg.saf.trip == TRIP_BOARD_HOT) &&
      ((dbg.temp.c_x10 == TEMP_INVALID) || (dbg.temp.c_x10 >= ((int16_t)g_board_clear * 10))))
  {
    return 0U;
  }

  pwm_set(0U, 0U, 0U);

  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  if (HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_RESET)
  {
    HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
    return 0U;
  }

  if (drv_configure() == 0U)
  {
    HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
    return 0U;
  }

  dbg.saf.drv_enabled = 1U;

  dbg.cur.offset_ok = 0U;
  calibrate_current_offset();

  obs_prev = enc_total;
  obs_x    = 0.0f;
  obs_v    = 0.0f;

  cur_over_run = 0U;
  cur_peak_lsb = 0;
  __HAL_GPIO_EXTI_CLEAR_IT(DRV_FAULT_Pin);
  NVIC_ClearPendingIRQ(EXTI1_IRQn);

  dbg.saf.trip = TRIP_NONE;
  dbg.saf.clear_count++;
  return 1U;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  {
    TIM_OC_InitTypeDef       oc = {0};
    TIM_MasterConfigTypeDef  mc = {0};

    oc.OCMode     = TIM_OCMODE_PWM2;
    oc.Pulse      = ADC_TRIG_CCR4;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_4) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_TIM_SET_AUTORELOAD(&htim2, PWM_ARR);

    mc.MasterOutputTrigger = TIM_TRGO_OC4REF;
    mc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &mc) != HAL_OK)
    {
      Error_Handler();
    }
  }

  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  {
    ADC_ChannelConfTypeDef sc = {0};
    sc.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    sc.SingleDiff   = ADC_SINGLE_ENDED;
    sc.OffsetNumber = ADC_OFFSET_NONE;
    sc.Offset       = 0;

    sc.Channel = ADC_CHANNEL_14; sc.Rank = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sc) != HAL_OK) { Error_Handler(); }
    sc.Channel = ADC_CHANNEL_11; sc.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sc) != HAL_OK) { Error_Handler(); }
    sc.Channel = ADC_CHANNEL_5;  sc.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sc) != HAL_OK) { Error_Handler(); }
  }

  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_adc1.Instance                 = DMA1_Channel1;
  hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;
  hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode                = DMA_CIRCULAR;
  hdma_adc1.Init.Priority            = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  foc_build_tables();

  prm_defaults();
  cal_valid = cal_load();
  dbg.foc.pole_pairs = (uint8_t)foc_pp;
  dbg.foc.offset     = foc_offset;
  dbg.foc.vbus_mv    = foc_vbus_mv;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_SPI_ENABLE(&hspi3);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma, 3) != HAL_OK)
  {
    Error_Handler();
  }

  pwm_set(0U, 0U, 0U);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  dbg.saf.drv_enabled = 1U;

  (void)drv_configure();

  nfault_arm();

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);

  can_filters_update();

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  TxHeader.Identifier          = CANID_TLM(NODE_ID_DEFAULT, 0);
  TxHeader.IdType              = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
  TxHeader.DataLength          = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker       = 0;

  TxHeader2 = TxHeader;
  TxHeader3 = TxHeader;
  TxHeader4 = TxHeader;
  TxHeader5 = TxHeader;

  TxHeaderMit = TxHeader;
  TxHeaderMit.DataLength = FDCAN_DLC_BYTES_6;

  can_ids_update();

  calibrate_current_offset();

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    read_temperature();

    dbg.enc.deg_x10 = (int16_t)(((uint32_t)dbg.enc.angle * 3600U) / 16384U);

    read_currents();
    track_current_offset();

    if ((foc_mode != FOC_MODE_IMPEDANCE) && (imp_lost != 0U))
    {
      imp_lost   = 0U;
      imp_lost_k = 1.0f;
    }
    if ((foc_mode == FOC_MODE_IMPEDANCE) && (imp_lost == 0U) && (foc_age_ms >= g_foc_timeout_ms)
        && ((g_lost_hold_ms != 0U) || (g_lost_rel_ms != 0U)))
    {
      __disable_irq();
      imp_p_des  = imp_p_des + imp_p_ext;
      imp_p_ext  = 0.0f;
      imp_v_des  = 0.0f;
      imp_v_ext  = 0.0f;
      imp_a_est  = 0.0f;
      imp_ext_t  = imp_ext_max;
      imp_lost   = 1U;
      __enable_irq();
      imp_lost_ms = 0U;
      dbg.cmd.lost_count++;
    }
    else if (imp_lost == 1U)
    {
      imp_lost_ms += 100U;
      if (imp_lost_ms >= g_lost_hold_ms)
      {
        imp_lost    = 2U;
        imp_lost_ms = 0U;
      }
    }
    else if (imp_lost == 2U)
    {
      imp_lost_ms += 100U;
      if ((g_lost_rel_ms == 0U) || (imp_lost_ms >= g_lost_rel_ms))
      {
        imp_lost_k   = 0.0f;
        foc_mode     = FOC_MODE_IDLE;
        foc_coast    = 1U;
        foc_coast_ms = 0U;
        imp_lost     = 0U;
        imp_lost_k   = 1.0f;
      }
      else
      {
        imp_lost_k = 1.0f - ((float)imp_lost_ms / (float)g_lost_rel_ms);
      }
    }
    else if (foc_mode != FOC_MODE_IDLE)
    {
      if (foc_age_ms >= g_foc_timeout_ms)
      {
        foc_mode     = FOC_MODE_IDLE;
        foc_coast    = 1U;
        foc_coast_ms = 0U;
      }
      else
      {
        foc_age_ms += 100U;
      }
    }

    if (foc_coast != 0U)
    {
      int16_t r = dbg.foc.rpm;
      if (r < 0) { r = (int16_t)-r; }
      foc_coast_ms += 100U;
      if ((r <= 20) || (foc_coast_ms >= 5000U))
      {
        foc_coast = 0U;
        foc_reset();
      }
    }
    dbg.foc.mode       = foc_mode;
    dbg.foc.pole_pairs = (uint8_t)foc_pp;
    dbg.foc.offset     = foc_offset;
    dbg.foc.vbus_mv    = foc_vbus_mv;

    {
      int32_t n   = isq_n;
      int32_t acc = isq_acc;
      int32_t u_x1000 = 0;

      isq_n   = 0;
      isq_acc = 0;

      if (therm_seeded == 0U)
      {
        int16_t tb = dbg.temp.c_x10;
        therm_seeded = 1U;
        if ((tb != TEMP_INVALID) && (g_therm_rise > 0))
        {
          int32_t th = (((int32_t)tb - ((int32_t)g_therm_amb * 10)) * 10000) / (int32_t)g_therm_rise;
          if (th > 0) { therm_x100000 = th; }
        }
      }

      if (n > 0)
      {
        int32_t mean_lsb2 = (acc / n) * 64;
        u_x1000 = (mean_lsb2 * 1000) / g_isq_rated;
      }

      {
        float tw = (float)g_therm_amb + ((float)g_therm_rise * (float)therm_x100000 * 0.00001f);
        float k  = (1.0f + (CU_ALPHA * (tw - 20.0f))) / (1.0f + (CU_ALPHA * ((float)g_therm_tref - 20.0f)));
        if (k < 0.5f) { k = 0.5f; }
        if (k > 2.0f) { k = 2.0f; }
        u_x1000 = (int32_t)((float)u_x1000 * k);
      }

      therm_x100000 += ((u_x1000 * 100) - therm_x100000) / g_therm_tau;
      if (therm_x100000 < 0) { therm_x100000 = 0; }
      dbg.saf.therm_pct = (int16_t)(therm_x100000 / 1000);

      if (dbg.saf.therm_pct >= g_therm_trip)
      {
        safety_trip(TRIP_THERMAL, 0);
      }
    }

    {
      int16_t tb = dbg.temp.c_x10;
      if ((tb == TEMP_INVALID) || (tb >= ((int16_t)g_board_trip * 10)))
      {
        if (board_hot_run < BOARD_HOT_N) { board_hot_run++; }
      }
      else
      {
        board_hot_run = 0U;
      }
      if (board_hot_run >= BOARD_HOT_N)
      {
        safety_trip(TRIP_BOARD_HOT, 0);
      }

      {
        float imax  = (float)g_imax_ma;
        float icont = 1.41421356f * (float)g_irated_ma;
        float lim   = imax;
        if (icont > imax) { icont = imax; }
        {
          float tw  = (float)g_therm_amb + ((float)g_therm_rise * (float)therm_x100000 * 0.00001f);
          float t0  = (float)g_wind_derate, t1 = (float)g_therm_amb + (float)g_therm_rise;
          if ((t1 > t0) && (tw > t0))
          {
            float f = (tw - t0) / (t1 - t0);
            if (f > 1.0f) { f = 1.0f; }
            float l = imax - (imax - icont) * f;
            if (l < lim) { lim = l; }
          }
        }
        if (tb != TEMP_INVALID)
        {
          float t  = (float)tb * 0.1f;
          float t0 = (float)g_board_derate, t1 = (float)g_board_trip;
          if ((t1 > t0) && (t > t0))
          {
            float f = (t - t0) / (t1 - t0);
            if (f > 1.0f) { f = 1.0f; }
            float l = imax - (imax - icont) * f;
            if (l < lim) { lim = l; }
          }
        }
        if (lim < icont) { lim = icont; }
        g_ilim_ma = (int32_t)lim;
        dbg.saf.ilim_ma = (int16_t)g_ilim_ma;
      }
    }

    if (dbg.cmd.drive_on != 0U)
    {
      if (dbg.cmd.age_ms >= CMD_TIMEOUT_MS)
      {
        dbg.cmd.duty_a = 0U;
        dbg.cmd.duty_b = 0U;
        dbg.cmd.duty_c = 0U;
        dbg.cmd.drive_on = 0U;
      }
      else
      {
        dbg.cmd.age_ms += 100U;
      }
    }

    if ((foc_autocal_req != 0U) && (dbg.saf.trip == TRIP_NONE))
    {
      foc_autocal_req = 0U;
      foc_autocal();
    }
    if (prm_save_req != 0U)
    {
      prm_save_req = 0U;
      if ((foc_mode == FOC_MODE_IDLE) && (dbg.cmd.drive_on == 0U))
      {
        (void)cal_save_as(1U);
        obs_prev = enc_total;
        obs_x    = 0.0f;
        obs_v    = 0.0f;
      }
    }
    if (cal_save_req != 0U)
    {
      cal_save_req = 0U;
      if ((foc_mode == FOC_MODE_IDLE) && (dbg.cmd.drive_on == 0U))
      {
        cal_valid = cal_save();

        obs_prev = enc_total;
        obs_x    = 0.0f;
        obs_v    = 0.0f;
      }
    }

    if (saf_clear_req != 0U)
    {
      saf_clear_req = 0U;
      (void)safety_clear();
    }

    if (dbg.saf.trip != TRIP_NONE)
    {
      pwm_set(0U, 0U, 0U);
    }
    else if ((foc_mode == FOC_MODE_IDLE) && (foc_coast == 0U))
    {
      pwm_set(dbg.cmd.duty_a, dbg.cmd.duty_b, dbg.cmd.duty_c);
    }

    dbg.drv.fault  = (uint8_t)HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin);

    if ((dbg.drv.fault == 0U) && (dbg.saf.trip == TRIP_NONE) && (dbg.saf.drv_enabled != 0U))
    {
      safety_trip(TRIP_NFAULT, 0);
    }

    dbg.saf.peak_ma = cur_peak_lsb * CUR_MA_PER_LSB_X100 / 100;

    uint32_t t   = HAL_GetTick();
    uint32_t n   = dbg.can.tx_count;
    uint16_t sec = (uint16_t)(t / 1000U);
    uint16_t adc = dbg.temp.adc_raw;
    uint16_t tmp = (uint16_t)dbg.temp.c_x10;
    dbg.uptime_ms = t;

    TxData[0] = (uint8_t)(n >> 8);
    TxData[1] = (uint8_t)(n);
    TxData[2] = (uint8_t)(adc >> 8);
    TxData[3] = (uint8_t)(adc);
    TxData[4] = (uint8_t)(tmp >> 8);
    TxData[5] = (uint8_t)(tmp);
    TxData[6] = (uint8_t)(sec >> 8);
    TxData[7] = (uint8_t)(sec);

    if (can_send(&TxHeader, TxData) != 0U)
    {
      dbg.can.tx_count = n + 1;
      dbg.can.bus_ok = 1;
    }
    else
    {
      dbg.can.bus_ok = 0;
    }

    {
      static uint16_t tlm_cnt = 0U;
      uint8_t tlm_now = 0U;

      if (tlm_div != 0U)
      {
        tlm_cnt++;
        if (tlm_cnt >= tlm_div) { tlm_cnt = 0U; tlm_now = 1U; }
      }

    if (tlm_now != 0U)
    {

    uint8_t flags = 0;
    if (dbg.drv.spi_ok        != 0U) { flags |= 0x01U; }
    if (dbg.enc.parity_ok != 0U) { flags |= 0x02U; }
    if (dbg.enc.nomag     != 0U) { flags |= 0x04U; }
    if (dbg.drv.fault     == 0U) { flags |= 0x08U; }
    if (dbg.drv.pwm3x     != 0U) { flags |= 0x10U; }
    if (dbg.drv.cfg_ok    != 0U) { flags |= 0x20U; }
    if (dbg.saf.drv_enabled != 0U) { flags |= 0x40U; }

    TxData2[0] = (uint8_t)(dbg.drv.reg[5] >> 8);
    TxData2[1] = (uint8_t)(dbg.drv.reg[5]);
    TxData2[2] = (uint8_t)(dbg.drv.reg[6] >> 8);
    TxData2[3] = (uint8_t)(dbg.drv.reg[6]);
    TxData2[4] = (uint8_t)(dbg.enc.angle >> 8);
    TxData2[5] = (uint8_t)(dbg.enc.angle);
    TxData2[6] = flags;
    TxData2[7] = dbg.saf.trip;

    (void)can_send(&TxHeader2, TxData2);

    TxData3[0] = (uint8_t)(dbg.cur.raw[0] >> 8);
    TxData3[1] = (uint8_t)(dbg.cur.raw[0]);
    TxData3[2] = (uint8_t)(dbg.cur.raw[1] >> 8);
    TxData3[3] = (uint8_t)(dbg.cur.raw[1]);
    TxData3[4] = (uint8_t)(dbg.cur.raw[2] >> 8);
    TxData3[5] = (uint8_t)(dbg.cur.raw[2]);
    TxData3[6] = dbg.cur.offset_ok;
    {
      int32_t pk = dbg.saf.peak_ma / 100;
      if (pk > 255) { pk = 255; }
      TxData3[7] = (uint8_t)pk;
    }

    (void)can_send(&TxHeader3, TxData3);

    {
      int32_t ia = dbg.cur.a_ma;
      int32_t ib = dbg.cur.b_ma;
      int32_t ic = dbg.cur.c_ma;
      if (ia >  32767) { ia =  32767; }  if (ia < -32768) { ia = -32768; }
      if (ib >  32767) { ib =  32767; }  if (ib < -32768) { ib = -32768; }
      if (ic >  32767) { ic =  32767; }  if (ic < -32768) { ic = -32768; }

      TxData4[0] = (uint8_t)((uint16_t)(int16_t)ia >> 8);
      TxData4[1] = (uint8_t)((uint16_t)(int16_t)ia);
      TxData4[2] = (uint8_t)((uint16_t)(int16_t)ib >> 8);
      TxData4[3] = (uint8_t)((uint16_t)(int16_t)ib);
      TxData4[4] = (uint8_t)((uint16_t)(int16_t)ic >> 8);
      TxData4[5] = (uint8_t)((uint16_t)(int16_t)ic);
      TxData4[6] = dbg.cmd.drive_on;
      TxData4[7] = (uint8_t)dbg.cmd.reject_count;
    }

    (void)can_send(&TxHeader4, TxData4);

    {
      int32_t  tp = dbg.saf.therm_pct;
      uint32_t ec = dbg.enc.err_count;
      if (tp < 0)   { tp = 0; }
      if (tp > 255) { tp = 255; }
      if (ec > 255U) { ec = 255U; }
      TxData5[0] = (uint8_t)tp;
      TxData5[1] = (uint8_t)ec;
      TxData5[2] = (uint8_t)(dbg.foc.isr_cycles >> 8);
      TxData5[3] = (uint8_t)(dbg.foc.isr_cycles);
      TxData5[4] = (uint8_t)(dbg.foc.isr_max >> 8);
      TxData5[5] = (uint8_t)(dbg.foc.isr_max);
      TxData5[6] = dbg.saf.trip;
      TxData5[7] = dbg.foc.mode;
    }

    (void)can_send(&TxHeader5, TxData5);

    TxData5[0] = (uint8_t)(dbg.foc.e_angle >> 8);
    TxData5[1] = (uint8_t)(dbg.foc.e_angle);
    TxData5[2] = (uint8_t)(dbg.foc.e_angle_enc >> 8);
    TxData5[3] = (uint8_t)(dbg.foc.e_angle_enc);
    TxData5[4] = (uint8_t)((uint16_t)dbg.foc.iq_ma >> 8);
    TxData5[5] = (uint8_t)((uint16_t)dbg.foc.iq_ma);
    TxData5[6] = (uint8_t)((uint16_t)dbg.foc.id_ma >> 8);
    TxData5[7] = (uint8_t)((uint16_t)dbg.foc.id_ma);
    TxHeader5.Identifier = CANID_TLM(node_id, 5);
    (void)can_send(&TxHeader5, TxData5);

    {
      uint32_t tot = (uint32_t)dbg.imp.pos_cnt;
      TxData5[0] = (uint8_t)(tot >> 24);
      TxData5[1] = (uint8_t)(tot >> 16);
      TxData5[2] = (uint8_t)(tot >> 8);
      TxData5[3] = (uint8_t)(tot);
    }
    TxData5[4] = (uint8_t)((uint16_t)dbg.foc.rpm >> 8);
    TxData5[5] = (uint8_t)((uint16_t)dbg.foc.rpm);
    {
      uint16_t tn = (uint16_t)(int16_t)foc_ol_turns;
      TxData5[6] = (uint8_t)(tn >> 8);
      TxData5[7] = (uint8_t)(tn);
    }
    TxHeader5.Identifier = CANID_TLM(node_id, 6);
    (void)can_send(&TxHeader5, TxData5);

    TxData5[0] = (uint8_t)((uint16_t)dbg.foc.vd_mv >> 8);
    TxData5[1] = (uint8_t)((uint16_t)dbg.foc.vd_mv);
    TxData5[2] = (uint8_t)((uint16_t)dbg.foc.vq_mv >> 8);
    TxData5[3] = (uint8_t)((uint16_t)dbg.foc.vq_mv);
    TxData5[4] = (uint8_t)(dbg.foc.vbus_mv >> 8);
    TxData5[5] = (uint8_t)(dbg.foc.vbus_mv);
    TxData5[6] = dbg.foc.pole_pairs;
    TxData5[7] = dbg.foc.mode;
    TxHeader5.Identifier = CANID_TLM(node_id, 7);
    (void)can_send(&TxHeader5, TxData5);

    TxData5[0] = (uint8_t)(dbg.foc.offset >> 8);
    TxData5[1] = (uint8_t)(dbg.foc.offset);
    TxData5[2] = (uint8_t)(foc_pp | ((cal_valid != 0U) ? 0x80U : 0x00U));
    TxData5[3] = (uint8_t)foc_dir;
    {
      uint16_t kp = (uint16_t)(int16_t)(foc_kp * 1000.0f);
      uint16_t ki = (uint16_t)(int16_t)(foc_ki);
      TxData5[4] = (uint8_t)(kp >> 8);
      TxData5[5] = (uint8_t)(kp);
      TxData5[6] = (uint8_t)(ki >> 8);
      TxData5[7] = (uint8_t)(ki);
    }
    TxHeader5.Identifier = CANID_TLM(node_id, 8);
    (void)can_send(&TxHeader5, TxData5);

    TxData5[0] = (uint8_t)((uint16_t)dbg.imp.pos_mrad >> 8);
    TxData5[1] = (uint8_t)((uint16_t)dbg.imp.pos_mrad);
    TxData5[2] = (uint8_t)((uint16_t)dbg.imp.vel_x10 >> 8);
    TxData5[3] = (uint8_t)((uint16_t)dbg.imp.vel_x10);
    TxData5[4] = (uint8_t)((uint16_t)dbg.imp.tau_mnm >> 8);
    TxData5[5] = (uint8_t)((uint16_t)dbg.imp.tau_mnm);
    TxData5[6] = (uint8_t)((uint16_t)dbg.imp.p_des_mrad >> 8);
    TxData5[7] = (uint8_t)((uint16_t)dbg.imp.p_des_mrad);
    TxHeader5.Identifier = CANID_TLM(node_id, 10);
    (void)can_send(&TxHeader5, TxData5);

    TxData5[0] = (uint8_t)((uint16_t)dbg.imp.v_des_x10 >> 8);
    TxData5[1] = (uint8_t)((uint16_t)dbg.imp.v_des_x10);
    TxData5[2] = (uint8_t)((uint16_t)dbg.imp.kp_x10 >> 8);
    TxData5[3] = (uint8_t)((uint16_t)dbg.imp.kp_x10);
    TxData5[4] = (uint8_t)((uint16_t)dbg.imp.kd_x100 >> 8);
    TxData5[5] = (uint8_t)((uint16_t)dbg.imp.kd_x100);
    TxData5[6] = (uint8_t)((uint16_t)dbg.imp.tff_mnm >> 8);
    TxData5[7] = (uint8_t)((uint16_t)dbg.imp.tff_mnm);
    TxHeader5.Identifier = CANID_TLM(node_id, 11);
    (void)can_send(&TxHeader5, TxData5);

    {
      int32_t lo = (int32_t)((float)lim_lo_cnt * RAD_PER_COUNT * 100.0f);
      int32_t hi = (int32_t)((float)lim_hi_cnt * RAD_PER_COUNT * 100.0f);
      if (lo >  32767) { lo =  32767; }  if (lo < -32768) { lo = -32768; }
      if (hi >  32767) { hi =  32767; }  if (hi < -32768) { hi = -32768; }
      TxData5[0] = (uint8_t)((uint16_t)(int16_t)lo >> 8);
      TxData5[1] = (uint8_t)((uint16_t)(int16_t)lo);
      TxData5[2] = (uint8_t)((uint16_t)(int16_t)hi >> 8);
      TxData5[3] = (uint8_t)((uint16_t)(int16_t)hi);
      TxData5[4] = tlm_div;

      TxData5[5] = (uint8_t)((170000000UL /
                              (2UL * ((uint32_t)htim2.Instance->ARR + 1UL))) / 1000UL);

      TxData5[6] = (uint8_t)(foc_dec_x1000 / 8);
      {
        float hm = sqrtf((foc_h6c * foc_h6c) + (foc_h6s * foc_h6s)) * 100.0f;
        TxData5[7] = (uint8_t)((hm > 255.0f) ? 255.0f : hm);
      }
    }
    TxHeader5.Identifier = CANID_TLM(node_id, 12);
    (void)can_send(&TxHeader5, TxData5);

    {
      static uint8_t prm_page = 0U;
      uint8_t a = (uint8_t)(1U + (2U * prm_page));
      uint8_t b = (uint8_t)(a + 1U);
      int16_t va = prm_get(a);
      int16_t vb = (b < PRM_N) ? prm_get(b) : 0;
      TxData5[0] = a;
      TxData5[1] = (uint8_t)((uint16_t)va >> 8);
      TxData5[2] = (uint8_t)((uint16_t)va);
      TxData5[3] = (b < PRM_N) ? b : 0U;
      TxData5[4] = (uint8_t)((uint16_t)vb >> 8);
      TxData5[5] = (uint8_t)((uint16_t)vb);
      TxData5[6] = prm_save_count;
      TxData5[7] = prm_page;
      prm_page = (uint8_t)(prm_page + 1U);
      if ((1U + (2U * prm_page)) >= PRM_N) { prm_page = 0U; }
    }
    TxHeader5.Identifier = CANID_TLM(node_id, 13);
    (void)can_send(&TxHeader5, TxData5);

    }
    }

    if (cap_state == 2U)
    {
      TxHeader5.Identifier = CANID_TLM(node_id, 9);

      TxData5[0] = 0xFFU;
      TxData5[1] = 0xFFU;
      TxData5[2] = (uint8_t)(cap_idx >> 8);
      TxData5[3] = (uint8_t)(cap_idx);
      TxData5[4] = 0U;  TxData5[5] = 0U;  TxData5[6] = 0U;  TxData5[7] = 0U;
      (void)can_send(&TxHeader5, TxData5);

      for (uint32_t i = 0U; i < CAP_N; i += 3U)
      {
        TxData5[0] = (uint8_t)(i >> 8);
        TxData5[1] = (uint8_t)(i);
        for (uint32_t j = 0U; j < 3U; j++)
        {
          uint16_t v = (uint16_t)cap_buf[i + j];
          TxData5[2U + (j * 2U)] = (uint8_t)(v >> 8);
          TxData5[3U + (j * 2U)] = (uint8_t)(v);
        }
        (void)can_send(&TxHeader5, TxData5);
      }
      cap_state = 0U;
    }

    TxHeader5.Identifier = CANID_TLM(node_id, 4);

    if (rx_led_hold > 0u)
    {
      rx_led_hold--;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
    else
    {
      led_tick++;
      if ((led_tick % (dbg.can.bus_ok ? 5u : 1u)) == 0u)
      {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      }
    }

    HAL_Delay(100);
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 0;

  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = 4249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, DRV_ENABLE_Pin|DRV_CAL_Pin|GPIO_Output_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, DRV_CS_Pin|ENC_CS_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = DRV_ENABLE_Pin|DRV_CAL_Pin|GPIO_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DRV_CS_Pin|ENC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DRV_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DRV_FAULT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(DRV_FAULT_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == DRV_FAULT_Pin)
  {
    dbg.saf.nfault_count++;
    safety_trip(TRIP_NFAULT, 0);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint32_t t0 = DWT->CYCCNT;
  int32_t  d[3];
  int32_t  worst = 0;
  uint32_t used;

  if (hadc->Instance != ADC1)
  {
    return;
  }

  enc_and_state();

  if ((dbg.cur.offset_ok == 0U) ||
      (dbg.saf.trip != TRIP_NONE) ||
      (dbg.saf.drv_enabled == 0U))
  {
    cur_over_run = 0U;
    return;
  }

  for (uint8_t i = 0U; i < 3U; i++)
  {
    d[i] = (int32_t)adc1_dma[i] - (int32_t)dbg.cur.offset[i];
  }

  if (foc_hi_phase < 3U)
  {
    d[foc_hi_phase] = -((d[0] + d[1] + d[2]) - d[foc_hi_phase]);
  }

  for (uint8_t i = 0U; i < 3U; i++)
  {
    int32_t a = (d[i] < 0) ? -d[i] : d[i];
    if (a > worst) { worst = a; }
  }

  if (worst > cur_peak_lsb)
  {
    cur_peak_lsb = worst;
  }

  if (cap_state == 3U)
  {
    cap_buf[cap_idx] = (int16_t)worst;
    cap_idx++;
    if (cap_idx >= CAP_N) { cap_idx = 0U; }
  }

  if (worst > CUR_TRIP_LSB)
  {
    cur_over_run++;
    if (cur_over_run >= CUR_TRIP_N)
    {
      safety_trip(TRIP_OVERCUR, worst);
      return;
    }
  }
  else
  {
    cur_over_run = 0U;
  }

  isq_acc += ((d[0] * d[0]) + (d[1] * d[1]) + (d[2] * d[2])) >> 6;
  isq_n++;

  foc_step(d);

  used = DWT->CYCCNT - t0;
  dbg.foc.isr_cycles = (uint16_t)((used > 65535U) ? 65535U : used);
  if (dbg.foc.isr_cycles > dbg.foc.isr_max)
  {
    dbg.foc.isr_max = dbg.foc.isr_cycles;
  }
}

#define RX_DRAIN_MAX  8U
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
  {

    uint32_t rx_n = 0U;
    while ((rx_n < RX_DRAIN_MAX) &&
           (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) &&
           (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK))
    {
      rx_n++;
      dbg.can.rx_count++;
      dbg.can.rx_id  = RxHeader.Identifier;

      if (RxHeader.Identifier == CANID_ESTOP)
      {

        safety_trip(TRIP_ESTOP, 0);
        dbg.cmd.cmd_count++;
      }

      else if (((RxHeader.Identifier == CANID_DUTY(node_id)) ||
                (RxHeader.Identifier == CANID_CFG(node_id))  ||
                (RxHeader.Identifier == CANID_IMP(node_id))) &&
               (RxHeader.DataLength != FDCAN_DLC_BYTES_8))
      {
        dbg.cmd.reject_count++;
      }
      else if (RxHeader.Identifier == CANID_DUTY(node_id))
      {
        uint16_t a = (uint16_t)((RxData[2] << 8) | RxData[3]);
        uint16_t b = (uint16_t)((RxData[4] << 8) | RxData[5]);
        uint16_t c = (uint16_t)((RxData[6] << 8) | RxData[7]);

        if (RxData[0] != CMD_MAGIC)
        {
          dbg.cmd.reject_count++;
        }
        else if (RxData[1] == CMD_STOP)
        {
          dbg.cmd.duty_a = 0U;
          dbg.cmd.duty_b = 0U;
          dbg.cmd.duty_c = 0U;
          dbg.cmd.drive_on = 0U;
          dbg.cmd.age_ms = 0U;
          dbg.cmd.cmd_count++;
        }
        else if (RxData[1] == CMD_TEST_TRIP)
        {

          EXTI->SWIER1 = DRV_FAULT_Pin;
          dbg.cmd.cmd_count++;
        }
        else if (RxData[1] == CMD_CLEAR_TRIP)
        {
          saf_clear_req = 1U;
          dbg.cmd.cmd_count++;
        }
        else if (dbg.saf.trip != TRIP_NONE)
        {
          dbg.cmd.reject_count++;
        }
        else if ((a > PWM_MAX_TEST) || (b > PWM_MAX_TEST) || (c > PWM_MAX_TEST))
        {
          dbg.cmd.reject_count++;
        }
        else
        {
          dbg.cmd.duty_a = a;
          dbg.cmd.duty_b = b;
          dbg.cmd.duty_c = c;
          dbg.cmd.drive_on = 1U;
          dbg.cmd.age_ms = 0U;
          dbg.cmd.cmd_count++;
        }
      }

      else if (RxHeader.Identifier == CANID_CFG(node_id))
      {
        int16_t p1 = (int16_t)(uint16_t)((RxData[2] << 8) | RxData[3]);
        int16_t p2 = (int16_t)(uint16_t)((RxData[4] << 8) | RxData[5]);
        uint8_t cmd = RxData[1];

        if (RxData[0] != CMD_MAGIC)
        {
          dbg.cmd.reject_count++;
        }
        else if ((dbg.saf.trip != TRIP_NONE) && (cmd != FOC_MODE_IDLE))
        {
          dbg.cmd.reject_count++;
        }
        else
        {
          switch (cmd)
          {
            case FOC_MODE_IDLE:

              foc_mode     = FOC_MODE_IDLE;
              foc_coast    = 1U;
              foc_coast_ms = 0U;
              break;

            case FOC_MODE_ALIGN:
              if (foc_mode != cmd) { foc_reset(); }
              foc_coast = 0U;
              foc_p1   = p1;
              foc_p2   = p2;
              foc_mode = cmd;
              break;

            case FOC_MODE_CURRENT:
              if (foc_mode != cmd) { foc_reset(); }
              foc_coast = 0U;

              if (p1 >  g_imax_ma) { p1 = (int16_t) g_imax_ma; }
              if (p1 < -g_imax_ma) { p1 = (int16_t)-g_imax_ma; }
              if (p2 >  g_imax_ma) { p2 = (int16_t) g_imax_ma; }
              if (p2 < -g_imax_ma) { p2 = (int16_t)-g_imax_ma; }
              foc_p1   = p1;
              foc_p2   = p2;
              foc_mode = cmd;
              break;

            case FOC_CMD_STEP_V:
              foc_reset();
              foc_p1    = p1;
              foc_p2    = p2;
              foc_mode  = FOC_MODE_ALIGN;
              cap_src   = 0U;
              cap_div   = 1U;
              cap_idx   = 0U;
              cap_cnt   = 0U;
              cap_state = 1U;
              break;

            case FOC_CMD_STEP_I:
              foc_reset();
              if (p1 >  g_imax_ma) { p1 = (int16_t) g_imax_ma; }
              if (p1 < -g_imax_ma) { p1 = (int16_t)-g_imax_ma; }
              foc_p1    = 0;
              foc_p2    = p1;
              foc_mode  = FOC_MODE_CURRENT;
              cap_src   = 0U;
              cap_div   = 1U;
              cap_idx   = 0U;
              cap_cnt   = 0U;
              cap_state = 1U;
              break;

            case FOC_CMD_AUTOCAL:
              foc_autocal_req = 1U;
              break;

            case FOC_CMD_SAVE_CAL:
              cal_save_req = 1U;
              break;

            case FOC_CMD_SAVE_PARAM:
              prm_save_req = 1U;
              break;

            case FOC_CMD_ARM_TRIPCAP:
              cap_src   = 2U;
              cap_idx   = 0U;
              cap_state = 3U;
              break;

            case FOC_CMD_SET_CAPSRC:

              cap_src = (uint8_t)p1;
              cap_div = (p2 > 0) ? (uint16_t)p2 : 1U;
              break;

            case FOC_CMD_ARM_CAP:
              cap_idx   = 0U;
              cap_cnt   = 0U;
              cap_state = 1U;
              break;

            case FOC_CMD_SET_NODE:

              if ((p1 >= 1) && (p1 <= (int16_t)NODE_ID_MAX))
              {
                node_id = (uint8_t)p1;
                can_ids_update();
              }
              else
              {
                dbg.cmd.reject_count++;
              }
              break;

            case FOC_CMD_SET_LIMIT:

              lim_lo_cnt = (int32_t)((float)p1 * 0.01f / RAD_PER_COUNT);
              lim_hi_cnt = (int32_t)((float)p2 * 0.01f / RAD_PER_COUNT);
              break;

            case FOC_CMD_SET_HARM:
              if (p1 == 2)
              {
                foc_h6c = 0.0f; foc_h6s = 0.0f;
                foc_h12c = 0.0f; foc_h12s = 0.0f;
              }
              else
              {
                foc_harm_on = (p1 != 0) ? 1U : 0U;

                if (p2 != 0) { foc_harm_mu = (float)p2 * 1e-6f; }
              }
              break;

            case FOC_CMD_SET_DECOUP:

              if (p1 < 0)    { p1 = 0; }
              if (p1 > 1000) { p1 = 1000; }
              foc_dec_x1000 = p1;
              break;

            case FOC_CMD_SET_TLM:
              tlm_div = (uint8_t)((p1 < 0) ? 0 : ((p1 > 255) ? 255 : p1));
              break;

            case FOC_CMD_IMPEDANCE:

              imp_p_des = pos_rad();
              imp_v_des = 0.0f;
              imp_kp    = 0.0f;
              imp_kd    = 0.0f;
              imp_tff   = 0.0f;
              foc_reset();
              foc_coast = 0U;
              foc_mode  = FOC_MODE_IMPEDANCE;
              break;

            case FOC_MODE_OPENLOOP:
              foc_coast = 0U;
              if (foc_mode != FOC_MODE_OPENLOOP)
              {
                foc_reset();
                foc_ol_acc   = 0;
                foc_ol_turns = 0;
              }
              foc_p1 = p1;

              foc_ol_inc = (int32_t)(((int64_t)p2 * 4294967296LL)
                                     / (10LL * (int64_t)PWM_HZ));
              foc_mode = FOC_MODE_OPENLOOP;
              break;

            case FOC_CMD_LEARN_OFS:
              foc_learn_req = 1U;
              break;

            case FOC_CMD_SET_VBUS:
              if (p1 > 0) { foc_vbus_mv = (uint16_t)p1; }
              break;

            case FOC_CMD_SET_PP:
              if ((p1 > 0) && (p1 < 128)) { foc_pp = (uint16_t)p1; }
              break;

            case FOC_CMD_SET_GAIN:
              foc_kp = (float)p1 * 0.001f;
              foc_ki = (float)p2;
              break;

            case FOC_CMD_SET_DIR:
              foc_dir = (p1 >= 0) ? (int8_t)1 : (int8_t)-1;
              break;

            case FOC_CMD_SET_PARAM:

              if (p1 == 0) { prm_defaults_as((foc_mode != FOC_MODE_IDLE) ? 1U : 0U); }
              else if ((p1 > 0) && (p1 < (int16_t)PRM_N))
              {
                if ((foc_mode != FOC_MODE_IDLE) && PRM_IDLE_ONLY((uint8_t)p1)) { dbg.cmd.reject_count++; }
                else { prm_set((uint8_t)p1, p2); }
              }
              else { dbg.cmd.reject_count++; }
              break;

            default:
              dbg.cmd.reject_count++;
              break;
          }
          foc_age_ms = 0U;
          dbg.cmd.cmd_count++;
        }
      }

      else if (RxHeader.Identifier == CANID_IMP(node_id))
      {
        if ((RxData[0] == 0xFFU) && (RxData[1] == 0xFFU) && (RxData[2] == 0xFFU) &&
            (RxData[3] == 0xFFU) && (RxData[4] == 0xFFU) && (RxData[5] == 0xFFU) &&
            (RxData[6] == 0xFFU))
        {
          switch (RxData[7])
          {
            case 0xFCU:
              if (dbg.saf.trip == TRIP_NONE)
              {

                imp_p_des = pos_rad();
                imp_v_des = 0.0f;
                imp_p_ext  = 0.0f;
                imp_v_ext  = 0.0f;
                imp_a_est  = 0.0f;
                imp_v_last = 0.0f;
                imp_ext_t  = 0.0f;
                imp_kp    = 0.0f;
                imp_kd    = 0.0f;
                imp_tff   = 0.0f;
                foc_reset();
                foc_coast = 0U;
                foc_mode  = FOC_MODE_IMPEDANCE;
                dbg.cmd.cmd_count++;
              }
              else
              {
                dbg.cmd.reject_count++;
              }
              break;

            case 0xFDU:
              foc_mode     = FOC_MODE_IDLE;
              foc_coast    = 1U;
              foc_coast_ms = 0U;
              dbg.cmd.cmd_count++;
              break;

            case 0xFEU:
              pos_zero_req = 1U;
              imp_p_des    = 0.0f;
              imp_p_ext    = 0.0f;
              imp_v_ext    = 0.0f;
              imp_a_est    = 0.0f;
              imp_ext_t    = 0.0f;
              dbg.cmd.cmd_count++;
              break;

            default:
              dbg.cmd.reject_count++;
              break;
          }
          foc_age_ms = 0U;
          mit_send_feedback();
        }
        else if (dbg.saf.trip != TRIP_NONE)
        {

          dbg.cmd.reject_count++;
          mit_send_feedback();
        }
        else if (imp_lost == 2U)
        {
          dbg.cmd.reject_count++;
          mit_send_feedback();
        }
        else
        {
          uint16_t p_i  = (uint16_t)(((uint16_t)RxData[0] << 8) | RxData[1]);
          uint16_t v_i  = (uint16_t)(((uint16_t)RxData[2] << 4) | (RxData[3] >> 4));
          uint16_t kp_i = (uint16_t)(((uint16_t)(RxData[3] & 0x0FU) << 8) | RxData[4]);
          uint16_t kd_i = (uint16_t)(((uint16_t)RxData[5] << 4) | (RxData[6] >> 4));
          uint16_t t_i  = (uint16_t)(((uint16_t)(RxData[6] & 0x0FU) << 8) | RxData[7]);

          imp_p_des = mit_unpack(p_i,  -g_p_max,  g_p_max,  16);
          imp_v_des = mit_unpack(v_i,  -g_v_max,  g_v_max,  12);

          {
            float dt_cmd = imp_ext_t;

            if ((dt_cmd > IMP_EXTRAP_MIN_S) && (dt_cmd < g_extrap_max))
            {
              float a = (imp_v_des - imp_v_last) / dt_cmd;
              if (a >  g_acc_max) { a =  g_acc_max; }
              if (a < -g_acc_max) { a = -g_acc_max; }
              imp_a_est   = a;
              imp_ext_max = dt_cmd * 1.5f;
              if (imp_ext_max > g_extrap_max) { imp_ext_max = g_extrap_max; }
            }
            else
            {
              imp_a_est   = 0.0f;
              imp_ext_max = g_extrap_max;
            }
          }
          imp_v_last = imp_v_des;
          imp_p_ext  = 0.0f;
          imp_v_ext  = 0.0f;
          imp_ext_t  = 0.0f;

          imp_kp    = mit_unpack(kp_i, 0.0f,        g_kp_max, 12);
          imp_kd    = mit_unpack(kd_i, 0.0f,        g_kd_max, 12);
          imp_tff   = mit_unpack(t_i,  -g_t_max,  g_t_max,  12);

          dbg.imp.p_des_mrad = (int16_t)(imp_p_des * 1000.0f);
          dbg.imp.v_des_x10  = (int16_t)(imp_v_des * 10.0f);
          dbg.imp.kp_x10     = (int16_t)(imp_kp * 10.0f);
          dbg.imp.kd_x100    = (int16_t)(imp_kd * 100.0f);
          dbg.imp.tff_mnm    = (int16_t)(imp_tff * 1000.0f);
          dbg.imp.cmd_count++;

          if (foc_mode != FOC_MODE_IMPEDANCE)
          {
            foc_reset();
            foc_coast = 0U;
            foc_mode  = FOC_MODE_IMPEDANCE;
          }
          foc_age_ms = 0U;
          imp_lost   = 0U;
          imp_lost_k = 1.0f;
          dbg.cmd.cmd_count++;

          mit_send_feedback();
        }
      }

      dbg.can.rx_dlc = RxHeader.DataLength;
      for (uint32_t i = 0; i < 8U; i++)
      {
        dbg.can.rx_data[i] = RxData[i];
      }

      rx_led_hold = 10;

    }

    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  /* USER CODE END 6 */
}
#endif
