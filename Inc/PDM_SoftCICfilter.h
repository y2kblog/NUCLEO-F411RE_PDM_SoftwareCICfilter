/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PDM_SOFTCICFILTER_H
#define __PDM_SOFTCICFILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include system header files -----------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Include user header files -------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* Private macro -------------------------------------------------------------*/
// CIC filter setting
#define DECIMATION_M        20
#define PDM_SAMPLE_SIZE     4096

/* Exported types ------------------------------------------------------------*/
/* Exported enum tag ---------------------------------------------------------*/
/* Exported struct/union tag -------------------------------------------------*/
struct CicFilter_t
{
    uint8_t order;
    uint32_t decimation;
    int32_t *out_i;
    int32_t *out_c;
    int32_t *z1_c;
};

/* Exported variables --------------------------------------------------------*/
volatile bool needs_CopyPDMbits;
volatile bool isCalled_PDM_DMA_Callback;

struct CicFilter_t PDM_st;
uint8_t PDM_Buff[PDM_SAMPLE_SIZE * DECIMATION_M / 8 * 2];     /* <Sample Size> * <Decimation of CIC filter> / <bits per 1Byte> * <for Double buffer> */
volatile uint8_t PDM_RawBits[PDM_SAMPLE_SIZE * DECIMATION_M / 8];
int32_t PDM_Filtered_int32[PDM_SAMPLE_SIZE];


/* Exported function prototypes ----------------------------------------------*/
void initializeCicFilterStruct(uint8_t, uint32_t, struct CicFilter_t*);
void resetCicFilterStruct(struct CicFilter_t*);
void startPDM(void);
void executeCicFilter(uint8_t*, uint32_t, int32_t*, struct CicFilter_t*);
void finalizeCicFilterStruct(struct CicFilter_t*);

/***** Interrupt function prototypes *****/
void PDM_DMA_Callback(bool);

#ifdef __cplusplus
}
#endif

#endif /* __PDM_SOFTCICFILTER_H */
/***************************************************************END OF FILE****/
