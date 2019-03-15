#include "tim.h"
#include "BSP_tim.h"

extern TIM_HandleTypeDef htim6;
/**
 * @brief Enable timer6
 * @param None
 * @return None
 * @attention None
 */
void TIM_Enable (void)
{

	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start_IT(&htim6);
	
}


