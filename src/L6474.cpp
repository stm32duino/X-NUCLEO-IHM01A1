/**
  ******************************************************************************
  * @file    L6474.cpp
  * @author  IPC Rennes
  * @version V1.5.0
  * @date    November 12, 2014
  * @brief   L6474 driver (fully integrated microstepping motor driver)
  * @note    (C) COPYRIGHT 2014 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Generated with STM32CubeTOO -----------------------------------------------*/


/* Revision ------------------------------------------------------------------*/
/*
    Repository:       http://svn.x-nucleodev.codex.cro.st.com/svnroot/X-NucleoDev
    Branch/Trunk/Tag: trunk
    Based on:         X-CUBE-SPN1/trunk/Drivers/BSP/Components/l6474/l6474.c
    Revision:         0
*/


/* Includes ------------------------------------------------------------------*/

#include "L6474.h"

    
/* Definitions ---------------------------------------------------------------*/

/* Error while initialising the SPI. */
#define L6474_ERROR_0        (0x8000)   

/* Error of bad SPI transaction. */
#define L6474_ERROR_1        (0x8001)
    
/* Maximum number of steps. */
#define MAX_STEPS            (0x7FFFFFFF)

/* Maximum frequency of the PWMs in Hz. */
#define L6474_MAX_PWM_FREQ   (10000)

/* Minimum frequency of the PWMs in Hz. */
#define L6474_MIN_PWM_FREQ   (2)


/* Variables  ----------------------------------------------------------------*/

/* Number of devices. */
uint8_t L6474::number_of_devices = 0;

/* ISR flags used to restart an interrupted SPI transfer when an error is reported. */
bool L6474::spi_preemtion_by_isr = FALSE;
bool L6474::isr_flag = FALSE;

/* SPI Transmission for Daisy-Chain Configuration. */
uint8_t L6474::spi_tx_bursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
uint8_t L6474::spi_rx_bursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];


/* Methods -------------------------------------------------------------------*/

/**********************************************************
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error Hanlder
 * @retval None
 **********************************************************/
void L6474::L6474_AttachErrorHandler(void (*callback)(uint16_t error))
{
  error_handler_callback = (void (*)(uint16_t error)) callback;
}

/**********************************************************
 * @brief Starts the L6474 library
 * @param  init Initialization structure.
 * @retval COMPONENT_OK in case of success.
 **********************************************************/
status_t L6474::L6474_Init(void *init)
{
  /* Initialise the PWMs used for the Step clocks ----------------------------*/
  L6474_PwmInit();

  /* Initialise the L6474s ------------------------------------------------*/
  
  /* Standby-reset deactivation */
  L6474_ReleaseReset();
  
  /* Let a delay after reset */
  L6474_Delay(1); 

  /* Set device parameters to the predefined values from "l6474_target_config.h". */
  L6474_SetDeviceParamsToPredefinedValues();
  
  if (init == NULL)
    /* Set device registers to the predefined values from "l6474_target_config.h". */
    L6474_SetRegisterToPredefinedValues();
  else
    /* Set device registers to the passed initialization values. */
    L6474_SetRegisterToInitializationValues((L6474_init_t *) init);
  
  /* Disable L6474 powerstage */
  L6474_CmdDisable();

  /* Get Status to clear flags after start up */
  L6474_CmdGetStatus();
  
  return COMPONENT_OK;
}

/**********************************************************
 * @brief Read id
 * @param id pointer to the identifier to be read.
 * @retval COMPONENT_OK in case of success.
 **********************************************************/
status_t L6474::L6474_ReadID(uint8_t *id)
{
  *id = device_instance;

  return COMPONENT_OK;
}

/**********************************************************
 * @brief Returns the acceleration of the specified device
 * @retval Acceleration in pps^2
 **********************************************************/
uint16_t L6474::L6474_GetAcceleration(void)
{
  return (device_prm.acceleration);
}            

/**********************************************************
 * @brief Returns the current speed of the specified device
 * @retval Speed in pps
 **********************************************************/
uint16_t L6474::L6474_GetCurrentSpeed(void)
{
  return device_prm.speed;
}

/**********************************************************
 * @brief Returns the deceleration of the specified device
 * @retval Deceleration in pps^2
 **********************************************************/
uint16_t L6474::L6474_GetDeceleration(void)
{                                                  
  return (device_prm.deceleration);
}          

/**********************************************************
 * @brief Returns the device state
 * @retval State (ACCELERATING, DECELERATING, STEADY or INACTIVE)
 **********************************************************/
motorState_t L6474::L6474_GetDeviceState(void)
{
  return device_prm.motionState;
}

/**********************************************************
 * @brief Returns the FW version of the library
 * @param None
 * @retval L6474_FW_VERSION
 **********************************************************/
uint8_t L6474::L6474_GetFwVersion(void)
{
  return (L6474_FW_VERSION);
}

/**********************************************************
 * @brief  Returns the mark position  of the specified device
 * @retval Mark register value converted in a 32b signed integer 
 **********************************************************/
int32_t L6474::L6474_GetMark(void)
{
  return L6474_ConvertPosition(L6474_CmdGetParam(L6474_MARK));
}

/**********************************************************
 * @brief  Returns the max speed of the specified device
 * @retval maxSpeed in pps
 **********************************************************/
uint16_t L6474::L6474_GetMaxSpeed(void)
{                                                  
  return (device_prm.maxSpeed);
}

/**********************************************************
 * @brief  Returns the min speed of the specified device
 * @retval minSpeed in pps
 **********************************************************/
uint16_t L6474::L6474_GetMinSpeed(void)
{                                                  
  return (device_prm.minSpeed);
}                                                     

/**********************************************************
 * @brief  Returns the ABS_POSITION of the specified device
 * @retval ABS_POSITION register value converted in a 32b signed integer
 **********************************************************/
int32_t L6474::L6474_GetPosition(void)
{
  return L6474_ConvertPosition(L6474_CmdGetParam(L6474_ABS_POS));
}

/**********************************************************
 * @brief  Requests the motor to move to the home position (ABS_POSITION = 0)
 * @retval None
 **********************************************************/
void L6474::L6474_GoHome(void)
{
  L6474_GoTo(0);
} 
  
/**********************************************************
 * @brief  Requests the motor to move to the mark position 
 * @retval None
 **********************************************************/
void L6474::L6474_GoMark(void)
{
    uint32_t mark;

    mark = L6474_ConvertPosition(L6474_CmdGetParam(L6474_MARK));
    L6474_GoTo(mark);  
}

/**********************************************************
 * @brief  Requests the motor to move to the specified position 
 * @param[in] targetPosition absolute position in steps
 * @retval None
 **********************************************************/
void L6474::L6474_GoTo(int32_t targetPosition)
{
  motorDir_t direction;
  int32_t steps;
  
  /* Eventually deactivate motor */
  if (device_prm.motionState != INACTIVE) 
  {
    L6474_HardStop();
  }

  /* Get current position */
  device_prm.currentPosition = L6474_ConvertPosition(L6474_CmdGetParam(L6474_ABS_POS));
  
  /* Compute the number of steps to perform */
  steps = targetPosition - device_prm.currentPosition;
  
  if (steps >= 0) 
  {
    device_prm.stepsToTake = steps;
    direction = FORWARD;
  } 
  else 
  {
    device_prm.stepsToTake = -steps;
    direction = BACKWARD;
  }
  
  if (steps != 0) 
  {
    device_prm.commandExecuted = MOVE_CMD;
        
    /* Direction setup */
    L6474_SetDirection(direction);

    L6474_ComputeSpeedProfile(device_prm.stepsToTake);
    
    /* Motor activation */
    L6474_StartMovement();
  }  
}

/**********************************************************
 * @brief  Immediatly stops the motor and disable the power bridge
 * @retval None
 **********************************************************/
void L6474::L6474_HardStop(void) 
{
  /* Disable corresponding PWM */
  L6474_PwmStop();

  /* Set inactive state */
  device_prm.motionState = INACTIVE;
  device_prm.commandExecuted = NO_CMD;
  device_prm.stepsToTake = MAX_STEPS;  
}

/**********************************************************
 * @brief  Moves the motor of the specified number of steps
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] stepCount Number of steps to perform
 * @retval None
 **********************************************************/
void L6474::L6474_Move(motorDir_t direction, uint32_t stepCount)
{
  /* Eventually deactivate motor */
  if (device_prm.motionState != INACTIVE) 
  {
    L6474_HardStop();
  }
  
  if (stepCount != 0) 
  {
    device_prm.stepsToTake = stepCount;
    
    device_prm.commandExecuted = MOVE_CMD;
    
    device_prm.currentPosition = L6474_ConvertPosition(L6474_CmdGetParam(L6474_ABS_POS));
    
    /* Direction setup */
    L6474_SetDirection(direction);

    L6474_ComputeSpeedProfile(stepCount);
    
    /* Motor activation */
    L6474_StartMovement();
  }  
}

/**********************************************************
 * @brief  Runs the motor. It will accelerate from the min 
 * speed up to the max speed by using the device acceleration.
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 **********************************************************/
void L6474::L6474_Run(motorDir_t direction)
{
  /* Eventually deactivate motor */
  if (device_prm.motionState != INACTIVE) 
  {
    L6474_HardStop();
  }
  
  /* Direction setup */
  L6474_SetDirection(direction);

  device_prm.commandExecuted = RUN_CMD;

  /* Motor activation */
  L6474_StartMovement(); 
}

/**********************************************************
 * @brief  Changes the acceleration of the specified device
 * @param[in] newAcc New acceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool L6474::L6474_SetAcceleration(uint16_t newAcc)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newAcc != 0)&&
      ((device_prm.motionState == INACTIVE)||
       (device_prm.commandExecuted == RUN_CMD)))
  {
    device_prm.acceleration = newAcc;
    cmdExecuted = TRUE;
  }    
  return cmdExecuted;
}            

/**********************************************************
 * @brief  Changes the deceleration of the specified device
 * @param[in] newDec New deceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool L6474::L6474_SetDeceleration(uint16_t newDec)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newDec != 0)&& 
      ((device_prm.motionState == INACTIVE)||
       (device_prm.commandExecuted == RUN_CMD)))
  {
    device_prm.deceleration = newDec;
    cmdExecuted = TRUE;
  }      
  return cmdExecuted;
}        

/**********************************************************
 * @brief  Set current position to be the Home position (ABS pos set to 0)
 * @retval None
 **********************************************************/
void L6474::L6474_SetHome(void)
{
  L6474_CmdSetParam(L6474_ABS_POS, 0);
}
 
/**********************************************************
 * @brief  Sets current position to be the Mark position 
 * @retval None
 **********************************************************/
void L6474::L6474_SetMark(void)
{
  uint32_t mark = L6474_CmdGetParam(L6474_ABS_POS);
  L6474_CmdSetParam(L6474_MARK, mark);
}

/**********************************************************
 * @brief  Changes the max speed of the specified device
 * @param[in] newMaxSpeed New max speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool L6474::L6474_SetMaxSpeed(uint16_t newMaxSpeed)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newMaxSpeed >= L6474_MIN_PWM_FREQ)&&
      (newMaxSpeed <= L6474_MAX_PWM_FREQ) &&
      (device_prm.minSpeed <= newMaxSpeed) &&
      ((device_prm.motionState == INACTIVE)||
       (device_prm.commandExecuted == RUN_CMD)))
  {
    device_prm.maxSpeed = newMaxSpeed;
    cmdExecuted = TRUE;
  }
  return cmdExecuted;
}                                                     

/**********************************************************
 * @brief  Changes the min speed of the specified device
 * @param[in] newMinSpeed New min speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool L6474::L6474_SetMinSpeed(uint16_t newMinSpeed)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newMinSpeed >= L6474_MIN_PWM_FREQ)&&
      (newMinSpeed <= L6474_MAX_PWM_FREQ) &&
      (newMinSpeed <= device_prm.maxSpeed) && 
      ((device_prm.motionState == INACTIVE)||
       (device_prm.commandExecuted == RUN_CMD)))
  {
    device_prm.minSpeed = newMinSpeed;
    cmdExecuted = TRUE;
  }  
  return cmdExecuted;
}                 

/**********************************************************
 * @brief  Stops the motor by using the device deceleration
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is in INACTIVE state.
 **********************************************************/
bool L6474::L6474_SoftStop(void)
{   
  bool cmdExecuted = FALSE;
  if (device_prm.motionState != INACTIVE)
  {
    device_prm.commandExecuted = SOFT_STOP_CMD;
    cmdExecuted = TRUE;
  }
  return (cmdExecuted);
}

/**********************************************************
 * @brief  Locks until the device state becomes Inactive
 * @retval None
 **********************************************************/
void L6474::L6474_WaitWhileActive(void)
{
  /* Wait while motor is running */
  while (L6474_GetDeviceState() != INACTIVE);
}

/**********************************************************
 * @brief  Issue the Disable command to the L6474 of the specified device
 * @retval None
 **********************************************************/
void L6474::L6474_CmdDisable(void)
{
  L6474_SendCommand(L6474_DISABLE);
}

/**********************************************************
 * @brief  Issues the Enable command to the L6474 of the specified device
 * @retval None
 **********************************************************/
void L6474::L6474_CmdEnable(void)
{
  L6474_SendCommand(L6474_ENABLE);
}

/**********************************************************
 * @brief  Issues the GetParam command to the L6474 of the specified device
 * @param[in] parameter Register adress (L6474_ABS_POS, L6474_MARK,...)
 * @retval Register value
 **********************************************************/
uint32_t L6474::L6474_CmdGetParam(L6474_Registers_t parameter)
{
  uint32_t i;
  uint32_t spiRxData;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = number_of_devices - device_instance - 1;
  bool itDisable = FALSE;  
  
  do
  {
    spi_preemtion_by_isr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_EnableIrq if disable in previous iteration */
      L6474_EnableIrq();
      itDisable = FALSE;
    }
  
    for (i = 0; i < number_of_devices; i++)
    {
      spi_tx_bursts[0][i] = L6474_NOP;
      spi_tx_bursts[1][i] = L6474_NOP;
      spi_tx_bursts[2][i] = L6474_NOP;
      spi_tx_bursts[3][i] = L6474_NOP;
      spi_rx_bursts[1][i] = 0;
      spi_rx_bursts[2][i] = 0;
      spi_rx_bursts[3][i] = 0;    
    }

    switch (parameter)
    {
      case L6474_ABS_POS: ;
      case L6474_MARK:
        spi_tx_bursts[0][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (parameter);
        maxArgumentNbBytes = 3;
        break;
      case L6474_EL_POS: ;
      case L6474_CONFIG: ;
      case L6474_STATUS:
        spi_tx_bursts[1][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (parameter);
        maxArgumentNbBytes = 2;
        break;
      default:
        spi_tx_bursts[2][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (parameter);
        maxArgumentNbBytes = 1;
    }
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_DisableIrq();
    itDisable = TRUE;
  } while (spi_preemtion_by_isr); // check pre-emption by ISR
    
  for (i = L6474_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6474_CMD_ARG_MAX_NB_BYTES;
       i++)
  {
     L6474_WriteBytes(&spi_tx_bursts[i][0], &spi_rx_bursts[i][0]);
  }
  
  spiRxData = ((uint32_t)spi_rx_bursts[1][spiIndex] << 16) |
              (spi_rx_bursts[2][spiIndex] << 8) |
              (spi_rx_bursts[3][spiIndex]);
  
  /* re-enable L6474_EnableIrq after SPI transfers*/
  L6474_EnableIrq();
    
  return (spiRxData);
}

/**********************************************************
 * @brief  Issues the GetStatus command to the L6474 of the specified device
 * @retval Status Register value
 * @note Once the GetStatus command is performed, the flags of the status register
 * are reset. This is not the case when the status register is read with the
 * GetParam command (via the functions L6474ReadStatusRegister or L6474_CmdGetParam).
 **********************************************************/
uint16_t L6474::L6474_CmdGetStatus(void)
{
  uint32_t i;
  uint16_t status;
  uint8_t spiIndex = number_of_devices - device_instance - 1;
  bool itDisable = FALSE;  
  
  do
  {
    spi_preemtion_by_isr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_EnableIrq if disable in previous iteration */
      L6474_EnableIrq();
      itDisable = FALSE;
    }

    for (i = 0; i < number_of_devices; i++)
    {
       spi_tx_bursts[0][i] = L6474_NOP;
       spi_tx_bursts[1][i] = L6474_NOP;
       spi_tx_bursts[2][i] = L6474_NOP;
       spi_rx_bursts[1][i] = 0;
       spi_rx_bursts[2][i] = 0;
    }
    spi_tx_bursts[0][spiIndex] = L6474_GET_STATUS;

    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_DisableIrq();
    itDisable = TRUE;
  } while (spi_preemtion_by_isr); // check pre-emption by ISR

  for (i = 0; i < L6474_CMD_ARG_NB_BYTES_GET_STATUS + L6474_RSP_NB_BYTES_GET_STATUS; i++)
  {
     L6474_WriteBytes(&spi_tx_bursts[i][0], &spi_rx_bursts[i][0]);
  }
  status = (spi_rx_bursts[1][spiIndex] << 8) | (spi_rx_bursts[2][spiIndex]);
  
  /* re-enable L6474_EnableIrq after SPI transfers*/
  L6474_EnableIrq();
  
  return (status);
}

/**********************************************************
 * @brief  Issues the Nop command to the L6474 of the specified device
 * @retval None
 **********************************************************/
void L6474::L6474_CmdNop(void)
{
  L6474_SendCommand(L6474_NOP);
}

/**********************************************************
 * @brief  Issues the SetParam command to the L6474 of the specified device
 * @param[in] parameter Register adress (L6474_ABS_POS, L6474_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 **********************************************************/
void L6474::L6474_CmdSetParam(L6474_Registers_t parameter, uint32_t value)
{
  uint32_t i;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = number_of_devices - device_instance - 1;
  bool itDisable = FALSE;  
  do
  {
    spi_preemtion_by_isr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_EnableIrq if disable in previous iteration */
      L6474_EnableIrq();
      itDisable = FALSE;
    }

    for (i = 0; i < number_of_devices; i++)
    {
      spi_tx_bursts[0][i] = L6474_NOP;
      spi_tx_bursts[1][i] = L6474_NOP;
      spi_tx_bursts[2][i] = L6474_NOP;
      spi_tx_bursts[3][i] = L6474_NOP;
    }

    switch (parameter)
    {
      case L6474_ABS_POS: ;
      case L6474_MARK:
          spi_tx_bursts[0][spiIndex] = parameter;
          spi_tx_bursts[1][spiIndex] = (uint8_t)(value >> 16);
          spi_tx_bursts[2][spiIndex] = (uint8_t)(value >> 8);
          maxArgumentNbBytes = 3;
          break;
      case L6474_EL_POS: ;
      case L6474_CONFIG:
          spi_tx_bursts[1][spiIndex] = parameter;
          spi_tx_bursts[2][spiIndex] = (uint8_t)(value >> 8);
          maxArgumentNbBytes = 2;
          break;
      default:
          spi_tx_bursts[2][spiIndex] = parameter;
          maxArgumentNbBytes = 1;
          break;
    }
    spi_tx_bursts[3][spiIndex] = (uint8_t)(value);
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_DisableIrq();
    itDisable = TRUE;
  } while (spi_preemtion_by_isr); // check pre-emption by ISR
 
  /* SPI transfer */
  for (i = L6474_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6474_CMD_ARG_MAX_NB_BYTES;
       i++)
  {
     L6474_WriteBytes(&spi_tx_bursts[i][0],&spi_rx_bursts[i][0]);
  }
  /* re-enable L6474_EnableIrq after SPI transfers*/
  L6474_EnableIrq();
}

/**********************************************************
 * @brief  Reads the Status Register value
 * @retval Status register valued
 * @note The status register flags are not cleared 
 * at the difference with L6474CmdGetStatus()
 **********************************************************/
uint16_t L6474::L6474_ReadStatusRegister(void)
{
  return (L6474_CmdGetParam(L6474_STATUS));
}

/**********************************************************
 * @brief  Set the stepping mode 
 * @param[in] stepMod from full step to 1/16 microstep as specified in enum motorStepMode_t
 * @retval None
 **********************************************************/
void L6474::L6474_SelectStepMode(motorStepMode_t stepMod)
{
  uint8_t stepModeRegister;
  L6474_STEP_SEL_t l6474StepMod;
  
  switch (stepMod)
  {
    case STEP_MODE_FULL:
      l6474StepMod = L6474_STEP_SEL_1;
      break;
    case STEP_MODE_HALF:
      l6474StepMod = L6474_STEP_SEL_1_2;
      break;    
    case STEP_MODE_1_4:
      l6474StepMod = L6474_STEP_SEL_1_4;
      break;        
    case STEP_MODE_1_8:
      l6474StepMod = L6474_STEP_SEL_1_8;
      break;       
    case STEP_MODE_1_16:
    default:
      l6474StepMod = L6474_STEP_SEL_1_16;
      break;       
  }

  /* Eventually deactivate motor */
  if (device_prm.motionState != INACTIVE) 
  {
    L6474_HardStop();
  }
  
  /* Read Step mode register and clear STEP_SEL field */
  stepModeRegister = (uint8_t)(0xF8 & L6474_CmdGetParam(L6474_STEP_MODE)) ;
  
  /* Apply new step mode */
  L6474_CmdSetParam(L6474_STEP_MODE, stepModeRegister | (uint8_t)l6474StepMod);

  /* Reset abs pos register */
  L6474_SetHome();
}

/**********************************************************
 * @brief  Get the direction
 * @param  None
 * @retval direction FORWARD or BACKWARD
 **********************************************************/
motorDir_t L6474::L6474_GetDirection(void)
{
  return device_prm.direction;
}

/**********************************************************
 * @brief  Specifies the direction 
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is only applied if the device 
 * is in INACTIVE state
 * @retval None
 **********************************************************/
void L6474::L6474_SetDirection(motorDir_t direction)
{
  if (device_prm.motionState == INACTIVE)
  {
    device_prm.direction = direction;
    L6474_SetDirectionGpio(direction);
  }
}

/**********************************************************
 * @brief  Updates the current speed of the device
 * @param[in] newSpeed in pps
 * @retval None
 **********************************************************/
void L6474::L6474_ApplySpeed(uint16_t newSpeed)
{
  if (newSpeed < L6474_MIN_PWM_FREQ)
  {
    newSpeed = L6474_MIN_PWM_FREQ;  
  }
  if (newSpeed > L6474_MAX_PWM_FREQ)
  {
    newSpeed = L6474_MAX_PWM_FREQ;
  }
  
  device_prm.speed = newSpeed;

  L6474_PwmSetFreq(newSpeed);
}

/**********************************************************
 * @brief  Computes the speed profile according to the number of steps to move
 * @param[in] nbSteps number of steps to perform
 * @retval None
 * @note Using the acceleration and deceleration of the device,
 * this function determines the duration in steps of the acceleration,
 * steady and deceleration phases.
 * If the total number of steps to perform is big enough, a trapezoidal move
 * is performed (i.e. there is a steady phase where the motor runs at the maximum
 * speed.
 * Else, a triangular move is performed (no steady phase: the maximum speed is never
 * reached.
 **********************************************************/
void L6474::L6474_ComputeSpeedProfile(uint32_t nbSteps)
{
  uint32_t reqAccSteps; 
  uint32_t reqDecSteps;
   
  /* compute the number of steps to get the targeted speed */
  uint16_t minSpeed = device_prm.minSpeed;
  reqAccSteps = (device_prm.maxSpeed - minSpeed);
  reqAccSteps *= (device_prm.maxSpeed + minSpeed);
  reqDecSteps = reqAccSteps;
  reqAccSteps /= (uint32_t)device_prm.acceleration;
  reqAccSteps /= 2;

  /* compute the number of steps to stop */
  reqDecSteps /= (uint32_t)device_prm.deceleration;
  reqDecSteps /= 2;

  if(( reqAccSteps + reqDecSteps ) > nbSteps)
  { 
    /* Triangular move  */
    /* reqDecSteps = (Pos * Dec) /(Dec+Acc) */
    uint32_t dec = device_prm.deceleration;
    uint32_t acc = device_prm.acceleration;
    
    reqDecSteps =  ((uint32_t) dec * nbSteps) / (acc + dec);
    if (reqDecSteps > 1)
    {
      reqAccSteps = reqDecSteps - 1;
      if(reqAccSteps == 0)
      {
        reqAccSteps = 1;
      }      
    }
    else
    {
      reqAccSteps = 0;
    }
    device_prm.endAccPos = reqAccSteps;
    device_prm.startDecPos = reqDecSteps;
  }
  else
  {  
    /* Trapezoidal move */
    /* accelerating phase to endAccPos */
    /* steady phase from  endAccPos to startDecPos */
    /* decelerating from startDecPos to stepsToTake*/
    device_prm.endAccPos = reqAccSteps;
    device_prm.startDecPos = nbSteps - reqDecSteps - 1;
  }
}

/**********************************************************
 * @brief  Converts the ABS_POSITION register value to a 32b signed integer
 * @param[in] abs_position_reg value of the ABS_POSITION register
 * @retval operation_result 32b signed integer corresponding to the absolute position 
 **********************************************************/
int32_t L6474::L6474_ConvertPosition(uint32_t abs_position_reg)
{
  int32_t operation_result;

  if (abs_position_reg & L6474_ABS_POS_SIGN_BIT_MASK) 
  {
    /* Negative register value */
    abs_position_reg = ~abs_position_reg;
    abs_position_reg += 1;

    operation_result = (int32_t) (abs_position_reg & L6474_ABS_POS_VALUE_MASK);
    operation_result = -operation_result;
  } 
  else 
  {
    operation_result = (int32_t) abs_position_reg;
  }

  return operation_result;
}

/**********************************************************
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void L6474::L6474_ErrorHandler(uint16_t error)
{
  if (error_handler_callback != 0)
  {
    (void) error_handler_callback(error);
  }
  else   
  {
    /* Aborting the program. */
    exit(EXIT_FAILURE);
  }
}

/**********************************************************
 * @brief  Sends a command without arguments to the L6474 via the SPI
 * @param[in] param Command to send 
 * @retval None
 **********************************************************/
void L6474::L6474_SendCommand(uint8_t param)
{
  uint32_t i;
  bool itDisable = FALSE;
  uint8_t spiIndex = number_of_devices - device_instance - 1;
  
  do
  {
    spi_preemtion_by_isr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_EnableIrq if disable in previous iteration */
      L6474_EnableIrq();
      itDisable = FALSE;
    }
  
    for (i = 0; i < number_of_devices; i++)
    {
      spi_tx_bursts[3][i] = L6474_NOP;     
    }
    spi_tx_bursts[3][spiIndex] = param;
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_DisableIrq();
    itDisable = TRUE;
  } while (spi_preemtion_by_isr); // check pre-emption by ISR

  L6474_WriteBytes(&spi_tx_bursts[3][0], &spi_rx_bursts[3][0]); 
  
  /* re-enable L6474_EnableIrq after SPI transfers*/
  L6474_EnableIrq();
}

/**********************************************************
 * @brief  Sets the registers of the L6474 to their predefined values 
 * from l6474_target_config.h
 * @retval None
 **********************************************************/
void L6474::L6474_SetRegisterToPredefinedValues(void)
{
  L6474_CmdSetParam(
                    L6474_ABS_POS,
                    0);
  L6474_CmdSetParam(
                    L6474_EL_POS,
                    0);
  L6474_CmdSetParam(
                    L6474_MARK,
                    0);
  switch (device_instance)
  {
    case 0:
      L6474_CmdSetParam(
                        L6474_TVAL,
                        L6474_Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_DEVICE_0));
      L6474_CmdSetParam(
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_0 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_0);
      L6474_CmdSetParam(
                        L6474_TON_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_DEVICE_0)
                        );
      L6474_CmdSetParam(
                        L6474_TOFF_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_DEVICE_0));
      L6474_CmdSetParam(
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_0);
      L6474_CmdSetParam(
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_0 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_0);
      L6474_CmdSetParam(
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_0);
      L6474_CmdSetParam(
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_0);
      break;
    case 1:
      L6474_CmdSetParam(
                        L6474_TVAL,
                        L6474_Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_DEVICE_1));
      L6474_CmdSetParam(
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_1 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_1);
      L6474_CmdSetParam(
                        L6474_TON_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_DEVICE_1));
      L6474_CmdSetParam(
                        L6474_TOFF_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_DEVICE_1));
      L6474_CmdSetParam(
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_1);
      L6474_CmdSetParam(
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_1 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_1);
      L6474_CmdSetParam(
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_1);
      L6474_CmdSetParam(
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_1);
      break;
    case 2:
      L6474_CmdSetParam(
                        L6474_TVAL,
                        L6474_Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_DEVICE_2));
      L6474_CmdSetParam(
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_2 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_2);
      L6474_CmdSetParam(
                        L6474_TON_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_DEVICE_2));
      L6474_CmdSetParam(
                        L6474_TOFF_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_DEVICE_2));
      L6474_CmdSetParam(
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_2);
      L6474_CmdSetParam(
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_2 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_2);
      L6474_CmdSetParam(
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_2);
      L6474_CmdSetParam(
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_2);
      break;
    default: ;
  }
}

/**********************************************************
 * @brief  Sets the registers of the L6474 to initialization values.
 * @param  init Initialization structure.
 * @retval None.
 **********************************************************/
void L6474::L6474_SetRegisterToInitializationValues(L6474_init_t *init)
{
  L6474_CmdSetParam(
                    L6474_ABS_POS,
                    0
                    );
  L6474_CmdSetParam(
                    L6474_EL_POS,
                    0
                    );
  L6474_CmdSetParam(
                    L6474_MARK,
                    0
                    );
  L6474_CmdSetParam(
                    L6474_TVAL,
                    L6474_Tval_Current_to_Par(init->torque_regulation_current_mA)
                    );
  L6474_CmdSetParam(
                    L6474_T_FAST,
                    (uint8_t) init->maximum_fast_decay_time |
                    (uint8_t) init->fall_time
                    );
  L6474_CmdSetParam(
                    L6474_TON_MIN,
                    L6474_Tmin_Time_to_Par(init->minimum_ON_time_us)
                    );
  L6474_CmdSetParam(
                    L6474_TOFF_MIN,
                    L6474_Tmin_Time_to_Par(init->minimum_OFF_time_us)
                    );
  L6474_CmdSetParam(
                    L6474_OCD_TH,
                    init->overcurrent_threshold
                    );
  L6474_CmdSetParam(
                    L6474_STEP_MODE,
                    (uint8_t) init->step_selection |
                    (uint8_t) init->sync_selection
                    );
  L6474_CmdSetParam(
                    L6474_ALARM_EN,
                    init->alarm
                    );
  L6474_CmdSetParam(
                    L6474_CONFIG,
                    (uint16_t) init->clock |
                    (uint16_t) init->torque_regulation_method |
                    (uint16_t) init->overcurrent_shutwdown |
                    (uint16_t) init->slew_rate |
                    (uint16_t) init->target_swicthing_period
                    );
  L6474_SetAcceleration((uint16_t) init->acceleration_pps_2);
  L6474_SetDeceleration((uint16_t) init->deceleration_pps_2);
  L6474_SetMaxSpeed((uint16_t) init->maximum_speed_pps);
  L6474_SetMinSpeed((uint16_t) init->minimum_speed_pps);
}

/**********************************************************
 * @brief  Sets the parameters of the device to predefined values
 * from l6474_target_config.h
 * @param None
 * @retval None
 **********************************************************/
void L6474::L6474_SetDeviceParamsToPredefinedValues(void)
{
  switch (device_instance)
  {
    case 0:
      device_prm.acceleration = L6474_CONF_PARAM_ACC_DEVICE_0;
      device_prm.deceleration = L6474_CONF_PARAM_DEC_DEVICE_0;
      device_prm.maxSpeed = L6474_CONF_PARAM_MAX_SPEED_DEVICE_0;
      device_prm.minSpeed = L6474_CONF_PARAM_MIN_SPEED_DEVICE_0;
      break;

    case 1:
      device_prm.acceleration = L6474_CONF_PARAM_ACC_DEVICE_1;
      device_prm.deceleration = L6474_CONF_PARAM_DEC_DEVICE_1;
      device_prm.maxSpeed = L6474_CONF_PARAM_MAX_SPEED_DEVICE_1;
      device_prm.minSpeed = L6474_CONF_PARAM_MIN_SPEED_DEVICE_1;
      break;

    case 2:
      device_prm.acceleration = L6474_CONF_PARAM_ACC_DEVICE_2;
      device_prm.deceleration = L6474_CONF_PARAM_DEC_DEVICE_2;
      device_prm.maxSpeed = L6474_CONF_PARAM_MAX_SPEED_DEVICE_2;
      device_prm.minSpeed = L6474_CONF_PARAM_MIN_SPEED_DEVICE_2;
      break;
  }

  device_prm.accu = 0;
  device_prm.currentPosition = 0;
  device_prm.endAccPos = 0;
  device_prm.relativePos = 0;
  device_prm.startDecPos = 0;
  device_prm.stepsToTake = 0;
  device_prm.speed = 0;
  device_prm.commandExecuted = NO_CMD;
  device_prm.direction = FORWARD;
  device_prm.motionState = INACTIVE;
}

/**********************************************************
 * @brief Initialises the bridge parameters to start the movement
 * and enable the power bridge
 * @retval None
 **********************************************************/
void L6474::L6474_StartMovement(void)  
{
  /* Enable L6474 powerstage */
  L6474_CmdEnable();
  if (device_prm.endAccPos != 0)
  {
    device_prm.motionState = ACCELERATING;
  }
  else
  {
    device_prm.motionState = DECELERATING;    
  }
  device_prm.accu = 0;
  device_prm.relativePos = 0;
  L6474_ApplySpeed(device_prm.minSpeed);
}

/**********************************************************
 * @brief  Handles the device state machine at each ste
 * @retval None
 * @note Must only be called by the timer ISR
 **********************************************************/
void L6474::L6474_StepClockHandler(void)
{
  /* Set isr flag */
  isr_flag = TRUE;
  
  /* Incrementation of the relative position */
  device_prm.relativePos++;

  switch (device_prm.motionState) 
  {
    case ACCELERATING: 
    {
        uint32_t relPos = device_prm.relativePos;
        uint32_t endAccPos = device_prm.endAccPos;
        uint16_t speed = device_prm.speed;
        uint32_t acc = ((uint32_t)device_prm.acceleration << 16);
        
        if ((device_prm.commandExecuted == SOFT_STOP_CMD)||
            ((device_prm.commandExecuted != RUN_CMD)&&  
             (relPos == device_prm.startDecPos)))
        {
          device_prm.motionState = DECELERATING;
          device_prm.accu = 0;
        }
        else if ((speed >= device_prm.maxSpeed)||
                 ((device_prm.commandExecuted != RUN_CMD)&&
                  (relPos == endAccPos)))
        {
          device_prm.motionState = STEADY;
        }
        else
        {
          bool speedUpdated = FALSE;
          /* Go on accelerating */
          if (speed == 0) speed =1;
          device_prm.accu += acc / speed;
          while (device_prm.accu >= (0X10000L))
          {
            device_prm.accu -= (0X10000L);
            speed +=1;
            speedUpdated = TRUE;
          }
          
          if (speedUpdated)
          {
            if (speed > device_prm.maxSpeed)
            {
              speed = device_prm.maxSpeed;
            }    
            device_prm.speed = speed;
            L6474_ApplySpeed(device_prm.speed);
          }
        }
        break;
    }
    case STEADY: 
    {
      uint16_t maxSpeed = device_prm.maxSpeed;
      uint32_t relativePos = device_prm.relativePos;
      if  ((device_prm.commandExecuted == SOFT_STOP_CMD)||
           ((device_prm.commandExecuted != RUN_CMD)&&
            (relativePos >= (device_prm.startDecPos))) ||
           ((device_prm.commandExecuted == RUN_CMD)&&
            (device_prm.speed > maxSpeed)))
      {
        device_prm.motionState = DECELERATING;
        device_prm.accu = 0;
      }
      else if ((device_prm.commandExecuted == RUN_CMD)&&
               (device_prm.speed < maxSpeed))
      {
        device_prm.motionState = ACCELERATING;
        device_prm.accu = 0;
      }
      break;
    }
    case DECELERATING: 
    {
      uint32_t relativePos = device_prm.relativePos;
      uint16_t speed = device_prm.speed;
      uint32_t deceleration = ((uint32_t)device_prm.deceleration << 16);
      if (((device_prm.commandExecuted == SOFT_STOP_CMD)&&(speed <=  device_prm.minSpeed))||
          ((device_prm.commandExecuted != RUN_CMD)&&
           (relativePos >= device_prm.stepsToTake)))
      {
        /* Motion process complete */
        L6474_HardStop();
      }
      else if ((device_prm.commandExecuted == RUN_CMD)&&
               (speed <= device_prm.maxSpeed))
      {
        device_prm.motionState = STEADY;
      }
      else
      {
        /* Go on decelerating */
        if (speed > device_prm.minSpeed)
        {
          bool speedUpdated = FALSE;
          if (speed == 0) speed =1;
          device_prm.accu += deceleration / speed;
          while (device_prm.accu >= (0X10000L))
          {
            device_prm.accu -= (0X10000L);
            if (speed > 1)
            {  
              speed -=1;
            }
            speedUpdated = TRUE;
          }
        
          if (speedUpdated)
          {
            if (speed < device_prm.minSpeed)
            {
              speed = device_prm.minSpeed;
            }  
            device_prm.speed = speed;
            L6474_ApplySpeed(device_prm.speed);
          }
        }
      }
      break;
    }
    default: 
    {
      break;
    }
  }  
  /* Set isr flag */
  isr_flag = FALSE;
}

/**********************************************************
 * @brief Converts current in mA to values for TVAL register 
 * @param[in] current_mA current in mA
 * @retval value for TVAL register
 **********************************************************/
float L6474::L6474_Tval_Current_to_Par(float current_mA)
{
  return ((float)(((current_mA - 31.25f) / 31.25f) + 0.5f));
}

/**********************************************************
 * @brief Converts values from TVAL register to mA
 * @param[in] Tval value from TVAL register
 * @retval current in mA
 **********************************************************/
float L6474::L6474_Par_to_Tval_Current(float Tval)
{
  return ((float)((Tval - 0.5f) * 31.25f + 31.25f));
}

/**********************************************************
 * @brief Convert time in us to values for TON_MIN register
 * @param[in] ton_min_us time in us
 * @retval value for TON_MIN register
 **********************************************************/
float L6474::L6474_Tmin_Time_to_Par(float ton_min_us)
{
  return ((float)(((ton_min_us - 0.5f) * 2.0f) + 0.5f));
}

/**********************************************************
 * @brief Convert values for TON_MIN register to time in us
 * @param[in] Tmin value from TON_MIN register
 * @retval time in us
 **********************************************************/
float L6474::L6474_Par_to_Tmin_Time(float Tmin)
{
  return ((float)(((Tmin - 0.5f) / 2.0f) + 0.5f));
}

/**********************************************************
 * @brief  Write and receive a byte via SPI
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte pointer to the received byte
 * @retval None
 **********************************************************/
void L6474::L6474_WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
{
  if (L6474_SpiWriteBytes(pByteToTransmit, pReceivedByte) != 0)
  {
    L6474_ErrorHandler(L6474_ERROR_1);
  }
  
  if (isr_flag)
  {
    spi_preemtion_by_isr = TRUE;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
