/**
 ******************************************************************************
 * @file    L6474.h
 * @author  SRA
 * @version V1.0.0
 * @date    July 2021
 * @brief   This file contains the class of an L6474 Motor Control component.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
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
    Based on:         X-CUBE-SPN1/trunk/Drivers/BSP/Components/l6474/l6474.h
    Revision:         0
*/


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __L6474_CLASS_H
#define __L6474_CLASS_H


/* Includes ------------------------------------------------------------------*/

/* ACTION 1 ------------------------------------------------------------------*
 * Include here platform specific header files.                               *
 *----------------------------------------------------------------------------*/
#include "Arduino.h"
#include "SPI.h"
/* ACTION 2 ------------------------------------------------------------------*
 * Include here component specific header files.                              *
 *----------------------------------------------------------------------------*/
#include "L6474_def.h"
/* ACTION 3 ------------------------------------------------------------------*
 * Include here interface specific header files.                              *
 *                                                                            *
 * Example:                                                                   *
 *   #include "HumiditySensor.h"                                              *
 *   #include "TemperatureSensor.h"                                           *
 *----------------------------------------------------------------------------*/
#include "StepperMotor.h"

/* Typedefs ------------------------------------------------------------------*/
template <typename T>
struct Callback;

template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
  template <typename... Args>
  static Ret callback(Args... args)
  {
    return func(args...);
  }
  static std::function<Ret(Params...)> func;
};

template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;

typedef void (*PwmHandler_Callback)(void);

/* Classes -------------------------------------------------------------------*/

/**
 * @brief Class representing an L6474 component.
 */
class L6474 : public StepperMotor {
  public:

    /*** Constructor and Destructor Methods ***/

    /**
     * @brief Constructor.
     * @param flag_irq      pin name of the FLAG pin of the component.
     * @param standby_reset pin name of the STBY\RST pin of the component.
     * @param direction     pin name of the DIR pin of the component.
     * @param pwm           pin name of the PWM pin of the component.
     * @param ssel          pin name of the SSEL pin of the SPI device to be used for communication.
     * @param spi           SPI device to be used for communication.
     */
    L6474(uint8_t flag_irq, uint8_t standby_reset, uint8_t direction, uint8_t pwm_pin, uint8_t ssel, SPIClass *spi, uint32_t spi_speed = 4000000) : StepperMotor(), flag_irq(flag_irq), standby_reset(standby_reset), direction(direction), pwm_pin(pwm_pin), ssel(ssel), dev_spi(spi), spi_speed(spi_speed)
    {
      /* Checking stackability. */
      if (!(number_of_devices < MAX_NUMBER_OF_DEVICES)) {
        while (1) {
          delay(1);
        }
      }

      pinMode(ssel, OUTPUT);
      digitalWrite(ssel, HIGH);
      pinMode(standby_reset, OUTPUT);
      digitalWrite(standby_reset, LOW);
      pinMode(direction, OUTPUT);
      digitalWrite(direction, LOW);
      pinMode(flag_irq, INPUT_PULLUP);

      Callback<void()>::func = std::bind(&L6474::L6474_StepClockHandler, this);
      callback_handler = static_cast<PwmHandler_Callback>(Callback<void()>::callback);

      pwm_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwm_pin), PinMap_PWM);
      pwm_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwm_pin), PinMap_PWM));

      pwm_timer = new HardwareTimer(pwm_instance);
      first_time_pwm = true;

      /* ACTION 4 ----------------------------------------------------------*
       * Initialize here the component's member variables, one variable per *
       * line.                                                              *
       *                                                                    *
       * Example:                                                           *
       *   measure = 0;                                                     *
       *   instance_id = number_of_instances++;                             *
       *--------------------------------------------------------------------*/
      error_handler_callback = 0;
      device_instance = number_of_devices++;
      memset(spi_tx_bursts, 0, L6474_CMD_ARG_MAX_NB_BYTES * MAX_NUMBER_OF_DEVICES * sizeof(uint8_t));
      memset(spi_rx_bursts, 0, L6474_CMD_ARG_MAX_NB_BYTES * MAX_NUMBER_OF_DEVICES * sizeof(uint8_t));
    }

    /**
     * @brief Destructor.
     */
    virtual ~L6474(void)
    {
      free(pwm_timer);
      number_of_devices--;
    }


    /*** Public Component Related Methods ***/

    /* ACTION 5 --------------------------------------------------------------*
     * Implement here the component's public methods, as wrappers of the C    *
     * component's functions.                                                 *
     * They should be:                                                        *
     *   + Methods with the same name of the C component's virtual table's    *
     *     functions (1);                                                     *
     *   + Methods with the same name of the C component's extended virtual   *
     *     table's functions, if any (2).                                     *
     *                                                                        *
     * Example:                                                               *
     *   virtual int get_value(float *p_data) //(1)                           *
     *   {                                                                    *
     *     return COMPONENT_get_value(float *pf_data);                        *
     *   }                                                                    *
     *                                                                        *
     *   virtual int enable_feature(void) //(2)                               *
     *   {                                                                    *
     *     return COMPONENT_enable_feature();                                 *
     *   }                                                                    *
     *------------------------------------------------------------------------*/
    /**
     * @brief  Initializing the component in 1/16 Microstepping mode.
     * @param  init Pointer to device specific initialization structure.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int init(void *init = NULL)
    {
      return (int) L6474_Init((void *) init);
    }

    /**
     * @brief  Getting the ID of the component.
     * @param  id Pointer to an allocated variable to store the ID into.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int read_id(uint8_t *id = NULL)
    {
      return (int) L6474_ReadID((uint8_t *) id);
    }

    /**
     * @brief  Getting the value of the Status Register.
     * @param  None.
     * @retval None.
     * @note   The Status Register's flags are cleared, contrary to the
     *         read_status_register() method.
     */
    virtual unsigned int get_status(void)
    {
      return (unsigned int) L6474_CmdGetStatus();
    }

    /**
     * @brief  Getting a parameter.
     * @param  parameter A parameter's register address.
     * @retval The parameter's value.
     * @note   The Status Register's flags are cleared, contrary to the
     *         read_status_register() method.
     *         The parameter can be one of the following:
     *           + L6474_ABS_POS
     *           + L6474_EL_POS
     *           + L6474_MARK
     *           + L6474_RESERVED_REG01
     *           + L6474_RESERVED_REG02
     *           + L6474_RESERVED_REG03
     *           + L6474_RESERVED_REG04
     *           + L6474_RESERVED_REG05
     *           + L6474_RESERVED_REG06
     *           + L6474_TVAL           : value in mA
     *           + L6474_RESERVED_REG07
     *           + L6474_RESERVED_REG08
     *           + L6474_RESERVED_REG09
     *           + L6474_RESERVED_REG10
     *           + L6474_T_FAST
     *           + L6474_TON_MIN        : value in us
     *           + L6474_TOFF_MIN       : value in us
     *           + L6474_RESERVED_REG11
     *           + L6474_ADC_OUT
     *           + L6474_OCD_TH
     *           + L6474_RESERVED_REG12
     *           + L6474_STEP_MODE
     *           + L6474_ALARM_EN
     *           + L6474_CONFIG
     *           + L6474_STATUS
     *           + L6474_RESERVED_REG13
     *           + L6474_RESERVED_REG14
     *           + L6474_INEXISTENT_REG
     */
    virtual float get_parameter(unsigned int parameter)
    {
      unsigned int register_value = (unsigned int) L6474_CmdGetParam((L6474_Registers_t) parameter);
      float value;

      switch ((L6474_Registers_t) parameter) {
        case L6474_TVAL:
          value = L6474_Par_to_Tval_Current((float) register_value);
          break;
        case L6474_TON_MIN:
        case L6474_TOFF_MIN:
          value = L6474_Par_to_Tmin_Time((float) register_value);
          break;
        default:
          value = (float) register_value;
          break;
      }

      return value;
    }

    /**
     * @brief  Getting the position.
     * @param  None.
     * @retval The position.
     */
    virtual signed int get_position(void)
    {
      return (signed int) L6474_GetPosition();
    }

    /**
     * @brief  Getting the marked position.
     * @param  None.
     * @retval The marked position.
     */
    virtual signed int get_mark(void)
    {
      return (signed int) L6474_GetMark();
    }

    /**
     * @brief  Getting the current speed in pps.
     * @param  None.
     * @retval The current speed in pps.
     */
    virtual unsigned int get_speed(void)
    {
      return (unsigned int) L6474_GetCurrentSpeed();
    }

    /**
     * @brief  Getting the maximum speed in pps.
     * @param  None.
     * @retval The maximum speed in pps.
     */
    virtual unsigned int get_max_speed(void)
    {
      return (unsigned int) L6474_GetMaxSpeed();
    }

    /**
     * @brief  Getting the minimum speed in pps.
     * @param  None.
     * @retval The minimum speed in pps.
     */
    virtual unsigned int get_min_speed(void)
    {
      return (unsigned int) L6474_GetMinSpeed();
    }

    /**
     * @brief  Getting the acceleration in pps^2.
     * @param  None.
     * @retval The acceleration in pps^2.
     */
    virtual unsigned int get_acceleration(void)
    {
      return (unsigned int) L6474_GetAcceleration();
    }

    /**
     * @brief  Getting the deceleration in pps^2.
     * @param  None.
     * @retval The deceleration in pps^2.
     */
    virtual unsigned int get_deceleration(void)
    {
      return (unsigned int) L6474_GetDeceleration();
    }

    /**
     * @brief  Getting the direction of rotation.
     * @param  None.
     * @retval The direction of rotation.
     */
    virtual direction_t get_direction(void)
    {
      return (direction_t)(L6474_GetDirection() == FORWARD ? StepperMotor::FWD : StepperMotor::BWD);
    }

    /**
     * @brief   Setting a parameter.
     * @param   parameter A parameter's register address.
     * @param   value The parameter's value.
     * @retval  None.
     * @note    The parameter can be one of the following:
     *           + L6474_ABS_POS
     *           + L6474_EL_POS
     *           + L6474_MARK
     *           + L6474_RESERVED_REG01
     *           + L6474_RESERVED_REG02
     *           + L6474_RESERVED_REG03
     *           + L6474_RESERVED_REG04
     *           + L6474_RESERVED_REG05
     *           + L6474_RESERVED_REG06
     *           + L6474_TVAL           : value in mA
     *           + L6474_RESERVED_REG07
     *           + L6474_RESERVED_REG08
     *           + L6474_RESERVED_REG09
     *           + L6474_RESERVED_REG10
     *           + L6474_T_FAST
     *           + L6474_TON_MIN        : value in us
     *           + L6474_TOFF_MIN       : value in us
     *           + L6474_RESERVED_REG11
     *           + L6474_ADC_OUT
     *           + L6474_OCD_TH
     *           + L6474_RESERVED_REG12
     *           + L6474_STEP_MODE
     *           + L6474_ALARM_EN
     *           + L6474_CONFIG
     *           + L6474_STATUS
     *           + L6474_RESERVED_REG13
     *           + L6474_RESERVED_REG14
     *           + L6474_INEXISTENT_REG
     * @warning Some registers can only be written in particular conditions (see L6474's datasheet).
     *          Any attempt to write one of those registers when the conditions are not satisfied
     *          causes the command to be ignored and the NOTPERF_CMD flag to rise at the end of the
     *          last argument byte. Any attempt to set an inexistent register (wrong address value)
     *          causes the command to be ignored and the WRONG_CMD flag to rise.
     *          For example, setting some parameters requires first to disable the power bridge;
     *          this can be done through the soft_hiz() method.
     *          They are the following:
     *           + L6474_EL_POS
     *           + L6474_T_FAST
     *           + L6474_TON_MIN        : value in us
     *           + L6474_TOFF_MIN       : value in us
     *           + L6474_ADC_OUT
     *           + L6474_STEP_MODE
     *           + L6474_CONFIG
     *           + L6474_STATUS
     */
    virtual void set_parameter(unsigned int parameter, float value)
    {
      float register_value;

      switch ((L6474_Registers_t) parameter) {
        case L6474_TVAL:
          register_value = L6474_Tval_Current_to_Par(value);
          break;
        case L6474_TON_MIN:
        case L6474_TOFF_MIN:
          register_value = L6474_Tmin_Time_to_Par(value);
          break;
        default:
          register_value = value;
          break;
      }

      L6474_CmdSetParam((L6474_Registers_t) parameter, (unsigned int) register_value);
    }

    /**
     * @brief  Setting the current position to be the home position.
     * @param  None.
     * @retval None.
     */
    virtual void set_home(void)
    {
      L6474_SetHome();
    }

    /**
     * @brief  Setting the current position to be the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void set_mark(void)
    {
      L6474_SetMark();
    }

    /**
     * @brief  Setting the maximum speed in pps.
     * @param  speed The maximum speed in pps.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_max_speed(unsigned int speed)
    {
      L6474_SetMaxSpeed((unsigned int) speed);
      return true;
    }

    /**
     * @brief  Setting the minimum speed in pps.
     * @param  speed The minimum speed in pps.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_min_speed(unsigned int speed)
    {
      L6474_SetMinSpeed((unsigned int) speed);
      return true;
    }

    /**
     * @brief  Setting the acceleration in pps^2.
     * @param  acceleration The acceleration in pps^2.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_acceleration(unsigned int acceleration)
    {
      L6474_SetAcceleration((unsigned int) acceleration);
      return true;
    }

    /**
     * @brief  Setting the deceleration in pps^2.
     * @param  deceleration The deceleration in pps^2.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_deceleration(unsigned int deceleration)
    {
      L6474_SetDeceleration((unsigned int) deceleration);
      return true;
    }

    /**
     * @brief  Going to a specified position.
     * @param  position The desired position.
     * @retval None.
     */
    virtual void go_to(signed int position)
    {
      L6474_GoTo((signed int) position);
    }

    /**
     * @brief  Going to the home position.
     * @param  None.
     * @retval None.
     */
    virtual void go_home(void)
    {
      L6474_GoHome();
    }

    /**
     * @brief  Going to the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void go_mark(void)
    {
      L6474_GoMark();
    }

    /**
     * @brief  Running the motor towards a specified direction.
     * @param  direction The direction of rotation.
     * @retval None.
     */
    virtual void run(direction_t direction)
    {
      L6474_Run((motorDir_t)(direction == StepperMotor::FWD ? FORWARD : BACKWARD));
    }

    /**
     * @brief  Moving the motor towards a specified direction for a certain number of steps.
     * @param  direction The direction of rotation.
     * @param  steps The desired number of steps.
     * @retval None.
     */
    virtual void move(direction_t direction, unsigned int steps)
    {
      L6474_Move((motorDir_t)(direction == StepperMotor::FWD ? FORWARD : BACKWARD), (unsigned int) steps);
    }

    /**
     * @brief  Stopping the motor through an immediate deceleration up to zero speed.
     * @param  None.
     * @retval None.
     */
    virtual void soft_stop(void)
    {
      L6474_SoftStop();
    }

    /**
     * @brief  Stopping the motor through an immediate infinite deceleration.
     * @param  None.
     * @retval None.
     */
    virtual void hard_stop(void)
    {
      L6474_HardStop();
    }

    /**
     * @brief  Disabling the power bridge after performing a deceleration to zero.
     * @param  None.
     * @retval None.
     */
    virtual void soft_hiz(void)
    {
      L6474_SoftStop();
      L6474_CmdDisable();
    }

    /**
     * @brief  Disabling the power bridge immediately.
     * @param  None.
     * @retval None.
     */
    virtual void hard_hiz(void)
    {
      L6474_HardStop();
      L6474_CmdDisable();
    }

    /**
     * @brief  Waiting while the motor is active.
     * @param  None.
     * @retval None.
     */
    virtual void wait_while_active(void)
    {
      L6474_WaitWhileActive();
    }

    /**
      * @brief  Getting the device state.
      * @param  None.
      * @retval The device state.
      * @note   The device state can be one of the following:
      *           + ACCELERATING
      *           + DECELERATING
      *           + STEADY
      *           + INACTIVE
     */
    virtual motorState_t get_device_state(void)
    {
      return (motorState_t) L6474_GetDeviceState();
    }

    /**
     * @brief  Reading the Status Register.
     * @param  None.
     * @retval None.
     * @note   The Status Register's flags are not cleared, contrary to the
     *         GetStatus() method.
     */
    virtual uint16_t read_status_register(void)
    {
      return (uint16_t) L6474_ReadStatusRegister();
    }

    /**
     * @brief   Setting the Step Mode.
     * @param   step_mode The Step Mode.
     * @retval "true" in case of success, "false" otherwise.
     * @warning Setting the step mode implies first disabling the power bridge through
     *          the soft_hiz() method.
     * @warning Every time step mode is changed, the values of the home
     *          and mark positions lose meaning and are reset.
     */
    virtual bool set_step_mode(step_mode_t step_mode)
    {
      if (step_mode > STEP_MODE_1_16) {
        return false;
      }

      soft_hiz();
      L6474_SelectStepMode((motorStepMode_t) step_mode);
      return true;
    }

    /**
     * @brief  Attaching an error handler.
     * @param  fptr An error handler.
     * @retval None.
     */
    virtual void attach_error_handler(void (*fptr)(uint16_t error))
    {
      L6474_AttachErrorHandler((void (*)(uint16_t error)) fptr);
    }

    /**
     * @brief  Enabling the device.
     * @param  None.
     * @retval None.
     */
    virtual void enable(void)
    {
      L6474_CmdEnable();
    }

    /**
     * @brief  Disabling the device.
     * @param  None.
     * @retval None.
     */
    virtual void disable(void)
    {
      L6474_CmdDisable();
    }

    /**
     * @brief  Getting the version of the firmware.
     * @param  None.
     * @retval The version of the firmware.
     */
    virtual uint8_t get_fw_version(void)
    {
      return (uint8_t) L6474_GetFwVersion();
    }


    /*** Public Interrupt Related Methods ***/

    /* ACTION 6 --------------------------------------------------------------*
     * Implement here interrupt related methods, if any.                      *
     * Note that interrupt handling is platform dependent, e.g.:              *
     *   + mbed:                                                              *
     *     InterruptIn feature_irq(pin); //Interrupt object.                  *
     *     feature_irq.rise(callback);   //Attach a callback.                 *
     *     feature_irq.mode(PullNone);   //Set interrupt mode.                *
     *     feature_irq.enable_irq();     //Enable interrupt.                  *
     *     feature_irq.disable_irq();    //Disable interrupt.                 *
     *   + Arduino:                                                           *
     *     attachInterrupt(pin, callback, RISING); //Attach a callback.       *
     *     detachInterrupt(pin);                   //Detach a callback.       *
     *                                                                        *
     * Example (mbed):                                                        *
     *   void attach_feature_irq(void (*fptr) (void))                         *
     *   {                                                                    *
     *     feature_irq.rise(fptr);                                            *
     *   }                                                                    *
     *                                                                        *
     *   void enable_feature_irq(void)                                        *
     *   {                                                                    *
     *     feature_irq.enable_irq();                                          *
     *   }                                                                    *
     *                                                                        *
     *   void disable_feature_irq(void)                                       *
     *   {                                                                    *
     *     feature_irq.disable_irq();                                         *
     *   }                                                                    *
     *------------------------------------------------------------------------*/
    /**
     * @brief  Attaching an interrupt handler to the FLAG interrupt.
     * @param  fptr An interrupt handler.
     * @retval None.
     */
    void attach_flag_irq(void (*fptr)(void))
    {
      int_cb = fptr;
    }

    /**
     * @brief  Enabling the FLAG interrupt handling.
     * @param  None.
     * @retval None.
     */
    void enable_flag_irq(void)
    {
      attachInterrupt(flag_irq, int_cb, FALLING);
    }

    /**
     * @brief  Disabling the FLAG interrupt handling.
     * @param  None.
     * @retval None.
     */
    void disable_flag_irq(void)
    {
      detachInterrupt(flag_irq);
    }


  protected:

    /*** Protected Component Related Methods ***/

    /* ACTION 7 --------------------------------------------------------------*
     * Declare here the component's specific methods.                         *
     * They should be:                                                        *
     *   + Methods with the same name of the C component's virtual table's    *
     *     functions (1);                                                     *
     *   + Methods with the same name of the C component's extended virtual   *
     *     table's functions, if any (2);                                     *
     *   + Helper methods, if any, like functions declared in the component's *
     *     source files but not pointed by the component's virtual table (3). *
     *                                                                        *
     * Example:                                                               *
     *   status_t COMPONENT_get_value(float *f);   //(1)                      *
     *   status_t COMPONENT_enable_feature(void);  //(2)                      *
     *   status_t COMPONENT_compute_average(void); //(3)                      *
     *------------------------------------------------------------------------*/
    void L6474_AttachErrorHandler(void (*callback)(uint16_t error));
    status_t L6474_Init(void *init);
    status_t L6474_ReadID(uint8_t *id);
    uint16_t L6474_GetAcceleration(void);
    uint16_t L6474_GetCurrentSpeed(void);
    uint16_t L6474_GetDeceleration(void);
    motorState_t L6474_GetDeviceState(void);
    uint8_t L6474_GetFwVersion(void);
    int32_t L6474_GetMark(void);
    uint16_t L6474_GetMaxSpeed(void);
    uint16_t L6474_GetMinSpeed(void);
    int32_t L6474_GetPosition(void);
    void L6474_GoHome(void);
    void L6474_GoMark(void);
    void L6474_GoTo(int32_t targetPosition);
    void L6474_HardStop(void);
    void L6474_Move(motorDir_t direction, uint32_t stepCount);
    void L6474_Run(motorDir_t direction);
    bool L6474_SetAcceleration(uint16_t newAcc);
    bool L6474_SetDeceleration(uint16_t newDec);
    void L6474_SetHome(void);
    void L6474_SetMark(void);
    bool L6474_SetMaxSpeed(uint16_t newMaxSpeed);
    bool L6474_SetMinSpeed(uint16_t newMinSpeed);
    bool L6474_SoftStop(void);
    void L6474_WaitWhileActive(void);
    void L6474_CmdDisable(void);
    void L6474_CmdEnable(void);
    uint32_t L6474_CmdGetParam(L6474_Registers_t parameter);
    uint16_t L6474_CmdGetStatus(void);
    void L6474_CmdNop(void);
    void L6474_CmdSetParam(L6474_Registers_t parameter, uint32_t value);
    uint16_t L6474_ReadStatusRegister(void);
    void L6474_SelectStepMode(motorStepMode_t stepMod);
    motorDir_t L6474_GetDirection(void);
    void L6474_SetDirection(motorDir_t direction);
    void L6474_ApplySpeed(uint16_t newSpeed);
    void L6474_ComputeSpeedProfile(uint32_t nbSteps);
    int32_t L6474_ConvertPosition(uint32_t abs_position_reg);
    void L6474_ErrorHandler(uint16_t error);
    void L6474_SendCommand(uint8_t param);
    void L6474_SetRegisterToPredefinedValues(void);
    void L6474_SetRegisterToInitializationValues(L6474_init_t *init);
    void L6474_WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte);
    void L6474_SetDeviceParamsToPredefinedValues(void);
    void L6474_StartMovement(void);
    void L6474_StepClockHandler(void);
    float L6474_Tval_Current_to_Par(float current_mA);
    float L6474_Par_to_Tval_Current(float Tval);
    float L6474_Tmin_Time_to_Par(float ton_min_us);
    float L6474_Par_to_Tmin_Time(float Tmin);


    /*** Component's I/O Methods ***/

    /**
     * @brief      Utility function to read data from L6474.
     * @param[out] pBuffer pointer to the buffer to read data into.
     * @param[in]  NumBytesToRead number of bytes to read.
     * @retval     COMPONENT_OK in case of success, COMPONENT_ERROR otherwise.
     */
    status_t Read(uint8_t *pBuffer, uint16_t NumBytesToRead)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
        digitalWrite(ssel, LOW);

        for (uint16_t i = 0; i < NumBytesToRead; i++) {
          *(pBuffer + i) = dev_spi->transfer(0x00);
        }

        digitalWrite(ssel, HIGH);

        dev_spi->endTransaction();
      }

      return COMPONENT_OK;
    }

    /**
     * @brief      Utility function to write data to L6474.
     * @param[in]  pBuffer pointer to the buffer of data to send.
     * @param[in]  NumBytesToWrite number of bytes to write.
     * @retval     COMPONENT_OK in case of success, COMPONENT_ERROR otherwise.
     */
    status_t Write(uint8_t *pBuffer, uint16_t NumBytesToWrite)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
        digitalWrite(ssel, LOW);

        for (uint16_t i = 0; i < NumBytesToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }

        digitalWrite(ssel, HIGH);

        dev_spi->endTransaction();
      }

      return COMPONENT_OK;
    }

    /**
     * @brief      Utility function to read and write data from/to L6474 at the same time.
     * @param[out] pBufferToRead pointer to the buffer to read data into.
     * @param[in]  pBufferToWrite pointer to the buffer of data to send.
     * @param[in]  NumBytes number of bytes to read and write.
     * @retval     COMPONENT_OK in case of success, COMPONENT_ERROR otherwise.
     */
    status_t ReadWrite(uint8_t *pBufferToRead, uint8_t *pBufferToWrite, uint16_t NumBytes)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
        digitalWrite(ssel, LOW);

        for (uint16_t i = 0; i < NumBytes; i++) {
          *(pBufferToRead + i) = dev_spi->transfer(pBufferToWrite[i]);
        }

        digitalWrite(ssel, HIGH);

        dev_spi->endTransaction();
      }

      return COMPONENT_OK;
    }

    /* ACTION 8 --------------------------------------------------------------*
     * Implement here other I/O methods beyond those already implemented      *
     * above, which are declared extern within the component's header file.   *
     *------------------------------------------------------------------------*/
    /**
     * @brief  Making the CPU wait.
     * @param  ms_delay delay in milliseconds.
     * @retval None.
     */
    void L6474_Delay(uint32_t ms_delay)
    {
      delay(ms_delay);
    }

    /**
     * @brief  Enabling interrupts.
     * @param  None.
     * @retval None.
     */
    void L6474_EnableIrq(void)
    {
      interrupts();
    }

    /**
     * @brief  Disabling interrupts.
     * @param  None.
     * @retval None.
     */
    void L6474_DisableIrq(void)
    {
      noInterrupts();
    }

    /**
     * @brief  Initialising the PWM.
     * @param  None.
     * @retval None.
     */
    void L6474_PwmInit(void) {}

    /**
     * @brief  Setting the frequency of PWM.
     *         The frequency controls directly the speed of the device.
     * @param  frequency the frequency of PWM.
     * @retval None.
     */
    void L6474_PwmSetFreq(uint16_t frequency)
    {
      if (!first_time_pwm) {
        pwm_timer->pauseChannel(pwm_channel);
      } else {
        first_time_pwm = false;
      }
      pwm_timer->setPWM(pwm_channel, pwm_pin, frequency, 50, callback_handler, NULL);
    }

    /**
     * @brief  Stopping the PWM.
     * @param  None.
     * @retval None.
     */
    void L6474_PwmStop(void)
    {
      pwm_timer->pause();
      pwm_timer->detachInterrupt();
    }

    /**
     * @brief  Putting the device in standby mode.
     * @param  None.
     * @retval None.
     */
    void L6474_ReleaseReset(void)
    {
      digitalWrite(standby_reset, HIGH);
    }

    /**
     * @brief  Putting the device in reset mode.
     * @param  None.
     * @retval None.
     */
    void L6474_Reset(void)
    {
      digitalWrite(standby_reset, LOW);
    }

    /**
     * @brief  Setting the direction of rotation.
     * @param  gpioState direction of rotation: "1" for forward, "0" for backward.
     * @retval None.
     */
    void L6474_SetDirectionGpio(uint8_t gpioState)
    {
      digitalWrite(direction, gpioState);
    }

    /**
     * @brief      Writing and reading bytes to/from the component through the SPI at the same time.
     * @param[in]  pByteToTransmit pointer to the buffer of data to send.
     * @param[out] pReceivedByte pointer to the buffer to read data into.
     * @retval     "0" in case of success, "1" otherwise.
     */
    uint8_t L6474_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
    {
      return (uint8_t)(ReadWrite(pReceivedByte, pByteToTransmit, number_of_devices) == COMPONENT_OK ? 0 : 1);
    }


    /*** Component's Instance Variables ***/

    /* ACTION 9 --------------------------------------------------------------*
     * Declare here interrupt related variables, if needed.                   *
     * Note that interrupt handling is platform dependent, see                *
     * "Interrupt Related Methods" above.                                     *
     *                                                                        *
     * Example:                                                               *
     *   + mbed:                                                              *
     *     InterruptIn feature_irq;                                           *
     *------------------------------------------------------------------------*/
    /* Flag Interrupt. */
    uint8_t flag_irq;
    void (*int_cb)(void);

    /* ACTION 10 -------------------------------------------------------------*
     * Declare here other pin related variables, if needed.                   *
     *                                                                        *
     * Example:                                                               *
     *   + mbed:                                                              *
     *     DigitalOut standby_reset;                                          *
     *------------------------------------------------------------------------*/
    /* Standby/reset pin. */
    uint8_t standby_reset;

    /* Direction of rotation pin. */
    uint8_t direction;

    /* Pulse Width Modulation pin. */
    HardwareTimer *pwm_timer;
    uint32_t pwm_channel;
    uint8_t pwm_pin;
    TIM_TypeDef *pwm_instance;
    bool first_time_pwm;
    PwmHandler_Callback callback_handler;

    /* ACTION 11 -------------------------------------------------------------*
     * Declare here communication related variables, if needed.               *
     *                                                                        *
     * Example:                                                               *
     *   + mbed:                                                              *
     *     DigitalOut ssel;                                                   *
     *     DevSPI &dev_spi;                                                   *
     *------------------------------------------------------------------------*/
    /* Configuration. */
    uint8_t ssel;

    /* IO Device. */
    SPIClass *dev_spi;
    uint32_t spi_speed;

    /* ACTION 12 -------------------------------------------------------------*
     * Declare here identity related variables, if needed.                    *
     * Note that there should be only a unique identifier for each component, *
     * which should be the "who_am_i" parameter.                              *
     *------------------------------------------------------------------------*/
    /* Identity */
    uint8_t who_am_i;

    /* ACTION 13 -------------------------------------------------------------*
     * Declare here the component's static and non-static data, one variable  *
     * per line.                                                              *
     *                                                                        *
     * Example:                                                               *
     *   float measure;                                                       *
     *   int instance_id;                                                     *
     *   static int number_of_instances;                                      *
     *------------------------------------------------------------------------*/
    /* Data. */
    void (*error_handler_callback)(uint16_t error);
    deviceParams_t device_prm;
    uint8_t device_instance;

    /* Static data. */
    static uint8_t number_of_devices;
    static uint8_t spi_tx_bursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
    static uint8_t spi_rx_bursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];


  public:

    /* Static data. */
    static bool spi_preemtion_by_isr;
    static bool isr_flag;
};

#endif // __L6474_CLASS_H

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
