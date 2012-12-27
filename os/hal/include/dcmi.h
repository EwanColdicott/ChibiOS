/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    dcmi.h
 * @brief   DCMI Driver macros and structures.
 *
 * @addtogroup DCMI
 * @{
 */

#ifndef _DCMI_H_
#define _DCMI_H_

#if HAL_USE_DCMI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    DCMI configuration options
 * @{
 */
/**
 * @brief   Enables synchronous APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(DCMI_USE_WAIT) || defined(__DOXYGEN__)
#define DCMI_USE_WAIT                TRUE
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  DCMI_UNINIT = 0,                   /**< Not initialized.                   */
  DCMI_STOP = 1,                     /**< Stopped.                           */
  DCMI_READY = 2,                    /**< Ready to begin listening.          */
  DCMI_LISTEN = 3,                   /**< Listening for frames, continuous   */
  DCMI_LISTEN_ONCE = 4,              /**< Listening for frames, one shot     */
  DCMI_ACTIVE = 5,                   /**< Receiving frame, continuous        */
  DCMI_ACTIVE_ONCE = 6               /**< Receiving frame, one shot          */
} dcmistate_t;

#include "dcmi_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Receives a frame on the DCMI.
 * @details This asynchronous function starts a receive operation.
 * @post    Upon either of the two buffers being filled, the configured callback
 *          (dcmiTransferComplete_cb) is invoked.
 *          At the end of the operation the configured callback 
 *          (dcmiFrameComplete_cb) is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] dcmip     pointer to the @p DCMIDriver object
 * @param[in] n         Size of each receive buffer, in DCMI words.
 * @param[in] rxbuf0    the pointer to the first receive buffer
 * @param[out] rxbuf1   the pointer to the second receive buffer
 *
 * @iclass
 */
#define dcmiStartReceiveI(dcmip, n, rxbuf0, rxbuf1) {                         \
  (dcmip)->state = DCMI_LISTEN;                                               \
  dcmi_lld_receive(dcmip, n, rxbuf0, rxbuf1);                                 \
}

/**
 * @brief   Receives a single frame over the DCMI
 * @details This asynchronous function starts a single shot receive operation.
 * @post    Upon either of the two buffers being filled, the configured callback
 *          (dcmiTransferComplete_cb) is invoked.
 *          At the end of the operation the configured callback
 *          (dcmiFrameComplete_cb) is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] dcmip     pointer to the @p DCMIDriver object
 * @param[in] n         Size of each receive buffer, in DCMI words.
 * @param[in] rxbuf0    the pointer to the first receive buffer
 * @param[in] rxbuf1    the pointer to the second receive buffer
 *
 * @iclass
 */
#define dcmiStartReceiveOneShotI(dcmip, n, rxbuf0, rxbuf1) {                  \
  (dcmip)->state = DCMI_LISTEN_ONCE;                                          \
  dcmi_lld_receive(dcmip, n, rxbuf0, rxbuf1);                                 \
}

/**
 * @brief   Receives data from the DCMI bus.
 * @details This asynchronous function starts a receive operation.
 * @pre     A slave must have been selected using @p dcmiSelect() or
 *          @p dcmiSelectI().
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] dcmip      pointer to the @p DCMIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @iclass
 */
#define dcmiStartReceiveI(dcmip, n, rxbuf) {                                  \
  (dcmip)->state = DCMI_ACTIVE;                                               \
  dcmi_lld_receive(dcmip, n, rxbuf);                                          \
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 * @note    This API is implemented as a macro in order to minimize latency.
 *
 * @param[in] dcmip      pointer to the @p DCMIDriver object
 * @param[in] frame     the data frame to send over the DCMI bus
 * @return              The received data frame from the DCMI bus.
 */
#define dcmiPolledExchange(dcmip, frame) dcmi_lld_polled_exchange(dcmip, frame)
/** @} */

/**
 * @name    Low Level driver helper macros
 * @{
 */
#if DCMI_USE_WAIT || defined(__DOXYGEN__)
/**
 * @brief   Waits for operation completion.
 * @details This function waits for the driver to complete the current
 *          operation.
 * @pre     An operation must be running while the function is invoked.
 * @note    No more than one thread can wait on a DCMI driver using
 *          this function.
 *
 * @param[in] dcmip      pointer to the @p DCMIDriver object
 *
 * @notapi
 */
#define _dcmi_wait_s(dcmip) {                                                 \
  chDbgAssert((dcmip)->thread == NULL,                                       \
              "_dcmi_wait(), #1", "already waiting");                        \
  (dcmip)->thread = chThdSelf();                                             \
  chSchGoSleepS(THD_STATE_SUSPENDED);                                       \
}

/**
 * @brief   Wakes up the waiting thread.
 *
 * @param[in] dcmip      pointer to the @p DCMIDriver object
 *
 * @notapi
 */
#define _dcmi_wakeup_isr(dcmip) {                                             \
  if ((dcmip)->thread != NULL) {                                             \
    Thread *tp = (dcmip)->thread;                                            \
    (dcmip)->thread = NULL;                                                  \
    chSysLockFromIsr();                                                     \
    chSchReadyI(tp);                                                        \
    chSysUnlockFromIsr();                                                   \
  }                                                                         \
}
#else /* !DCMI_USE_WAIT */
#define _dcmi_wait_s(dcmip)
#define _dcmi_wakeup_isr(dcmip)
#endif /* !DCMI_USE_WAIT */

/**
 * @brief   Common ISR code.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] dcmip      pointer to the @p DCMIDriver object
 *
 * @notapi
 */
#define _dcmi_isr_code(dcmip) {                                               \
  if ((dcmip)->config->end_cb) {                                             \
    (dcmip)->state = DCMI_COMPLETE;                                           \
    (dcmip)->config->end_cb(dcmip);                                           \
    if ((dcmip)->state == DCMI_COMPLETE)                                      \
      (dcmip)->state = DCMI_READY;                                            \
  }                                                                         \
  else                                                                      \
    (dcmip)->state = DCMI_READY;                                              \
  _dcmi_wakeup_isr(dcmip);                                                    \
}
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void dcmiInit(void);
  void dcmiObjectInit(DCMIDriver *dcmip);
  void dcmiStart(DCMIDriver *dcmip, const DCMIConfig *config);
  void dcmiStop(DCMIDriver *dcmip);
  void dcmiSelect(DCMIDriver *dcmip);
  void dcmiUnselect(DCMIDriver *dcmip);
  void dcmiStartIgnore(DCMIDriver *dcmip, size_t n);
  void dcmiStartExchange(DCMIDriver *dcmip, size_t n,
                        const void *txbuf, void *rxbuf);
  void dcmiStartSend(DCMIDriver *dcmip, size_t n, const void *txbuf);
  void dcmiStartReceive(DCMIDriver *dcmip, size_t n, void *rxbuf);
#if DCMI_USE_WAIT
  void dcmiIgnore(DCMIDriver *dcmip, size_t n);
  void dcmiExchange(DCMIDriver *dcmip, size_t n, const void *txbuf, void *rxbuf);
  void dcmiSend(DCMIDriver *dcmip, size_t n, const void *txbuf);
  void dcmiReceive(DCMIDriver *dcmip, size_t n, void *rxbuf);
#endif /* DCMI_USE_WAIT */
#if DCMI_USE_MUTUAL_EXCLUSION
  void dcmiAcquireBus(DCMIDriver *dcmip);
  void dcmiReleaseBus(DCMIDriver *dcmip);
#endif /* DCMI_USE_MUTUAL_EXCLUSION */
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_DCMI */

#endif /* _DCMI_H_ */

/** @} */
