/*
 * matrix.h
 *
 *  Created on: 14.10.2017
 *      Author: user
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_

#include "stm32l0xx_hal.h"

#define MATRIX_CHAR_WIDTH 6

extern const uint8_t font[475];

struct matrix {
  const struct matrix_init *init;

  uint16_t text_len;
  uint8_t current_column;
  uint16_t scroll_period_counter;
  uint8_t current_scroll;
};

struct matrix_init {
  uint8_t columns;
  uint8_t rows;
  uint8_t modules;
  uint16_t scroll_period;

  uint8_t *column_buffer;
  char *text_buffer;

  GPIO_TypeDef *cdi_port;
  uint16_t cdi_pin;
  GPIO_TypeDef *cck_port;
  uint16_t cck_pin;
  GPIO_TypeDef *cle_port;
  uint16_t cle_pin;

  GPIO_TypeDef *csa_port;
  uint16_t csa_pin;
  GPIO_TypeDef *csb_port;
  uint16_t csb_pin;
  GPIO_TypeDef *csc_port;
  uint16_t csc_pin;
};

void init_matrix(struct matrix *matrix, const struct matrix_init *init);

// turn off all matrix LEDs
void matrix_off(const struct matrix *m);

// shift out column data to shift registers
void matrix_write_column(const struct matrix *m, uint8_t dat);

// flush column data from shift registers to output
// (turn on LEDs)
void matrix_flush_column(const struct matrix *m);

// select column within the modules to write to
void matrix_select_column(const struct matrix *m, uint8_t col);

// clear data buffer
void matrix_clear(const struct matrix *m);

// update display. should be called every 1 ms,
// for example from a timer interrupt.
void matrix_update(struct matrix *m);

#endif /* INC_MATRIX_H_ */
