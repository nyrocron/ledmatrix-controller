/*
 * matrix.c
 *
 *  Created on: 14.10.2017
 *      Author: user
 */

#include "matrix.h"

void init_matrix(struct matrix *matrix, const struct matrix_init *init)
{
  matrix->init = init;

  matrix->text_len = 0;
  matrix->current_column = 0;
  matrix->scroll_period_counter = 0;
  matrix->current_scroll = 0;
}

void matrix_off(const struct matrix *m)
{
  for (uint8_t i = 0; i < m->init->modules; i++)
    matrix_write_column(m, 0);
  matrix_flush_column(m);
}

void matrix_write_column(const struct matrix *m, uint8_t dat)
{
  // shift out column data (CDI, CCK)
  for (uint8_t i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(m->init->cdi_port, m->init->cdi_pin, dat & (0b10000000 >> i) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->init->cck_port, m->init->cck_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->init->cck_port, m->init->cck_pin, GPIO_PIN_RESET);
  }
}

void matrix_flush_column(const struct matrix *m)
{
  // set column latch (CLE)
  HAL_GPIO_WritePin(m->init->cle_port, m->init->cle_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(m->init->cle_port, m->init->cle_pin, GPIO_PIN_RESET);
}

void matrix_select_column(const struct matrix *m, uint8_t col)
{
  HAL_GPIO_WritePin(m->init->csa_port, m->init->csa_pin, col & 0b001 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m->init->csb_port, m->init->csb_pin, col & 0b010 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m->init->csc_port, m->init->csc_pin, col & 0b100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void matrix_clear(const struct matrix *m)
{
  for (uint8_t col = 0; col < m->init->columns; col++) {
    m->init->column_buffer[col] = 0;
  }
}

void matrix_update(struct matrix *m)
{
  // flash current display column
  matrix_off(m);
  matrix_select_column(m, m->current_column);
  for (int16_t i_mod = m->init->modules - 1; i_mod >= 0; i_mod--)
    matrix_write_column(m, m->init->column_buffer[m->current_column + 8 * i_mod]);
  matrix_flush_column(m);

  m->current_column++;
  if (m->current_column >= 8)
    m->current_column = 0;

  // after cycling through all columns (every 8ms)
  if (m->current_column == 0) {
    m->scroll_period_counter++;

    // scroll every display_scroll_period cycles (8ms per cycle)
    if (m->scroll_period_counter >= m->init->scroll_period) {
      m->current_scroll++;
      if (m->text_len * MATRIX_CHAR_WIDTH < m->init->columns ||
          m->current_scroll >= m->text_len * MATRIX_CHAR_WIDTH)
        m->current_scroll = 0;
      m->scroll_period_counter = 0;
    }

    // update display from text after last column
    uint16_t i_char = m->current_scroll / MATRIX_CHAR_WIDTH; // index of character in text buffer
    uint16_t char_base = (m->init->text_buffer[i_char] - 32) * 5; // character base index in font
    uint16_t char_col = m->current_scroll % MATRIX_CHAR_WIDTH; //
    for (uint8_t display_col = 0; display_col < m->init->columns; display_col++) {
      m->init->column_buffer[display_col] = char_col < 5 ? font[char_base + char_col] : 0x00;

      char_col++;
      if (char_col > 5) {
        i_char++;
        if (i_char >= m->text_len)
          i_char = 0;
        char_base = (m->init->text_buffer[i_char] - 32) * 5;
        char_col = 0;
      }
    }
  }
}
