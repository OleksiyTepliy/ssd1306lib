/* MIT License
 * 
 * Copyright 2018, Tymofii Khodniev <thodnev @ github>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE.
 */

#include "oled.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#if !defined(OLED_NO_I2C)
/***** I2C-related logic *****/
uint8_t OLED_cmdbuffer[OLED_CMDBUFFER_LEN];

static uint8_t _i2c_cmd_init[] = {
	0x80, 0x8D, 0x80, 0x14	/* Enable charge pump	 */
	,0x80, 0xAF		/* Display on	      	 */
	,0x80, 0x81, 0x80, 0xFF /* Set brightness to 255 */
	,0x80, 0xA7		/* Enable inversion 	 */
};

static uint8_t _i2c_cmd_setpage[] = {
	0x80, 0x00, 0x80, 0x10, /* Set column cursor to 0 */
	0x80, 0xB0 /* Last nibble in 0xB0 defines page (0xB0..0xB7) */
};

static uint8_t _i2c_cmd_setbrightness[] = {
	0x80, 0x81, 0x80, 0xFF  /* Last byte is brightness level (0..255) */
};

static uint8_t _i2c_cmd_dataprefix[] = {0x40};

static uint8_t i2c_devaddr;
static uint8_t *i2c_prefix_ptr;
static uint8_t *i2c_prefix_count;
static uint8_t *i2c_data_ptr;
static uint16_t i2c_data_count;
static bool i2c_is_fastfail;
static void (*i2c_callback)(void *); /* called after transaction finish */
static void *i2c_callback_args;

/* States used in ISR FSM */
enum I2C_State_e {
	I2C_STATE_IDLE = 0,
	I2C_STATE_STOP,
	I2C_STATE_SLAVEADDR,
	I2C_STATE_WRITEPREFIX,
	I2C_STATE_WRITEBYTE
};
static enum I2C_State_e i2c_state = I2C_STATE_IDLE;


static void I2C_init(uint32_t hz_freq)
{
	i2c_state = I2C_STATE_IDLE;
	/* Enable the Two Wire Interface module */
	power_twi_enable();

	/* Select TWBR and TWPS based on frequency. Quite tricky, the main point */
	/* is that prescaler is a pow(4, TWPS)				 	 */
	/* TWBR * TWPS_prescaler value */
	uint32_t twbr = F_CPU / (2 * hz_freq) - 8;
	uint8_t twps;
	for (twps = 0; twps < 4; twps++) {
		if (twbr <= 255)
			break;
		twbr /= 4;
	}

	TWBR = (uint8_t)twbr;
	TWSR = (TWSR & 0xFC) | (twps & 0x03);

	TWCR = (1 << TWEN) | (1 << TWIE);
}


bool OLED_i2c_tx_shed(uint8_t addr, uint8_t *prefix, uint8_t prefix_len, uint8_t *bytes, uint16_t bytes_len, 
		      void (*end_cbk)(void *), void *cbk_args, bool fastfail)
{
	bool ret = false;
	/* No interrupts can occur while this block is executed */
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (i2c_state == I2C_STATE_IDLE) {
			i2c_prefix_ptr = prefix;
			i2c_prefix_count = prefix_len;
			i2c_data_ptr = bytes;
			i2c_data_count = bytes_len;
			i2c_is_fastfail = fastfail;
			i2c_callback = end_cbk;
			i2c_callback_args = cbk_args;
			/* Send START signal and initiating new transaction */
			i2c_state = I2C_STATE_SLAVEADDR;
			i2c_devaddr = (addr << 1);
			TWCR |= (1 << TWSTA) | (1 << TWINT);
			ret = true;
		}
	}
	return ret;
}


ISR(TWI_vect, ISR_BLOCK)
{
	switch(i2c_state) {
	case(I2C_STATE_IDLE):
	case(I2C_STATE_STOP):
		/* transfer stop and go to IDLE*/
		/* signal with callback that transaction is over */
		TWCR |= (1 << TWSTO) | (1 << TWINT);
		i2c_state = I2C_STATE_IDLE;
		(*i2c_callback)(i2c_callback_args);
		break;
	case(I2C_STATE_SLAVEADDR):
		// load value
		TWDR = i2c_devaddr;
		TWCR = (TWCR & ~(1 << TWSTA)) | (1 << TWINT);
		if ((NULL == i2c_prefix_ptr) && (NULL == i2c_data_ptr)) {
			i2c_state = I2C_STATE_STOP;
		} else if (NULL == i2c_prefix_ptr) {
			i2c_state = I2C_STATE_WRITEBYTE;
		} else {
			i2c_state = I2C_STATE_WRITEPREFIX;
		}
		break;
	case(I2C_STATE_WRITEPREFIX):
		// load next byte of prefix
		TWDR = *i2c_prefix_ptr++;
		i2c_prefix_count--;
		TWCR |= (1 << TWINT);
		if (!i2c_prefix_count) {
			i2c_state = (NULL == i2c_data_ptr) ? I2C_STATE_STOP : I2C_STATE_WRITEBYTE;
		}
		break;
	case(I2C_STATE_WRITEBYTE):
		// load next byte
		TWDR = *i2c_data_ptr++;
		i2c_data_count--;
		TWCR |= (1 << TWINT);
		if (!i2c_data_count)
			i2c_state = I2C_STATE_STOP;
		break;
	}
}


/* Callback which essentially does nothing */
static void OLED_cbk_empty(void *args)
{
	// empty callback
}


/* A dummy callback which simply unlocks the oled lock */
static void OLED_cbk_unlock(void *args)
{
	OLED *oled = args;
	OLED_unlock(oled);
}


/* Callbacks which are used to write each page */
static void OLED_cbk_writepage(void *args);
static void OLED_cbk_setwritepage(void *args);
/* Writes page. This is called after OLED_cbk_setwritepage */
static void OLED_cbk_writepage(void *args)
{
	OLED *oled = args;
	if (oled->cur_page >= oled->num_pages) {
		OLED_unlock(oled);
		return;
	}
	uint8_t *lineptr = &oled->frame_buffer[oled->cur_page * (uint16_t)oled->width];
	oled->cur_page++;
	while(!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_dataprefix, OLED_ARR_SIZE(_i2c_cmd_dataprefix), 
				lineptr, oled->width,
				&OLED_cbk_setwritepage, oled, true)) {
		// nop
	}
}

/* Sets page index and calls OLED_cbk_writepage via callback */
static void OLED_cbk_setwritepage(void *args)
{
	OLED *oled = args;
	_i2c_cmd_setpage[OLED_ARR_SIZE(_i2c_cmd_setpage) - 1] = 0xB0 | oled->cur_page;
	while(!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_setpage, 
                                OLED_ARR_SIZE(_i2c_cmd_setpage), NULL, 0,
				&OLED_cbk_writepage, oled, true)) {
		// nop
	}
}



void OLED_cmd_setbrightness(OLED *oled, uint8_t level)
{
	_i2c_cmd_setbrightness[OLED_ARR_SIZE(_i2c_cmd_setbrightness) - 1] = level;
	OLED_spinlock(oled);
	while(!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_setbrightness, 
                                OLED_ARR_SIZE(_i2c_cmd_setbrightness), NULL, 0,
				&OLED_cbk_unlock, oled, true)) {
		// nop
	}
}


void OLED_refresh(OLED *oled)
{
	OLED_spinlock(oled);
	/* Code below is executed under lock */
	oled->cur_page = 0;
	OLED_cbk_setwritepage(oled);
	/* Lock is unlocked after series of callbacks, in the last one */
}
#endif // OLED_NO_I2C


/***** Display-related logic *****/
OLED_err __OLED_init(OLED *oled, uint8_t width, uint8_t height, uint8_t *frame_buffer, uint32_t i2c_freq_hz, uint8_t i2c_addr)
{
	oled->width = width;
	oled->height = height;
	oled->frame_buffer = frame_buffer;
	oled->busy_lock = 1;	/* Initially: 1 - unlocked */

	OLED_I2CWRAP(
		oled->i2c_addr = i2c_addr;
		oled->cur_page = 0;
		oled->num_pages = 8;

		I2C_init(i2c_freq_hz);
		
		if (!OLED_i2c_tx_shed(oled->i2c_addr, _i2c_cmd_init, OLED_ARR_SIZE(_i2c_cmd_init),
				      NULL, 0, OLED_cbk_empty, NULL, true)) {
			return OLED_EBUSY;
		}
	) // OLED_I2CWRAP

	return OLED_EOK;
}


OLED_err OLED_put_pixel(OLED *oled, uint8_t x, uint8_t y, bool pixel_state)
{
	if ((x >= oled->width) || (y >= oled->height))
		return OLED_EBOUNDS;
	OLED_put_pixel_(oled, x, y, pixel_state);	/* Use inline */
	return OLED_EOK;
}


OLED_err OLED_put_rectangle(OLED *oled, uint8_t x_from, uint8_t y_from, uint8_t x_to, uint8_t y_to, enum OLED_params params)
{
	if (params > (OLED_BLACK | OLED_FILL))
		return OLED_EPARAMS;
	bool pixel_color = (OLED_BLACK & params) != 0;
	bool is_fill = (OLED_FILL & params) != 0;

	/* Limit coordinates to display bounds */
	uint8_t size_errors = 0;
	uint8_t w_max = oled->width - 1;
	uint8_t h_max = oled->height - 1;
	if (x_from > w_max) {
		x_from = w_max;
		size_errors++;
	}
	if (x_to > w_max) {
		x_to = w_max;
		size_errors++;
	}
	if (y_from > h_max) {
		y_from = h_max;
		size_errors++;
	}
	if (y_to > h_max) {
		y_to = h_max;
		size_errors++;
	}
	/* If all coordinates are out of bounds */
	if (size_errors >= 4)
		return OLED_EBOUNDS;

	//OLED_WITH_SPINLOCK(oled) {
		/* Normalize coordinates */
		/* start_@ indicates coordinates of upper left corner  */
		/* stop_@ indicates coordinates of bottom right corner */
		uint8_t start_x = x_to < x_from ? x_to : x_from; /* x min */
		uint8_t start_y = y_to < y_from ? y_to : y_from; /* y min */
		uint8_t stop_x = x_to > x_from ? x_to : x_from;  /* x max */
		uint8_t stop_y = y_to > y_from ? y_to : y_from;  /* y max */

		if (is_fill) {
			/* Fill whole area */
			for (uint8_t x = start_x; x <= stop_x; x++) {
				for (uint8_t y = start_y; y <= stop_y; y++) {
					OLED_put_pixel_(oled, x, y, pixel_color);
				}
			}
		} else {
			/* Draw outer frame */
			for (uint8_t x = start_x; x <= stop_x; x++) {
				OLED_put_pixel_(oled, x, start_y, pixel_color);
				OLED_put_pixel_(oled, x, stop_y, pixel_color);
			}
			for (uint8_t y = start_y; y <= stop_y; y++) {
				OLED_put_pixel_(oled, start_x, y, pixel_color);
				OLED_put_pixel_(oled, stop_x, y, pixel_color);
			}
		}
	//}

	return OLED_EOK;
}


/********************* PUT REGION *********************/
/******************************************************/

static void _put_reg_repl(uint8_t *a, uint8_t *b)
{
	*a = *b;
}
static void _put_reg_and(uint8_t *a, uint8_t *b)
{
	*a &= *b;
}
static void _put_reg_or(uint8_t *a, uint8_t *b)
{
	*a |= *b;
}
static void _put_reg_nand(uint8_t *a, uint8_t *b)
{
	*a &= ~(*b);
}
static void _put_reg_nor(uint8_t *a, uint8_t *b)
{
	*a |= ~(*b);
}
static void _put_reg_xor(uint8_t *a, uint8_t *b)
{
	*a ^= *b;
}

/* array of function pointers for masked operations */
static void (*func_ptr[])(uint8_t *, uint8_t *) = {_put_reg_repl, _put_reg_and, _put_reg_or,
 						_put_reg_nand, _put_reg_nor, _put_reg_xor};

/**
 * _print_error - prints ann error message 
 * in the middle of the screen.
 * 
 */
static void _print_error(OLED *oled, OLED_err err) {
	/* if an error occurs we print Err depending on error parameters */
	uint8_t err_message[] = {
		0xFF, 0xFF, 0xDB, 0xDB, 0xDB,		//E
		0x00, 0x00, 0xFF, 0xFF, 0x03, 0x03,	//r
		0x00, 0x00, 0xFF, 0xFF, 0x03, 0x03	//r
	};
	/* Fill region in the center of the screen */
	OLED_put_region(oled, 39, 3, err_message, 47, 16, FILL_WHITE, REPLACE, TOP_LEFT);
	/* print an error message on top of it */
	OLED_put_region(oled, 17, 1, err_message, 54, 24, FILL_PICTURE, REPLACE, TOP_LEFT);
	
	if (err == OLED_EPARAMS) {	// P letter
		uint8_t err_PARAM[] = {
			0xFF, 0xFF,
			0x09, 0x09,
			0x0F
		};
		OLED_put_region(oled, 5, 1, err_PARAM, 75, 24, FILL_PICTURE, REPLACE, TOP_LEFT);
	}
	if (err == OLED_EBOUNDS) {	// B letter
		uint8_t err_BOUNDS[] = {
			0xFF, 0x99,
			0x99, 0x9F,
			0xF0
		};
		OLED_put_region(oled, 5, 1, err_BOUNDS, 75, 24, FILL_PICTURE, REPLACE, TOP_LEFT);
	}	
}

/* flags for _put_region_fill_bits func */
static const uint8_t TOP_BITS = 1;
static const uint8_t BOT_BITS = 0;

/**
 * _put_region_fill_bits -  sub function of OLED_put_region.
 * Fills top and bottom bits.
 */
static OLED_err _put_region_fill_bits(OLED *oled, uint16_t x_start, uint16_t x_stop, uint8_t offset,
uint8_t and_mask, enum fill_type colour_byte, enum operations op_flag, uint8_t bits_pos)
{
	uint8_t bits; 
	if (bits_pos == TOP_BITS) {
		bits = colour_byte << offset;
	} else if (bits_pos == BOT_BITS) {
		bits = colour_byte >> offset;
	}
	else {
		return OLED_EPARAMS;
	}

	uint8_t mask_or_xor_nand = bits;
	uint8_t mask_and_nor = bits | and_mask;

	for (uint16_t i = x_start; i < x_stop; i++) {
		if (op_flag == REPLACE) {
			oled->frame_buffer[i] |= bits;
			oled->frame_buffer[i] &= (bits | and_mask);
		} else if (op_flag == OR || op_flag == XOR || op_flag == NAND) {
			func_ptr[op_flag](&oled->frame_buffer[i], &mask_or_xor_nand);
		} else if (op_flag == AND || op_flag == NOR) {
			func_ptr[op_flag](&oled->frame_buffer[i], &mask_and_nor);
		} 
		else {
			return OLED_EPARAMS;
		}
	}
	return OLED_EOK;
}



OLED_err OLED_put_region(OLED *oled,  uint8_t col,  uint8_t row, const uint8_t *const data, 
		int8_t x_begin, int8_t y_begin, enum fill_type arr_flag, enum operations op_flag,
		enum position corner_pos)
{
	OLED_err err = OLED_EOK;

	if (corner_pos != TOP_LEFT) {
		switch (corner_pos) {
		case TOP_RIGHT: 
			x_begin -= col;
			break;
		case BOT_LEFT: 
			y_begin -= row * 8;
			break;
		case BOT_RIGHT: 
			x_begin -= col;
			y_begin -= row * 8;
			break;
		case CENTER:
			x_begin -= col / 2;
			y_begin -= row * 8 / 2;
			break;
		default: 
			err = OLED_EPARAMS;
			_print_error(oled, err);
			return err;
		}
	}
	
	/* if picture went out of bounds, we printing an error message */
	if (x_begin < 0 || x_begin > 127 || y_begin < 0 || y_begin  > 63 || col > 127
		|| row > 8 || x_begin + col - 1 > 127 || y_begin + row * 8 - 1 > 63) {
		err = OLED_EBOUNDS;
		_print_error(oled, err);
		return err;
	}

	if (op_flag < REPLACE || op_flag > XOR) {
		err = OLED_EPARAMS;
		_print_error(oled, err);
		return err;
	}

	uint8_t start_page = y_begin / 8; // display page.
	uint8_t stop_page = start_page + row;
	uint16_t indx = 0; // data array index
	uint16_t x_start; // x_start, x_stop - fb array indexes
	uint16_t x_stop;

	/* When coordinates y_begin multiples of 8, we just filling bytes */
	if (y_begin % 8 == 0) {
		for (uint8_t page = start_page; page < stop_page; page++) {
			x_start = (page << 7) + x_begin;
			x_stop = x_start + col;
			if (arr_flag == FILL_PICTURE) {
				for (uint16_t i = x_start; i < x_stop; i++, indx++) {
					func_ptr[op_flag](&oled->frame_buffer[i], &data[indx]);
				}
			} else if (arr_flag == FILL_BLACK || arr_flag == FILL_WHITE) {
				for (uint16_t i = x_start; i < x_stop; i++) {
					func_ptr[op_flag](&oled->frame_buffer[i], &arr_flag);
				}
			} else {
				err = OLED_EPARAMS;
				_print_error(oled, err);
				return err;
			}
		}
		return err;
	}
	
	uint8_t top_offset = y_begin % 8;
	uint8_t bot_offset = 8 - y_begin % 8;

	uint8_t top_and_mask = 0xFF >> bot_offset;
	uint8_t bot_and_mask = 0xFF << top_offset;

	if (arr_flag == FILL_BLACK || arr_flag == FILL_WHITE) {

		/* fill top bits */
		x_start = (start_page << 7) + x_begin;
		x_stop = x_start + col;
		err = _put_region_fill_bits(oled, x_start, x_stop, top_offset, 
				top_and_mask, arr_flag, op_flag, TOP_BITS);

		/* fill bottom bits */
		x_start = (stop_page << 7) + x_begin;
		x_stop = x_start + col;
		err = _put_region_fill_bits(oled, x_start, x_stop, bot_offset, 
				bot_and_mask, arr_flag, op_flag, BOT_BITS);
		
		/* fill bytes */
		for (uint8_t page = start_page + 1; page < stop_page; page++) {
			x_start = (page << 7) + x_begin;
			x_stop = x_start + col;
			for (uint16_t i = x_start; i < x_stop; i++) {
				func_ptr[op_flag](&oled->frame_buffer[i], &arr_flag);
			}
		}
		return err;
	}
				
	if (arr_flag == FILL_PICTURE) {
					
		const uint8_t top_read_mask = 0xFF << bot_offset;
		const uint8_t bot_read_mask = 0xFF >> top_offset;

		uint16_t i = (start_page << 7) + x_begin;
		/* move thrue picture array */
		for (uint8_t indx = 0; indx < col * row; indx++, i++) {
			
			/* when we finish write row, move to the next page */
			if (indx % col == 0 && indx > 0) {
				i = ((++start_page) << 7) + x_begin;
			}

			/* read top and bottom bits of a single byte of the data array */
			uint8_t top_bits = (data[indx] | top_read_mask);
			uint8_t bot_bits = (data[indx] | bot_read_mask);

			/* prepare to write bits */
			top_bits <<= top_offset;
			bot_bits >>= bot_offset;
			
			/* creating masks */
			uint8_t mask_or_xor_nand_top = top_bits;
			uint8_t mask_or_xor_nand_bot = bot_bits;
			uint8_t mask_and_nor_top = top_bits | top_and_mask;
			uint8_t mask_and_nor_bot = bot_bits | bot_and_mask;
			
			if (op_flag == REPLACE) {
				oled->frame_buffer[i] |= mask_or_xor_nand_top;
				oled->frame_buffer[i] &= mask_and_nor_top;
				oled->frame_buffer[i + 128] |= mask_or_xor_nand_bot;
				oled->frame_buffer[i + 128] &= mask_and_nor_bot;	 
			} else if (op_flag == OR || op_flag == XOR || op_flag == NAND) {
				func_ptr[op_flag](&oled->frame_buffer[i], &mask_or_xor_nand_top);
				func_ptr[op_flag](&oled->frame_buffer[i + 128], &mask_or_xor_nand_bot);
			} else if (op_flag == AND || op_flag == NOR) {
				func_ptr[op_flag](&oled->frame_buffer[i], &mask_and_nor_top);
				func_ptr[op_flag](&oled->frame_buffer[i + 128], &mask_and_nor_bot);
			}
		}
		return err;
	}
	/* if arr_flag did not match, return an error */
	err = OLED_EPARAMS;
	_print_error(oled, err);
	return err;
}

/******************************************************/
/******************************************************/
