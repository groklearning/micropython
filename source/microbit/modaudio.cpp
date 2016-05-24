/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Mark Shannon
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
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

#include "string.h"

extern "C" {

#include "gpio_api.h"
#include "device.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_delay.h"

#include "lib/ticker.h"
#include "py/runtime0.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/mphal.h"
#include "py/gc.h"
#include "microbit/modmicrobit.h"
#include "microbit/modaudio.h"

#define TheTimer NRF_TIMER2
#define TheTimer_IRQn TIMER2_IRQn
#define TheTimer_Anomaly73_Addr (NRF_TIMER2_BASE + 0xC0C)

#define DEBUG_AUDIO 0
#if DEBUG_AUDIO
#include <stdio.h>
#define DEBUG(s) printf s
#else
#define DEBUG(s) (void)0
#endif

static bool running = false;

void audio_stop(void) {
}

void audio_play_source(mp_obj_t src, mp_obj_t pin1, mp_obj_t pin2, bool wait) {
    if (running) {
        audio_stop();
    }
}

STATIC mp_obj_t stop() {
    audio_stop();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(microbit_audio_stop_obj, stop);

STATIC mp_obj_t play(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_source, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_wait,  MP_ARG_BOOL, {.u_bool = true} },
        { MP_QSTR_pin,   MP_ARG_OBJ, {.u_obj = mp_const_none } },
        { MP_QSTR_return_pin,   MP_ARG_OBJ, {.u_obj = mp_const_none } },
    };
    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    mp_obj_t src = args[0].u_obj;
    mp_obj_t pin1 = args[2].u_obj;
    mp_obj_t pin2 = args[3].u_obj;
    audio_play_source(src, pin1, pin2, args[1].u_bool);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(microbit_audio_play_obj, 0, play);

bool microbit_audio_is_playing(void) {
    return running;
}

mp_obj_t is_playing(void) {
    return mp_obj_new_bool(running);
}
MP_DEFINE_CONST_FUN_OBJ_0(microbit_audio_is_playing_obj, is_playing);


microbit_audio_frame_obj_t *new_microbit_audio_frame(void);

STATIC mp_obj_t microbit_audio_frame_new(const mp_obj_type_t *type_in, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
    (void)type_in;
    (void)args;
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return new_microbit_audio_frame();
}

STATIC mp_obj_t audio_frame_subscr(mp_obj_t self_in, mp_obj_t index_in, mp_obj_t value_in) {
    microbit_audio_frame_obj_t *self = (microbit_audio_frame_obj_t *)self_in;
    mp_int_t index = mp_obj_get_int(index_in);
    if (index < 0 || index >= AUDIO_CHUNK_SIZE) {
         mp_raise_ValueError("index out of bounds");
    }
    if (value_in == MP_OBJ_NULL) {
        // delete
        mp_raise_TypeError("cannot delete elements of AudioFrame");
    } else if (value_in == MP_OBJ_SENTINEL) {
        // load
        return MP_OBJ_NEW_SMALL_INT(self->data[index]);
    } else {
        mp_int_t value = mp_obj_get_int(value_in);
        if (value < 0 || value > 255) {
            mp_raise_ValueError("value out of range");
        }
        self->data[index] = value;
        return mp_const_none;
    }
}

static mp_obj_t audio_frame_unary_op(mp_uint_t op, mp_obj_t self_in) {
    (void)self_in;
    switch (op) {
        case MP_UNARY_OP_LEN: return MP_OBJ_NEW_SMALL_INT(32);
        default: return MP_OBJ_NULL; // op not supported
    }
}

static mp_int_t audio_frame_get_buffer(mp_obj_t self_in, mp_buffer_info_t *bufinfo, mp_uint_t flags) {
    (void)flags;
    microbit_audio_frame_obj_t *self = (microbit_audio_frame_obj_t *)self_in;
    bufinfo->buf = self->data;
    bufinfo->len = AUDIO_CHUNK_SIZE;
    bufinfo->typecode = 'b';
    return 0;
}

static void add_into(microbit_audio_frame_obj_t *self, microbit_audio_frame_obj_t *other, bool add) {
    int mult = add ? 1 : -1;
    for (int i = 0; i < AUDIO_CHUNK_SIZE; i++) {
        unsigned val = (int)self->data[i] + mult*(other->data[i]-128);
        // Clamp to 0-255
        if (val > 255) {
            val = (1-(val>>31))*255;
        }
        self->data[i] = val;
    }
}

static microbit_audio_frame_obj_t *copy(microbit_audio_frame_obj_t *self) {
    microbit_audio_frame_obj_t *result = new_microbit_audio_frame();
    for (int i = 0; i < AUDIO_CHUNK_SIZE; i++) {
        result->data[i] = self->data[i];
    }
    return result;
}

mp_obj_t copyfrom(mp_obj_t self_in, mp_obj_t other) {
    microbit_audio_frame_obj_t *self = (microbit_audio_frame_obj_t *)self_in;
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(other, &bufinfo, MP_BUFFER_READ);
    uint32_t len = bufinfo.len > AUDIO_CHUNK_SIZE ? AUDIO_CHUNK_SIZE : bufinfo.len;
    for (uint32_t i = 0; i < len; i++) {
        self->data[i] = ((uint8_t *)bufinfo.buf)[i];
    }
   return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(copyfrom_obj, copyfrom);

union _i2f {
    int32_t bits;
    float value;
};

/* Convert a small float to a fixed-point number */
int32_t float_to_fixed(float f, uint32_t scale) {
    union _i2f x;
    x.value = f;
    int32_t sign = 1-((x.bits>>30)&2);
    /* Subtract 127 from exponent for IEEE-754 and 23 for mantissa scaling */
    int32_t exponent = ((x.bits>>23)&255)-150;
    /* Mantissa scaled by 2**23, including implicit 1 */
    int32_t mantissa = (1<<23) | ((x.bits)&((1<<23)-1));
    int32_t shift = scale+exponent;
    int32_t result;
    if (shift > 0) {
        result = sign*(mantissa<<shift);
    } else if (shift < -31) {
        result = 0;
    } else {
        result = sign*(mantissa>>(-shift));
    }
    // printf("Float %f: %d %d %x (scale %d) => %d\n", f, sign, exponent, mantissa, scale, result);
    return result;
}

static void mult(microbit_audio_frame_obj_t *self, float f) {
    int scaled = float_to_fixed(f, 15);
    for (int i = 0; i < AUDIO_CHUNK_SIZE; i++) {
        unsigned val = ((((int)self->data[i]-128) * scaled) >> 15)+128;
        if (val > 255) {
            val = (1-(val>>31))*255;
        }
        self->data[i] = val;
    }
}

STATIC mp_obj_t audio_frame_binary_op(mp_uint_t op, mp_obj_t lhs_in, mp_obj_t rhs_in) {
    if (mp_obj_get_type(lhs_in) != &microbit_audio_frame_type) {
        return MP_OBJ_NULL; // op not supported
    }
    microbit_audio_frame_obj_t *lhs = (microbit_audio_frame_obj_t *)lhs_in;
    switch(op) {
    case MP_BINARY_OP_ADD:
    case MP_BINARY_OP_SUBTRACT:
        lhs = copy(lhs);
    case MP_BINARY_OP_INPLACE_ADD:
    case MP_BINARY_OP_INPLACE_SUBTRACT:
        if (mp_obj_get_type(rhs_in) != &microbit_audio_frame_type) {
            return MP_OBJ_NULL; // op not supported
        }
        add_into(lhs, (microbit_audio_frame_obj_t *)rhs_in, op==MP_BINARY_OP_ADD||op==MP_BINARY_OP_INPLACE_ADD);
        return lhs;
    case MP_BINARY_OP_MULTIPLY:
        lhs = copy(lhs);
    case MP_BINARY_OP_INPLACE_MULTIPLY:
        mult(lhs, mp_obj_get_float(rhs_in));
        return lhs;
    }
    return MP_OBJ_NULL; // op not supported
}

STATIC const mp_map_elem_t microbit_audio_frame_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_copyfrom), (mp_obj_t)&copyfrom_obj },
};
STATIC MP_DEFINE_CONST_DICT(microbit_audio_frame_locals_dict, microbit_audio_frame_locals_dict_table);


const mp_obj_type_t microbit_audio_frame_type = {
    { &mp_type_type },
    .name = MP_QSTR_AudioFrame,
    .print = NULL,
    .make_new = microbit_audio_frame_new,
    .call = NULL,
    .unary_op = audio_frame_unary_op,
    .binary_op = audio_frame_binary_op,
    .attr = NULL,
    .subscr = audio_frame_subscr,
    .getiter = NULL,
    .iternext = NULL,
    .buffer_p = { .get_buffer = audio_frame_get_buffer },
    .protocol = NULL,
    .parent = NULL,
    .locals_dict = (mp_obj_dict_t*)&microbit_audio_frame_locals_dict,
};

microbit_audio_frame_obj_t *new_microbit_audio_frame(void) {
    microbit_audio_frame_obj_t *res = m_new_obj(microbit_audio_frame_obj_t);
    res->base.type = &microbit_audio_frame_type;
    memset(res->data, 128, AUDIO_CHUNK_SIZE);
    return res;
}

STATIC const mp_map_elem_t audio_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_audio) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_stop), (mp_obj_t)&microbit_audio_stop_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_play), (mp_obj_t)&microbit_audio_play_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_is_playing), (mp_obj_t)&microbit_audio_is_playing_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_AudioFrame), (mp_obj_t)&microbit_audio_frame_type },
};

STATIC MP_DEFINE_CONST_DICT(audio_module_globals, audio_globals_table);

const mp_obj_module_t audio_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&audio_module_globals,
};


}
