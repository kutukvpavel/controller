#include "sr_io.h"

#include "user.h"
#include <math.h>

typedef uint32_t buf_t;
#define BUF_WORD_BITS (__CHAR_BIT__ * sizeof(buf_t))

#define BUF_LEN(n) (((n) - 1) / BUF_WORD_BITS + 1)
#define BIT_TO_WORD_IDX(b) ((b) / BUF_WORD_BITS)
#define BIT_REMAINDER_IDX(b) ((b) % BUF_WORD_BITS) 
#define BV(b) (1u << (b))

namespace sr_io
{
    buf_t input_buffer[BUF_LEN(in::INPUT_NUM)];
    buf_t output_buffer[BUF_LEN(out::OUTPUT_NUM)];

    size_t get_inputs(const uint16_t*& buffer)
    {
        buffer = reinterpret_cast<uint16_t*>(&(input_buffer[0]));
        return sizeof(input_buffer) / (sizeof(uint16_t));
    }
    bool get_input(in i)
    {
        return input_buffer[BIT_TO_WORD_IDX(i)] & BV(BIT_REMAINDER_IDX(i));
    }
    void set_output(out i, bool v)
    {
        if (v)
        {
            output_buffer[BIT_TO_WORD_IDX(i)] |= BV(BIT_REMAINDER_IDX(i));
        }
        else
        {
            output_buffer[BIT_TO_WORD_IDX(i)] &= ~BV(BIT_REMAINDER_IDX(i));
        }
    }
    bool get_output(size_t i)
    {
        return output_buffer[BIT_TO_WORD_IDX(i)] & BV(BIT_REMAINDER_IDX(i));
    }

    /// @brief Private, or for debug purposes
    void set_input(size_t i, bool v)
    {
        if (v)
        {
            input_buffer[BIT_TO_WORD_IDX(i)] |= BV(BIT_REMAINDER_IDX(i));
        }
        else
        {
            input_buffer[BIT_TO_WORD_IDX(i)] &= ~BV(BIT_REMAINDER_IDX(i));
        }
    }


    void pulse_output(GPIO_TypeDef* port, uint16_t pin)
    {
        user::uDelay(1);
        LL_GPIO_SetOutputPin(port, pin);
        user::uDelay(1);
        LL_GPIO_ResetOutputPin(port, pin);
        user::uDelay(1);
    }
    void sync()
    {
        static_assert(out::OUTPUT_NUM >= in::INPUT_NUM);

        pulse_output(IN_LOAD_GPIO_Port, IN_LOAD_Pin);
        for (size_t i = 0; i < out::OUTPUT_NUM; i++)
        {
            if (i < in::INPUT_NUM)
            {
                set_input(i, HAL_GPIO_ReadPin(IN_DATA_GPIO_Port, IN_DATA_Pin) == GPIO_PinState::GPIO_PIN_SET);
            }
            if (i < out::OUTPUT_NUM)
            {
                HAL_GPIO_WritePin(OUT_DATA_GPIO_Port, OUT_DATA_Pin, 
                    get_output(i) ? GPIO_PinState::GPIO_PIN_SET : GPIO_PinState::GPIO_PIN_RESET);
            }
            pulse_output(GPIO_SHIFT_GPIO_Port, GPIO_SHIFT_Pin);
        }
        pulse_output(OUT_STORE_GPIO_Port, OUT_STORE_Pin);
    }
}