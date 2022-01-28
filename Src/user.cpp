#include "usbd_cdc_if.h"
#include "user.h"

//Private vars
static int delay_length = 500;
static user::pin_t led_pin = user::pin_t(MASTER_ENABLE_GPIO_Port, MASTER_ENABLE_Pin);

//Private forward-decls
static void cdc_receive(uint8_t* buf, uint32_t* len);

/**
 * PUBLIC, hide behind a namespace
 */

namespace user 
{
    /****
     * MAIN
     * */
    void setup()
    {
        while (CDC_IsConnected() != USBD_OK); //Note: requires DTR (i.e. hardware handshake)
        CDC_Register_RX_Callback(cdc_receive);
        user_prints("Hello World!\n");
    }
    void main()
    {
        LL_GPIO_TogglePin(led_pin.port, led_pin.mask);
        LL_mDelay(delay_length);
    }
}

/**
 * PRIVATE
 */

static void cdc_receive(uint8_t* buf, uint32_t* len)
{
    delay_length /= 2;
    if (delay_length < 99) delay_length = 500;
    user_prints("\nAck\n");
}