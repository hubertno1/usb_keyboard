#include "key.h"
#include 	".\Public\CH554.H"
#include 	".\Public\debug.h"

typedef enum {
    KEY_STATE_IDLE,
    KEY_STATE_DEBOUNCE_PRESS,
    KEY_STATE_DEBOUNCE_RELEASE,
    KEY_STATE_PRESSED
} key_state_t;

static key_cb_t key_cb = NULL;
static key_state_t key_state = KEY_STATE_IDLE;
static key_mode_t key_mode;
// static uint32_t debounce_time = 0;

void key_init(void)
{
    /* 0000 0001 << 4  ->  0001 0000 */ 
    P1_MOD_OC |= (1 << 7); // Set P1.4 as open-drain
    P1_DIR_PU |= (1 << 7); // Set P1.4 as input

    P1_MOD_OC |= (1 << 1);  // Set P1.7 as open-drain
    P1_DIR_PU |= (1 << 1);  // Set P1.7 as input

    if (MODE_PIN == 0)
    {
        key_mode = KEY_MODE_EDGE;
    }
    else
    {
        key_mode = KEY_MODE_RISING;
    }

}

void register_key_cb(key_cb_t cb)
{
    key_cb = cb;
}

static key_event_t get_key_event(void)
{
    key_event_t event = KEY_EVENT_NONE;
    switch (key_state)
    {
        case KEY_STATE_IDLE:
        {
            if (KEY == 0)
                key_state = KEY_STATE_DEBOUNCE_PRESS;
            
            break;
        }
        case KEY_STATE_DEBOUNCE_PRESS:
        {
            /* 后续考虑定时器，这样不会阻塞程序 */
            mDelaymS(10);
            if (KEY == 0)
            {
                key_state = KEY_STATE_PRESSED;

                if (key_mode == KEY_MODE_EDGE)
                {
                    event = KEY_EVENT_PRESSED;
                }
            }
            else
            {
                key_state = KEY_STATE_IDLE;
            }
            break;
        }
        case KEY_STATE_PRESSED:
        {
            if (KEY == 1)
            {
                key_state = KEY_STATE_DEBOUNCE_RELEASE;
            }
            break;
        }
        case KEY_STATE_DEBOUNCE_RELEASE:
        {
            mDelaymS(10);
            if (KEY == 1)
            {
                key_state = KEY_STATE_IDLE;
                event = KEY_EVENT_RELEASED;
            }
            else
            {
                key_state = KEY_STATE_PRESSED;
            }
            break;
        }

        default:
        {
            key_state = KEY_STATE_IDLE;
            break;
        }

    }

    return event;
}

void key_scan(void)
{
    key_event_t evt = get_key_event();
    if (evt != KEY_EVENT_NONE && key_cb != NULL)
        key_cb(evt);
}
