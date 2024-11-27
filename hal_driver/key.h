#ifndef __KEY_H__
#define __KEY_H__

#include ".\Public\CH554.H" 

typedef enum {
    KEY_EVENT_NONE,
    KEY_EVENT_PRESSED,
    KEY_EVENT_RELEASED
}   key_event_t;

typedef enum {
    KEY_MODE_EDGE,
    KEY_MODE_RISING
} key_mode_t;

typedef void (*key_cb_t)(key_event_t event);

sbit KEY = P1^7;
sbit MODE_PIN = P1^1;


void key_init(void);
void key_scan(void);
void register_key_cb(key_cb_t cb);


#endif /* __KEY_H__ */
