#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>
#include "CH554.H" 

typedef enum {
    KEY_EVENT_NONE,
    KEY_EVENT_PRESSED,
    KEY_EVENT_RELEASED
}   key_event_t;

typedef void (*key_cb_t)(key_event_t event);

sbit KEY = P1^4;

void key_init(void);
void key_scan(void);
void register_key_cb(key_cb_t cb);


#endif /* __KEY_H__ */
