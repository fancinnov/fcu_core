#include <stdio.h>

static bool send_lock=false;
static bool receive_lock=false;

bool get_send_lock(void){
    return send_lock;
}

bool get_receive_lock(void){
    return receive_lock;
}

void set_send_lock(bool lock){
    send_lock=lock;
}

void set_receive_lock(bool lock){
    receive_lock=lock;
}
