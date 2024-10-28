#include "enc28j60.h"

#define BUF_SIZE 1<<16 

uint8_t buf[BUF_SIZE];

int main() {
    while (1) {
        uint16_t read_bytes = enc28j60_recv_packet(buf, BUF_SIZE);
        if (read_bytes > 0) {
            send_to_others();
        }

    }
}

uint8_t send_to_others() {
    return 0;
}
