#include <string.h>
#include <stdio.h>

char debug_buf[100]; 


#define debug_log(...) \
    sprintf(debug_buf, __VA_ARGS__); \
    printf(debug_buf)

int main() {
    debug_log("enc28j60 initialized\r\n");

    int i = 0;
    debug_log("reading from PB%d\n", i+1);

    return 0;
}
