#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <time.h>

/* Sends the monotonic clock of the machine to a host, once a second, so that the host can compare
 * the rate of the clock with its own. The host is the address of the host machine of qemu
 * user-mode networking. */

#define HOST        "10.0.2.2"
#define PORT        47000
#define SAMPLES     12

int main(void)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        exit(EXIT_FAILURE);
    }
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(PORT) };
    inet_pton(AF_INET, HOST, &addr.sin_addr);
    for (int i = 0; i < SAMPLES; i++) {
        struct timespec ts, delay = { 1, 0 };
        clock_gettime(CLOCK_MONOTONIC, &ts);
        char buf[32];
        int len = snprintf(buf, sizeof(buf), "%ld", ts.tv_sec * 1000000000L + ts.tv_nsec);
        if (sendto(fd, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            perror("sendto");   /* the host counts the samples that reach it */
        nanosleep(&delay, 0);
    }
    return 0;
}
