#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define GROUP "239.255.42.99"
#define PORT 45999

static int fails;

#define CHECK(what, cond) do { int ok = (cond); printf("MCAST %-52s %s\n", what, ok ? "ok" : "FAIL"); if (!ok) fails++; } while (0)

int main(void)
{
    int rx = socket(AF_INET, SOCK_DGRAM, 0), tx = socket(AF_INET, SOCK_DGRAM, 0);
    int one = 1;
    CHECK("sockets", rx >= 0 && tx >= 0);
    CHECK("SO_REUSEADDR", setsockopt(rx, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) == 0);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    CHECK("bind 0.0.0.0", bind(rx, (struct sockaddr *)&a, sizeof(a)) == 0);
    struct ip_mreq m;
    inet_pton(AF_INET, GROUP, &m.imr_multiaddr);
    m.imr_interface.s_addr = htonl(INADDR_ANY);
    int r = setsockopt(rx, IPPROTO_IP, IP_ADD_MEMBERSHIP, &m, sizeof(m));
    printf("MCAST IP_ADD_MEMBERSHIP returned %d errno %d\n", r, r ? errno : 0);
    CHECK("IP_ADD_MEMBERSHIP", r == 0);
    unsigned char ttl = 4, loop = 1;
    r = setsockopt(tx, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
    printf("MCAST IP_MULTICAST_TTL returned %d errno %d\n", r, r ? errno : 0);
    r = setsockopt(tx, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));
    printf("MCAST IP_MULTICAST_LOOP returned %d errno %d\n", r, r ? errno : 0);
    struct in_addr ifa; ifa.s_addr = htonl(INADDR_ANY);
    r = setsockopt(tx, IPPROTO_IP, IP_MULTICAST_IF, &ifa, sizeof(ifa));
    printf("MCAST IP_MULTICAST_IF returned %d errno %d\n", r, r ? errno : 0);
    unsigned char got = 0;
    socklen_t l = sizeof(got);
    r = getsockopt(tx, IPPROTO_IP, IP_MULTICAST_TTL, &got, &l);
    printf("MCAST getsockopt IP_MULTICAST_TTL returned %d value %d (set 4)\n", r, got);
    struct sockaddr_in g = { .sin_family = AF_INET, .sin_port = htons(PORT) };
    inet_pton(AF_INET, GROUP, &g.sin_addr);
    int n = sendto(tx, "hello", 5, 0, (struct sockaddr *)&g, sizeof(g));
    printf("MCAST sendto group returned %d errno %d\n", n, n < 0 ? errno : 0);
    struct timeval tv = { 3, 0 };
    setsockopt(rx, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    char buf[64];
    n = recv(rx, buf, sizeof(buf), 0);
    printf("MCAST recv returned %d errno %d\n", n, n < 0 ? errno : 0);
    CHECK("datagram to the group received on the same machine", n == 5);
    printf(fails ? "MCAST RESULT FAIL %d\n" : "MCAST RESULT OK %d\n", fails);
    return 0;
}
