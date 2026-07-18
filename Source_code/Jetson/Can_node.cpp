#include <iostream>
#include <linux/can/isotp.h>

int main()
{
    int s;
    int ret;
    struct sockaddr_can addr = {
        .can_family = AF_CAN,
        .can_ifindex = if_nametoindex("can0"),
        .can_addr = {
            .rx_id = 0x100,
            .tx_id = 0x101
        }
    };
    struct can_isotp_fc_options fc_opts = {
        .struct can_isotp_options opts = { //for precode, ignore the syntax
            .bs = 8,
            .stmin = 0x64, // 100ms
            .wftmax = 1
        }
    };
    uint32_t stmin = 1000; // 1ms, for CF transmission



    s = socket(PF_CAN, SOCK_DGRAM, CAN_ISOTP);
    if (s < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return -1;
    }
    ret = setsockopt(s, SOL_CAN_ISOTP, CAN_ISOTP_RECV_FC, &fc_opts, sizeof(fc_opts));
    ret = setsockopt(s, SOL_CAN_ISOTP, CAN_ISOTP_RX_STMIN, &stmin, sizeof(stmin));
    
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        std::cerr << "Error binding socket" << std::endl;
        return -1;
    }


    while(true) {
        char buf[4096];
        ssize_t nbytes = recv(s, buf, sizeof(buf), 0);
        if (nbytes < 0) {
            std::cerr << "Error receiving data" << std::endl;
            break;
        }
        std::cout << "Received " << nbytes << " bytes: ";
        for (ssize_t i = 0; i < nbytes; i++) {
            std::cout << std::hex << static_cast<int>(buf[i]) << " ";
        }
        std::cout << std::dec << std::endl;
        send(s, "This is a long message", 24, 0);
    }
    return 0;
}