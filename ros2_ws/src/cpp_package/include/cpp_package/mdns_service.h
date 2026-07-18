#ifndef MDNS_SERVICE_H_
#define MDNS_SERVICE_H_

#include <avahi-client/client.h>
#include <avahi-client/publish.h>
#include <avahi-common/thread-watch.h>
#include <avahi-common/alternative.h>
#include <avahi-common/malloc.h>

class MDNSService {
private:
    AvahiThreadedPoll *thread_poll = nullptr;
    AvahiClient *client = nullptr;
    AvahiEntryGroup *group = nullptr;
    char *service_name = nullptr;
    static void signal_handler(int signum);
public:
    void create_service(AvahiClient *c);
    static void entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state, AVAHI_GCC_UNUSED void *userdata);
    static void client_callback(AvahiClient *c, AvahiClientState state, AVAHI_GCC_UNUSED void *userdata);
    static MDNSService& getInstance() {
        static MDNSService instance;
        return instance;
    }
    MDNSService();
    ~MDNSService();
};

#endif