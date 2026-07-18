#include "cpp_package/mdns_service.h"

void MDNSService::create_service(AvahiClient *c) 
{
    int ret;
    auto *service = this;
    
    if (!service->group) {
        service->group = avahi_entry_group_new(c, MDNSService::entry_group_callback, service);
    }

    if (avahi_entry_group_is_empty(service->group)) {
        ret = avahi_entry_group_add_service(
            service->group, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC, (AvahiPublishFlags)0,
            service->service_name, "_robot._tcp", nullptr, nullptr, 2004, nullptr);

        if (ret < 0) {
            avahi_threaded_poll_quit(service->thread_poll);
            return;
        }

        ret = avahi_entry_group_commit(service->group);
        if (ret < 0) {
            avahi_threaded_poll_quit(service->thread_poll);
            return;
        }
    }
}

void MDNSService::entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state, AVAHI_GCC_UNUSED void *userdata) 
{
    auto *service = static_cast<MDNSService *>(userdata);

    switch (state) {
        case AVAHI_ENTRY_GROUP_COLLISION:
            char *n;
            n = avahi_alternative_service_name(service->service_name);
            avahi_free(service->service_name);
            service->service_name = n;
            service->create_service(avahi_entry_group_get_client(g));
            break;
        case AVAHI_ENTRY_GROUP_FAILURE:
            avahi_threaded_poll_quit(service->thread_poll);
            break;
        default:
            break;
    }
}

void MDNSService::client_callback(AvahiClient *c, AvahiClientState state, AVAHI_GCC_UNUSED void *userdata) {
    auto *service = static_cast<MDNSService *>(userdata);

    switch (state) {
        case AVAHI_CLIENT_S_RUNNING:
            avahi_client_set_host_name(c, "mecanum_robot");
            service->create_service(c);
            break;

        case AVAHI_CLIENT_S_COLLISION:
        case AVAHI_CLIENT_S_REGISTERING:
            if (service->group) 
            {
                avahi_entry_group_reset(service->group);
            }
            break;

        case AVAHI_CLIENT_FAILURE:
            if (service->thread_poll) 
            {
                avahi_threaded_poll_quit(service->thread_poll);
            }
            break;

        default:
            break;
    }
}

MDNSService::MDNSService() {
    thread_poll = avahi_threaded_poll_new();
    if (!thread_poll) {
        return;
    }
    
    int error;
    service_name = avahi_strdup("Mecanum Robot");
    client = avahi_client_new(
        avahi_threaded_poll_get(thread_poll),
        (AvahiClientFlags)0,
        MDNSService::client_callback,
        this,
        &error
    );

    if (!client) {
        avahi_threaded_poll_free(thread_poll);
        thread_poll = nullptr;
        return;
    }

    if (avahi_threaded_poll_start(thread_poll) < 0) {
        avahi_client_free(client);
        client = nullptr;
        avahi_threaded_poll_free(thread_poll);
        thread_poll = nullptr;
        return;
    }
}

MDNSService::~MDNSService() {
    if (thread_poll) {
        avahi_threaded_poll_stop(thread_poll);
    }
    if (group) {
        avahi_entry_group_free(group);
        group = nullptr;
    }
    if (client) {
        avahi_client_free(client);
        client = nullptr;
    }
    if (thread_poll) {
        avahi_threaded_poll_free(thread_poll);
        thread_poll = nullptr;
    }
}