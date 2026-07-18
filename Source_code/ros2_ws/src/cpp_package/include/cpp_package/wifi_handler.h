#ifndef WIFI_HANDLER_H_
#define WIFI_HANDLER_H_

#include <wpa_ctrl.h>
#include <hostapd.h>
#include <eloop.h>
#include <string>
#include <cstring>
#include <sstream>
#include <thread>

class WifiHandler 
{
public:
    WifiHandler();
    ~WifiHandler();

    WifiHandler* getInstance() {
        static WifiHandler instance;
        return &instance;
    }
    void connectToNetwork(const std::string& ssid, const std::string& password);
    void startAP();
    void stopAP();      
private:
    struct wpa_ctrl* ctrl;
    struct hostapd_iface *iface;
    struct hostapd_config *conf;
    std::thread *eloop_thread;
    bool ap_running;

};

#endif