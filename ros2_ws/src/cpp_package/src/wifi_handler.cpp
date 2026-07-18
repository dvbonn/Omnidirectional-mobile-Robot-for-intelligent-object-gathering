#include "wifi_handler.h"

WifiHandler::WifiHandler()
{
    ctrl = wpa_ctrl_open("/var/run/wpa_supplicant/wlan0");
    iface = nullptr;
    conf = nullptr;
    eloop_thread = nullptr;
    ap_running = false;
}

WifiHandler::~WifiHandler()
{
    stopAP();
    
    if (ctrl) {
        wpa_ctrl_close(ctrl);
    }
}

void WifiHandler::connectToNetwork(const std::string& ssid, const std::string& password)
{
    if (!ctrl) return;
    
    char reply[4096];
    size_t reply_len = sizeof(reply);
    int ret;
    
    ret = wpa_ctrl_request(ctrl, "LIST_NETWORKS", 13, reply, &reply_len, nullptr);
    if (ret < 0) return;

    std::string response(reply, reply_len);
    std::istringstream iss(response);
    std::string line;
    
    while (std::getline(iss, line)) {
        if (line.empty()) continue;
        std::istringstream line_stream(line);
        std::string network_id, network_ssid;
        
        if (std::getline(line_stream, network_id, '\t') &&
            std::getline(line_stream, network_ssid, '\t')) {
            if (network_ssid == ssid) {
                std::string select_cmd = "SELECT_NETWORK " + network_id;
                if (wpa_ctrl_request(ctrl, select_cmd.c_str(), select_cmd.length(), 
                                     nullptr, nullptr, nullptr) >= 0) {
                    return;
                }
            }
        }
    }
    
    reply_len = sizeof(reply);
    ret = wpa_ctrl_request(ctrl, "ADD_NETWORK", 11, reply, &reply_len, nullptr);
    if (ret < 0) return;
    
    std::string network_id(reply, reply_len);
    network_id.erase(network_id.find_last_not_of(" \n\r\t") + 1);
    
    std::string set_ssid_cmd = "SET_NETWORK " + network_id + " ssid \"" + ssid + "\"";
    ret = wpa_ctrl_request(ctrl, set_ssid_cmd.c_str(), set_ssid_cmd.length(), nullptr, nullptr, nullptr);
    if (ret < 0) return;

    std::string set_psk_cmd = "SET_NETWORK " + network_id + " psk \"" + password + "\"";
    ret = wpa_ctrl_request(ctrl, set_psk_cmd.c_str(), set_psk_cmd.length(), nullptr, nullptr, nullptr);
    if (ret < 0) return;
    
    std::string enable_cmd = "ENABLE_NETWORK " + network_id;
    ret = wpa_ctrl_request(ctrl, enable_cmd.c_str(), enable_cmd.length(), nullptr, nullptr, nullptr);
    if (ret < 0) return;
    
    wpa_ctrl_request(ctrl, "SAVE_CONFIG", 11, nullptr, nullptr, nullptr);
}

void WifiHandler::startAP()
{
    if (ap_running) return;
    
    if (eloop_init() < 0) {
        return;
    }
    
    iface = new hostapd_iface();
    if (!iface) return;
    
    conf = hostapd_config_read("/etc/hostapd/hostapd.conf");
    if (!conf) {
        delete iface;
        iface = nullptr;
        return;
    }
    
    iface->conf = conf;
    
    strncpy((char*)conf->bss[0].ssid.ssid, "Hello", HOSTAPD_MAX_SSID_LEN - 1);
    conf->bss[0].ssid.ssid_len = 5;
    conf->bss[0].wpa = 2;
    
    conf->bss[0].wpa_passphrase = strdup("12345678");
    if (!conf->bss[0].wpa_passphrase) {
        hostapd_config_free(conf);
        delete iface;
        iface = nullptr;
        conf = nullptr;
        return;
    }
    
    struct hostapd_data *hapd = hostapd_alloc_bss_data(iface, conf, &conf->bss[0]);
    if (!hapd) {
        free(conf->bss[0].wpa_passphrase);
        hostapd_config_free(conf);
        delete iface;
        iface = nullptr;
        conf = nullptr;
        return;
    }
    
    iface->bss = &hapd;
    iface->num_bss = 1;
    
    if (hostapd_setup_interface(iface) < 0) {
        hostapd_config_free(conf);
        delete iface;
        iface = nullptr;
        conf = nullptr;
        return;
    }
    
    ap_running = true;
    
    eloop_thread = new std::thread([this]() {
        eloop_run();
    });
}

void WifiHandler::stopAP()
{
    if (!ap_running) return;
    
    ap_running = false;
    
    eloop_terminate();
    
    if (eloop_thread) {
        eloop_thread->join();
        delete eloop_thread;
        eloop_thread = nullptr;
    }
    
    if (conf) {
        if (conf->bss[0].wpa_passphrase) {
            free(conf->bss[0].wpa_passphrase);
        }
        hostapd_config_free(conf);
        conf = nullptr;
    }
    
    if (iface) {
        delete iface;
        iface = nullptr;
    }
}
