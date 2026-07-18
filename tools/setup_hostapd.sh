#!/bin/bash
# Script to start an AP + DHCP + NAT (Internet) on the Jetson Xavier
# Default WAN_IF: enp2s0
set -u

LOG_HOSTAPD=/var/log/hostapd-ap.log
CONF_FILE=/etc/hostapd/hostapd-jetson.conf
SSID="Jetson"
PASS="12345678"
CHAN=6
CC=US
WAN_IF="enp2s0"

echo "=== STARTING AP + DHCP SERVER + NAT ==="

echo "[*] Cleaning up old processes..."
pkill hostapd 2>/dev/null
pkill dnsmasq 2>/dev/null
sleep 2 

echo "[*] Configuring the 8821cu driver..."
echo "REGDOMAIN=$CC" > /etc/default/crda
echo "options cfg80211 ieee80211_regdom=$CC" > /etc/modprobe.d/cfg80211.conf
cat > /etc/modprobe.d/8821cu.conf <<EOF
options 8821cu rtw_drv_log_level=1 rtw_vht_enable=2 rtw_power_mgnt=0 rtw_enusbss=0 rtw_ips_mode=0 rtw_dfs_region_domain=1 rtw_country_code=$CC
EOF

echo "[*] Loading the wlan0 Wi-Fi card config..."
nmcli device set wlan0 managed no 2>/dev/null
ip link set wlan0 down 2>/dev/null

modprobe -r 8821cu 2>/dev/null; sleep 1
modprobe -r cfg80211 2>/dev/null; sleep 1
modprobe cfg80211 ieee80211_regdom=$CC 2>/dev/null
modprobe 8821cu rtw_vht_enable=2 rtw_power_mgnt=0 rtw_enusbss=0 rtw_ips_mode=0 rtw_dfs_region_domain=1 rtw_country_code=$CC

for i in $(seq 1 15); do ip link show wlan0 >/dev/null 2>&1 && break; sleep 1; done
ip link show wlan0 >/dev/null 2>&1 || { echo "!! ERROR: wlan0 card not found"; exit 1; }

iw reg set $CC 2>/dev/null
rfkill unblock wifi 2>/dev/null
sleep 1

echo "[*] Assigning IP 192.168.137.1..."
ip link set wlan0 up
sleep 2
ip addr flush dev wlan0 2>/dev/null
ip addr add 192.168.137.1/24 dev wlan0

mkdir -p /etc/hostapd
cat > "$CONF_FILE" <<EOF
interface=wlan0
driver=nl80211
ssid=$SSID
utf8_ssid=1
country_code=$CC
ieee80211d=1
ieee80211h=0
hw_mode=g
channel=$CHAN
ignore_broadcast_ssid=0
ieee80211n=1
wmm_enabled=1
ht_capab=[HT40+][HT40-][SHORT-GI-20][SHORT-GI-40]
auth_algs=1
wpa=2
wpa_passphrase=$PASS
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
macaddr_acl=0
EOF

echo "[*] Broadcasting Wi-Fi (Hostapd)..."
hostapd -B -f "$LOG_HOSTAPD" "$CONF_FILE"
sleep 2

echo "[*] Starting the DHCP server..."
dnsmasq --interface=wlan0 \
        --port=0 \
        --dhcp-range=192.168.137.50,192.168.137.150,255.255.255.0,12h \
        --dhcp-option=option:router,192.168.137.1 \
        --dhcp-option=6,8.8.8.8,1.1.1.1\
        --bind-interfaces

echo "[*] Setting up Internet forwarding out of $WAN_IF..."
echo 1 > /proc/sys/net/ipv4/ip_forward

iptables -C FORWARD -i wlan0 -o $WAN_IF -j ACCEPT 2>/dev/null || iptables -A FORWARD -i wlan0 -o $WAN_IF -j ACCEPT
iptables -C FORWARD -i $WAN_IF -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || iptables -A FORWARD -i $WAN_IF -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
iptables -t nat -C POSTROUTING -o $WAN_IF -j MASQUERADE 2>/dev/null || iptables -t nat -A POSTROUTING -o $WAN_IF -j MASQUERADE

echo ""
echo "=========================================================="
echo "    DONE! Wi-Fi: $SSID is broadcasting WITH INTERNET."
echo "=========================================================="