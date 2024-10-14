#ifndef SSID_CONFIG_H
#define SSID_CONFIG_H

#define DEFAULT_SSID                          "cicada"
#define DEFAULT_PSK                           "cosmos327"

#define DEFAULT_IP_ADDRESS                    "192.168.43.243"
#define DEFAULT_GATEWAY_ADDRESS               "192.168.43.1"
#define DEFAULT_SUBNET                        "255.255.255.0"
#define DEFAULT_USE_DHCP                      "n"

#define DEFAULT_WIFI_STA_MODE                 "n"
// our param saving system supports only string literal
// to write integer value we inject them to string as ASCII hex number
#define DEFAULT_WIFI_CHANNEL                  "\x07"
#define DEFAULT_WIFI_TX_POWER_LEVEL           "\x50"
#define DEFAULT_WIFI_PHY_MODE                 "\x02"
#define DEFAULT_WIFI_RATE                     "\x0B"

#endif
