#ifndef WIFI_CONNECT_H_
#define WIFI_CONNECT_H_
typedef void (*on_connected_f)(void);
typedef void (*on_failed_f)(void);
typedef struct {
    on_connected_f on_connected;
    on_failed_f on_failed;
    char *ssid;
    char *password;
} connect_wifi_params_t;

void wifi_connect(connect_wifi_params_t);
#endif