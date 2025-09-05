#include <string.h>
#include <stdio.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"

#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "mdns.h"

#include "esp_http_server.h"
#include "esp_timer.h"
#include "driver/ledc.h"

static const char *TAG = "REMOTE_SERVO";

/* ====== PIN MAP (W5500 + Servo + Buttons + LED) =======
   SPI  : MOSI=GPIO23, MISO=GPIO19, SCLK=GPIO18
   W5500: CS=GPIO5, INT=GPIO4, RST=GPIO16
   SERVO: PWM GPIO21 (5 V power externally), 50 Hz
   BUTTONS:
     - RESET is hardware to EN (not read in code)
     - FACTORY DEFAULT: GPIO0 to GND (hold at boot >=10s to wipe NVS)
   STATUS LED: GPIO2 (active high) through 330R (optional)
======================================================== */

#define PIN_SPI_MOSI   23
#define PIN_SPI_MISO   19
#define PIN_SPI_SCLK   18
#define PIN_W5500_CS    5
#define PIN_W5500_INT   4
#define PIN_W5500_RST  16
#define PIN_SERVO_PWM  21
#define PIN_FACTORY    0
#define PIN_LED        2

/* ====== Defaults ====== */
typedef struct {
    char admin_user[32];
    char admin_pass[64];
    // Ethernet static IP
    uint32_t ip;      // in network byte order
    uint32_t netmask;
    uint32_t gw;
    // Wi-Fi AP creds
    char ap_ssid[32];
    char ap_pass[64];
    // Wi-Fi STA creds (join existing network)
    char sta_ssid[32];
    char sta_pass[64];

    // Servo positions (microseconds)
    uint16_t servo_home_us;   // idle / start position
    uint16_t servo_press_us;  // press / stop position
} app_config_t;

static app_config_t g_cfg;

/* ====== NVS Keys ====== */
#define NVS_NAMESPACE "cfg"
#define NVS_KEY_USER  "user"
#define NVS_KEY_PASS  "pass"
#define NVS_KEY_IP    "ip"
#define NVS_KEY_MASK  "mask"
#define NVS_KEY_GW    "gw"
#define NVS_KEY_SSID  "ap_ssid"
#define NVS_KEY_APPSK "ap_pass"
#define NVS_KEY_STA_SSID  "sta_ssid"
#define NVS_KEY_STA_PSK   "sta_pass"
#define NVS_KEY_SV_HOME "sv_home"
#define NVS_KEY_SV_PRESS "sv_press"
/* ====== Auth (very simple Basic Auth) ======
   For production, prefer cookies + CSRF.
   Here we keep it short to get you running fast. */
static bool check_basic_auth(httpd_req_t *req) {
    size_t buf_len = 0;
    char auth[128];
    buf_len = httpd_req_get_hdr_value_len(req, "Authorization") + 1;
    if (buf_len <= 1 || buf_len > sizeof(auth)) return false;
    httpd_req_get_hdr_value_str(req, "Authorization", auth, buf_len);
    /* Expect: "Basic base64(user:pass)" */
    const char *prefix = "Basic ";
    if (strncmp(auth, prefix, strlen(prefix)) != 0) return false;

    // Build "user:pass"
    char up[100];
    snprintf(up, sizeof(up), "%s:%s", g_cfg.admin_user, g_cfg.admin_pass);

    // Base64 compare (quick & dirty decoding)
    // To avoid pulling a base64 lib, compare decoded against up by decoding into a small buffer
    // Minimal base64 decoder:
    const char *b64 = auth + strlen(prefix);
    static const char *B64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int val=0, valb=-8;
    char out[100];
    int oi=0;
    for (const unsigned char *p=(const unsigned char*)b64; *p && *p!='\r' && *p!='\n'; p++) {
        const char *pp = strchr(B64, *p);
        if (*p=='=') { // pad
            break;
        }
        if (!pp) continue;
        val = (val<<6) + (pp - B64);
        valb += 6;
        if (valb >= 0) {
            if (oi < (int)sizeof(out)-1)
                out[oi++] = (char)((val>>valb)&0xFF);
            valb -= 8;
        }
    }
    out[oi] = 0;
    return strcmp(out, up) == 0;
}

static esp_netif_t *eth_netif = NULL;
static esp_netif_t *wifi_ap_netif = NULL;
static esp_netif_t *wifi_sta_netif = NULL;
static httpd_handle_t server = NULL;

/* ====== Servo control via LEDC ====== */
static void servo_write_us(int microseconds);
static void servo_init(void) {
    // (Optional safety) Explicitly set pin as output before LEDC takes over
    gpio_config_t sgio = { .pin_bit_mask = 1ULL<<PIN_SERVO_PWM, .mode = GPIO_MODE_OUTPUT, .pull_up_en = 0, .pull_down_en = 0 };
    gpio_config(&sgio);

    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT, // 0..4095
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,   // standard servo
        .clk_cfg = LEDC_USE_APB_CLK
    };
    ledc_timer_config(&tcfg);
    ledc_channel_config_t ccfg = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .gpio_num   = PIN_SERVO_PWM,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ccfg);
    gpio_set_drive_capability(PIN_SERVO_PWM, GPIO_DRIVE_CAP_3);
    // Initialize output to home position for visible startup pulse
    servo_write_us(g_cfg.servo_home_us ? g_cfg.servo_home_us : 1500);
}

static void servo_write_us(int microseconds) {
    // 50Hz period = 20,000 us; 12-bit duty out of 4095
    // duty = (us / 20000) * 4095
    uint32_t duty = (uint32_t)((microseconds * 4095ULL) / 20000ULL);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

static void servo_home(void) { servo_write_us(g_cfg.servo_home_us); } // neutral
static void servo_press(void){ servo_write_us(g_cfg.servo_press_us); } // adjust per geometry

static void led_set(bool on) {
    gpio_set_level(PIN_LED, on ? 1 : 0);
}

/* ====== Timing helpers ====== */
static void msleep(uint32_t ms){ vTaskDelay(pdMS_TO_TICKS(ms)); }

/* ====== Factory default check ====== */
static void maybe_factory_reset(void) {
    gpio_config_t io = {.pin_bit_mask = 1ULL<<PIN_FACTORY, .mode = GPIO_MODE_INPUT, .pull_up_en = true};
    gpio_config(&io);
    // If held low for >= 10s at boot, wipe NVS
    int64_t start = esp_timer_get_time();
    const int64_t timeout_us = 10LL * 1000 * 1000;
    if (gpio_get_level(PIN_FACTORY) == 0) {
        ESP_LOGW(TAG, "Factory button held - waiting for 10s…");
        while (gpio_get_level(PIN_FACTORY) == 0) {
            if (esp_timer_get_time() - start > timeout_us) {
                ESP_LOGW(TAG, "FACTORY RESET: wiping NVS");
                nvs_flash_erase();
                esp_restart();
            }
            msleep(50);
        }
    }
}

/* ====== Config load/save ====== */
static void cfg_load_defaults(void){
    memset(&g_cfg, 0, sizeof(g_cfg));

#ifdef CONFIG_APP_PRESEED_ENABLE
    strcpy(g_cfg.admin_user, CONFIG_APP_DEFAULT_ADMIN_USER);
    strcpy(g_cfg.admin_pass, CONFIG_APP_DEFAULT_ADMIN_PASS);
    g_cfg.servo_home_us = 1500;
    g_cfg.servo_press_us = 1800;

    ip4_addr_t ip;
    ip4addr_aton(CONFIG_APP_DEFAULT_ETH_IP, &ip); g_cfg.ip = ip.addr;
    ip4addr_aton(CONFIG_APP_DEFAULT_ETH_MASK, &ip); g_cfg.netmask = ip.addr;
    ip4addr_aton(CONFIG_APP_DEFAULT_ETH_GW, &ip);  g_cfg.gw = ip.addr;

    strcpy(g_cfg.ap_ssid,  CONFIG_APP_DEFAULT_AP_SSID);
    strcpy(g_cfg.ap_pass,  CONFIG_APP_DEFAULT_AP_PASS);

    strncpy(g_cfg.sta_ssid, CONFIG_APP_DEFAULT_STA_SSID, sizeof(g_cfg.sta_ssid)-1);
    strncpy(g_cfg.sta_pass, CONFIG_APP_DEFAULT_STA_PASS, sizeof(g_cfg.sta_pass)-1);
#else
    // your current hardcoded defaults (fallback)
    strcpy(g_cfg.admin_user, "admin");
    strcpy(g_cfg.admin_pass, "changeme");
    ip4_addr_t ip;
    ip4addr_aton("192.168.1.50", &ip); g_cfg.ip = ip.addr;
    ip4addr_aton("255.255.255.0", &ip); g_cfg.netmask = ip.addr;
    ip4addr_aton("192.168.1.1", &ip);  g_cfg.gw = ip.addr;
    strcpy(g_cfg.ap_ssid,  "Device-XXXX");
    strcpy(g_cfg.ap_pass,  "setup-1234");
    g_cfg.sta_ssid[0] = '\\0';
    g_cfg.sta_pass[0] = '\\0';
    g_cfg.servo_home_us = 1500;
    g_cfg.servo_press_us = 1800;
#endif
}

static void cfg_save(void){
    nvs_handle_t h; esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return;
    nvs_set_str(h, NVS_KEY_USER, g_cfg.admin_user);
    nvs_set_str(h, NVS_KEY_PASS, g_cfg.admin_pass);
    nvs_set_u32(h, NVS_KEY_IP,    g_cfg.ip);
    nvs_set_u32(h, NVS_KEY_MASK,  g_cfg.netmask);
    nvs_set_u32(h, NVS_KEY_GW,    g_cfg.gw);
    nvs_set_str(h, NVS_KEY_SSID,  g_cfg.ap_ssid);
    nvs_set_str(h, NVS_KEY_APPSK, g_cfg.ap_pass);
    nvs_set_str(h, NVS_KEY_STA_SSID, g_cfg.sta_ssid);
    nvs_set_str(h, NVS_KEY_STA_PSK,  g_cfg.sta_pass);
    nvs_set_u16(h, NVS_KEY_SV_HOME, g_cfg.servo_home_us);
    nvs_set_u16(h, NVS_KEY_SV_PRESS, g_cfg.servo_press_us);
    nvs_commit(h);
    nvs_close(h);
}

static void cfg_load(void){
    cfg_load_defaults();
    nvs_handle_t h; if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    size_t len=0;
    if (nvs_get_str(h, NVS_KEY_USER, NULL, &len) == ESP_OK && len<sizeof(g_cfg.admin_user)) {
        nvs_get_str(h, NVS_KEY_USER, g_cfg.admin_user, &len);
    }
    if (nvs_get_str(h, NVS_KEY_PASS, NULL, &len) == ESP_OK && len<sizeof(g_cfg.admin_pass)) {
        nvs_get_str(h, NVS_KEY_PASS, g_cfg.admin_pass, &len);
    }
    if (nvs_get_str(h, NVS_KEY_STA_SSID, NULL, &len) == ESP_OK && len<sizeof(g_cfg.sta_ssid)) {
        nvs_get_str(h, NVS_KEY_STA_SSID, g_cfg.sta_ssid, &len);
    }
    if (nvs_get_str(h, NVS_KEY_STA_PSK, NULL, &len) == ESP_OK && len<sizeof(g_cfg.sta_pass)) {
        nvs_get_str(h, NVS_KEY_STA_PSK, g_cfg.sta_pass, &len);
    }
    uint32_t v;
    if (nvs_get_u32(h, NVS_KEY_IP, &v) == ESP_OK) g_cfg.ip=v;
    if (nvs_get_u32(h, NVS_KEY_MASK, &v) == ESP_OK) g_cfg.netmask=v;
    if (nvs_get_u32(h, NVS_KEY_GW, &v) == ESP_OK) g_cfg.gw=v;
    if (nvs_get_str(h, NVS_KEY_SSID, NULL, &len) == ESP_OK && len<sizeof(g_cfg.ap_ssid)) {
        nvs_get_str(h, NVS_KEY_SSID, g_cfg.ap_ssid, &len);
    }
    if (nvs_get_str(h, NVS_KEY_APPSK, NULL, &len) == ESP_OK && len<sizeof(g_cfg.ap_pass)) {
        nvs_get_str(h, NVS_KEY_APPSK, g_cfg.ap_pass, &len);
    }
    uint16_t sv;
    if (nvs_get_u16(h, NVS_KEY_SV_HOME, &sv) == ESP_OK) g_cfg.servo_home_us = sv; else if (!g_cfg.servo_home_us) g_cfg.servo_home_us = 1500;
    if (nvs_get_u16(h, NVS_KEY_SV_PRESS, &sv) == ESP_OK) g_cfg.servo_press_us = sv; else if (!g_cfg.servo_press_us) g_cfg.servo_press_us = 1800;
    nvs_close(h);
}

/* ====== Minimal HTML (served inline) ====== */
static const char *INDEX_HTML =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Remote Servo Power</title>"
"<style>body{font-family:sans-serif;max-width:560px;margin:24px auto;padding:0 12px}"
"button{font-size:18px;padding:12px 16px;margin:8px 8px;border-radius:10px;border:1px solid #bbb}"
"input{font-size:16px;padding:8px;margin:4px 0;width:100%}fieldset{margin-top:18px}</style></head>"
"<body><h2>Remote Servo Power</h2>"
"<p>Use <b>Short</b> for power toggle; <b>Long</b> (~5s) for force-off.</p>"
"<div><button onclick='act(\"short\")'>Short Press</button>"
"<button onclick='act(\"long\")'>Long Press</button></div>"
"<fieldset><legend>Config</legend>"
"<label>Username<input id=u></label>"
"<label>Password<input id=p type=password></label>"
"<label>Ethernet IP<input id=ip placeholder='192.168.1.50'></label>"
"<label>Netmask<input id=mask placeholder='255.255.255.0'></label>"
"<label>Gateway<input id=gw placeholder='192.168.1.1'></label>"
"<label>AP SSID<input id=ssid></label>"
"<label>AP Password<input id=psk type=password></label>"

"<hr><b>Join Existing Wi-Fi (STA)</b>"
"<label>STA SSID<input id=sta_ssid></label>"
"<label>STA Password<input id=sta_psk type=password></label>"
"<button onclick='joinsta()'>Join Wi-Fi</button>"

"<hr><b>Servo Positions</b>"
"<label>Home (µs)<input id=svh type=number min=800 max=2200 step=10></label>"
"<label>Press (µs)<input id=svp type=number min=800 max=2200 step=10></label>"
"<div style='margin-top:8px'>"
"<input id=svslider type=range min=800 max=2200 step=5 oninput='slidermove(this.value)'>"
"<div id=svreadout>1500 µs</div>"
"</div>"
"<div style='display:flex;gap:8px;margin-top:8px'>"
"<button onclick='testservo(\"home\")'>Test Home</button>"
"<button onclick='testservo(\"press\")'>Test Press</button>"
"</div>"
"<button onclick='save()'>Save</button>"
"</fieldset>"
"<script>"
"async function act(kind){"
" const r=await fetch('/api/actuator',{method:'POST',headers:{'Content-Type':'application/json'},"
" body:JSON.stringify({action:kind})}); if(!r.ok) alert('Actuator failed');}"
"async function load(){const r=await fetch('/api/config'); if(r.ok){const c=await r.json();"
" u.value=c.user; ip.value=c.ip; mask.value=c.mask; gw.value=c.gw; ssid.value=c.ap_ssid; if(c.sta_ssid) sta_ssid.value=c.sta_ssid; if(c.servo_home_us){ svh.value=c.servo_home_us; svslider.value=c.servo_home_us; svreadout.innerText=c.servo_home_us+' µs'; } if(c.servo_press_us) svp.value=c.servo_press_us;}}"
"async function joinsta(){const body={ssid:sta_ssid.value, pass:sta_psk.value};"
" const r=await fetch('/api/wifi/join',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});"
" if(r.ok) alert('Joining… check logs and STA IP.'); else alert('Join failed');}"
"let _svdeb=null;"
"async function testservo(kind){const us=(kind==='home')?parseInt(svh.value||0):parseInt(svp.value||0);"
" await fetch('/api/servo/test',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({us:us})});}"
"function slidermove(v){svreadout.innerText=v+' µs'; if(_svdeb) clearTimeout(_svdeb); _svdeb=setTimeout(async()=>{"
" await fetch('/api/servo/test',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({us:parseInt(v)})}); _svdeb=null;},120);}"
"async function save(){const body={user:u.value, pass:p.value, ip:ip.value, mask:mask.value, gw:gw.value, ap_ssid:ssid.value, ap_pass:psk.value, servo_home_us:parseInt(svh.value||0), servo_press_us:parseInt(svp.value||0)};"
" const r=await fetch('/api/config',{method:'PUT',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});"
" if(r.ok) alert('Saved. Reboot may be needed.'); else alert('Save failed');}"
"load();</script></body></html>";

/* ====== HTTP handlers ====== */
static esp_err_t auth_guard(httpd_req_t *req) {
    if (!check_basic_auth(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"remote-servo\"");
        httpd_resp_send(req, "auth required", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t root_get(httpd_req_t *req){
    if (auth_guard(req) != ESP_OK) return ESP_FAIL;
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get(httpd_req_t *req){
    if (auth_guard(req) != ESP_OK) return ESP_FAIL;
    char buf[384];
    ip4_addr_t ip = { .addr = g_cfg.ip };
    ip4_addr_t mask = { .addr = g_cfg.netmask };
    ip4_addr_t gw = { .addr = g_cfg.gw };
    snprintf(buf, sizeof(buf),
        "{\"user\":\"%s\",\"ip\":\"%s\",\"mask\":\"%s\",\"gw\":\"%s\",\"ap_ssid\":\"%s\",\"sta_ssid\":\"%s\",\"servo_home_us\":%u,\"servo_press_us\":%u}",
        g_cfg.admin_user, ip4addr_ntoa(&ip), ip4addr_ntoa(&mask), ip4addr_ntoa(&gw), g_cfg.ap_ssid, g_cfg.sta_ssid,
        (unsigned)g_cfg.servo_home_us, (unsigned)g_cfg.servo_press_us);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t config_put(httpd_req_t *req){
    if (auth_guard(req) != ESP_OK) return ESP_FAIL;
    char body[512]; int len = httpd_req_recv(req, body, sizeof(body)-1);
    if (len <= 0) {
        return ESP_FAIL;
    }
    body[len] = 0;

    // ultra-light JSON parse
    // Expect keys: user, pass, ip, mask, gw, ap_ssid, ap_pass
    #define GET_STR(KEY,DEST,SZ) do{ char *p=strstr(body, "\"" KEY "\""); \
        if(p){ p=strchr(p,':'); if(p){ p++; while(*p==' '||*p=='\"') p++; char *q=p; while(*q && *q!='\"' && *q!='}' && *q!=',' && (q-p)<(SZ-1)) q++; \
        size_t n=q-p; if(n>0){ memcpy(DEST,p,n); DEST[n]=0; } } } }while(0)

    char user[32]={0},pass[64]={0}, ip[32]={0},mask[32]={0},gw[32]={0}, ssid[32]={0}, apsk[64]={0}, svh[8]={0}, svp[8]={0};
    GET_STR("user", user, sizeof(user));
    GET_STR("pass", pass, sizeof(pass));
    GET_STR("ip", ip, sizeof(ip));
    GET_STR("mask", mask, sizeof(mask));
    GET_STR("gw", gw, sizeof(gw));
    GET_STR("ap_ssid", ssid, sizeof(ssid));
    GET_STR("ap_pass", apsk, sizeof(apsk));
    GET_STR("servo_home_us", svh, sizeof(svh));
    GET_STR("servo_press_us", svp, sizeof(svp));

    if (user[0]) strncpy(g_cfg.admin_user, user, sizeof(g_cfg.admin_user)-1);
    if (pass[0]) strncpy(g_cfg.admin_pass, pass, sizeof(g_cfg.admin_pass)-1);
    ip4_addr_t a;
    if (ip4addr_aton(ip, &a)) g_cfg.ip = a.addr;
    if (ip4addr_aton(mask, &a)) g_cfg.netmask = a.addr;
    if (ip4addr_aton(gw, &a)) g_cfg.gw = a.addr;
    if (ssid[0]) strncpy(g_cfg.ap_ssid, ssid, sizeof(g_cfg.ap_ssid)-1);
    if (apsk[0]) strncpy(g_cfg.ap_pass, apsk, sizeof(g_cfg.ap_pass)-1);

    if (svh[0]) { int v = atoi(svh); if (v < 800) v = 800; if (v > 2200) v = 2200; g_cfg.servo_home_us = (uint16_t)v; }
    if (svp[0]) { int v = atoi(svp); if (v < 800) v = 800; if (v > 2200) v = 2200; g_cfg.servo_press_us = (uint16_t)v; }

    cfg_save();
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t wifi_join_sta(const char* ssid, const char* pass);

static esp_err_t wifi_join_post(httpd_req_t *req){
    if (auth_guard(req) != ESP_OK) return ESP_FAIL;
    char body[256]; int len = httpd_req_recv(req, body, sizeof(body)-1);
    if (len <= 0) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no body"); return ESP_FAIL; }
    body[len] = 0;

    char ssid[32]={0}, psk[64]={0};
    #define GET_STR(KEY,DEST,SZ) do{ char *p=strstr(body, "\"" KEY "\""); \
        if(p){ p=strchr(p,':'); if(p){ p++; while(*p==' '||*p=='\"') p++; char *q=p; while(*q && *q!='\"' && *q!='}' && *q!=',' && (q-p)<(SZ-1)) q++; \
        size_t n=q-p; if(n>0){ memcpy(DEST,p,n); DEST[n]=0; } } } }while(0)
    GET_STR("ssid", ssid, sizeof(ssid));
    GET_STR("pass", psk,  sizeof(psk));

    if (!ssid[0]) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid required"); return ESP_FAIL; }
    esp_err_t r = wifi_join_sta(ssid, psk);
    if (r != ESP_OK) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "join failed"); return ESP_FAIL; }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t servo_test_post(httpd_req_t *req){
    if (auth_guard(req) != ESP_OK) return ESP_FAIL;
    char body[64]; int len = httpd_req_recv(req, body, sizeof(body)-1);
    if (len <= 0) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no body"); return ESP_FAIL; }
    body[len] = 0;

    char us_str[8] = {0};
    #define GET_STR_TEST(KEY,DEST,SZ) do{ char *p=strstr(body, "\"" KEY "\""); \
        if(p){ p=strchr(p,':'); if(p){ p++; while(*p==' '||*p=='\"') p++; char *q=p; while(*q && *q!='\"' && *q!='}' && *q!=',' && (q-p)<(SZ-1)) q++; \
        size_t n=q-p; if(n>0){ memcpy(DEST,p,n); DEST[n]=0; } } } }while(0)
    GET_STR_TEST("us", us_str, sizeof(us_str));

    if (!us_str[0]) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "us required"); return ESP_FAIL; }
    int v = atoi(us_str);
    if (v < 500) {
        v = 500;
    }
    if (v > 2600) {
        v = 2600; // slightly wider for testing
    }
    servo_write_us(v);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t actuator_post(httpd_req_t *req){
    if (auth_guard(req) != ESP_OK) return ESP_FAIL;
    char body[64]; int len=httpd_req_recv(req, body, sizeof(body)-1);
    if (len<=0) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no body"); return ESP_FAIL; }
    body[len]=0;
    bool is_long = strstr(body, "long")!=NULL;
    ESP_LOGI(TAG, "Actuator: %s press", is_long?"LONG":"SHORT");
    led_set(true);
    servo_press();
    if (is_long) msleep(5000);
    else msleep(200);
    servo_home();
    led_set(false);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static httpd_handle_t start_web(void){
    httpd_config_t conf = HTTPD_DEFAULT_CONFIG();
    conf.stack_size = 8192;
    if (httpd_start(&server, &conf) != ESP_OK) return NULL;

    httpd_uri_t root = {.uri="/", .method=HTTP_GET, .handler=root_get};
    httpd_uri_t stat = {.uri="/api/config", .method=HTTP_GET, .handler=status_get};
    httpd_uri_t confput = {.uri="/api/config", .method=HTTP_PUT, .handler=config_put};
    httpd_uri_t act  = {.uri="/api/actuator", .method=HTTP_POST, .handler=actuator_post};
    httpd_uri_t join = {.uri="/api/wifi/join", .method=HTTP_POST, .handler=wifi_join_post};
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &stat);
    httpd_register_uri_handler(server, &confput);
    httpd_register_uri_handler(server, &act);
    httpd_register_uri_handler(server, &join);
    httpd_uri_t st = {.uri="/api/servo/test", .method=HTTP_POST, .handler=servo_test_post};
    httpd_register_uri_handler(server, &st);
    return server;
}

// Helper to ensure GPIO ISR service is installed (safe if called multiple times)
static void ensure_gpio_isr(void) {
    // Install ISR service once. If already installed, ignore the state error.
    esp_err_t r = gpio_install_isr_service(0);
    if (r != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(r);
    }
}

/* ====== Ethernet (W5500 SPI) ====== */
// static void eth_init(void) { // temporary. to disable warning
__attribute__((unused)) static void eth_init(void) {
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&cfg);
    assert(eth_netif);

    // Static IP
    esp_netif_dhcpc_stop(eth_netif);
    esp_netif_ip_info_t ipi = {0};
    ipi.ip.addr     = g_cfg.ip;
    ipi.netmask.addr= g_cfg.netmask;
    ipi.gw.addr     = g_cfg.gw;
    ESP_ERROR_CHECK( esp_netif_set_ip_info(eth_netif, &ipi) );

    // SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_SPI_MISO,
        .mosi_io_num = PIN_SPI_MOSI,
        .sclk_io_num = PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };
    ESP_ERROR_CHECK( spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO) );

    // W5500 SPI device configuration (full-duplex; driver will allocate device internally)
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 13 * 1000 * 1000, // start conservative; raise if stable
        .spics_io_num = PIN_W5500_CS,
        .queue_size = 20,
        .flags = 0
    };

    // Prepare INT pin and ISR service for W5500
    gpio_config_t intr_io = {
        .pin_bit_mask = 1ULL << PIN_W5500_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&intr_io);
    ensure_gpio_isr();

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.reset_gpio_num = PIN_W5500_RST;

    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &devcfg);
    w5500_config.int_gpio_num = PIN_W5500_INT;

    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK( esp_eth_driver_install(&eth_config, &eth_handle) );

    // Attach to TCP/IP stack
    ESP_ERROR_CHECK( esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)) );
    ESP_ERROR_CHECK( esp_eth_start(eth_handle) );
}

/* ====== Wi-Fi STA (connect to network) ====== */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "STA started, connecting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "STA disconnected, retrying in 2s");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* e = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "STA got IP: %s", ip4addr_ntoa((const ip4_addr_t*)&e->ip_info.ip));
    }
}

static esp_err_t wifi_join_sta(const char* ssid, const char* pass) {
    if (ssid && ssid[0]) strncpy(g_cfg.sta_ssid, ssid, sizeof(g_cfg.sta_ssid)-1);
    if (pass && pass[0]) strncpy(g_cfg.sta_pass, pass, sizeof(g_cfg.sta_pass)-1);
    cfg_save();

    wifi_config_t sta = {0};
    snprintf((char*)sta.sta.ssid, sizeof(sta.sta.ssid), "%s", g_cfg.sta_ssid);
    snprintf((char*)sta.sta.password, sizeof(sta.sta.password), "%s", g_cfg.sta_pass);
    sta.sta.threshold.authmode = (strlen((char*)sta.sta.password) >= 8) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_APSTA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta) );
    return esp_wifi_connect();
}

/* ====== Wi-Fi AP (setup network) ====== */
static void wifi_ap_init(void){
    ESP_ERROR_CHECK( esp_netif_init() ); // ensure called

    wifi_ap_netif  = esp_netif_create_default_wifi_ap();
    wifi_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t wicfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&wicfg) );

    ESP_ERROR_CHECK( esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL) );

    wifi_config_t ap = {0};
    snprintf((char*)ap.ap.ssid, sizeof(ap.ap.ssid), "%s", g_cfg.ap_ssid);
    ap.ap.ssid_len = strlen((const char*)ap.ap.ssid);
    snprintf((char*)ap.ap.password, sizeof(ap.ap.password), "%s", g_cfg.ap_pass);
    ap.ap.max_connection = 4;
    ap.ap.authmode = (strlen((char*)ap.ap.password) >= 8) ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;
    ap.ap.channel = 6;
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_APSTA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_AP, &ap) );

    if (g_cfg.sta_ssid[0]) {
        wifi_config_t sta = {0};
        snprintf((char*)sta.sta.ssid, sizeof(sta.sta.ssid), "%s", g_cfg.sta_ssid);
        snprintf((char*)sta.sta.password, sizeof(sta.sta.password), "%s", g_cfg.sta_pass);
        sta.sta.threshold.authmode = (strlen((char*)sta.sta.password) >= 8) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta) );
    }

    ESP_ERROR_CHECK( esp_wifi_start() );

    if (g_cfg.sta_ssid[0]) {
        esp_wifi_connect();
    }
}

/* ====== mDNS ====== */
__attribute__((unused)) static void start_mdns(void){
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char host[32]; snprintf(host, sizeof(host), "device-%02X%02X", mac[4], mac[5]);
    ESP_ERROR_CHECK( mdns_init() );
    mdns_hostname_set(host);
    mdns_instance_name_set("Remote Servo");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
}

/* ====== App entry ====== */
void app_main(void)
{
    // LED
    gpio_config_t led_io = {.pin_bit_mask=1ULL<<PIN_LED, .mode=GPIO_MODE_OUTPUT};
    gpio_config(&led_io);
    led_set(false);

    ESP_ERROR_CHECK( nvs_flash_init() );
    maybe_factory_reset();
    cfg_load();

    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    ESP_ERROR_CHECK( esp_netif_init() );

    servo_init();
    servo_home();

    // Bring up Ethernet (static IP) and Wi-Fi AP
    // eth_init(); // Uncomment to enable Ethernet
    wifi_ap_init();
    // start_mdns(); // Uncomment to enable mDNS

    // Web server
    start_web();

    ESP_LOGI(TAG, "Ready. HTTP on http://%s (Ethernet) and http://192.168.4.1 (AP)", ip4addr_ntoa((const ip4_addr_t*)&g_cfg.ip));
    ESP_LOGI(TAG, "Use Basic Auth: username=%s password=%s", g_cfg.admin_user, g_cfg.admin_pass);
}
