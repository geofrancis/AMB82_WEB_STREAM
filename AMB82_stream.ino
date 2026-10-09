#include <WiFi.h>
#include "VideoStream.h"

#define WIFI_SSID       "2.4"
#define WIFI_PASSWORD   "password"
#define SERVER_IP       "192.0.0.13"
#define SERVER_PORT     14080
#define CHANNEL         0
#define FRAME_INTERVAL  100
#define LED_PULSE_MS    20

VideoSetting config(VIDEO_VGA, 30, VIDEO_JPEG, 1);

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("\n=== Booting ===");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi connected.");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    delay(2000);

    Serial.println("[C1] configVideoChannel");
    Camera.configVideoChannel(CHANNEL, config);

    Serial.println("[C2] videoInit");
    Camera.videoInit();
    delay(500);

    Serial.println("[C3] channelBegin");
    Camera.channelBegin(CHANNEL);
    delay(500);
    Serial.println("[C3b] channelBegin returned");

    Serial.println("[W] Warming up pipeline...");
    for (int i = 0; i < 5; i++) {
        uint32_t a = 0, l = 0;
        Serial.print("[W] attempt ");
        Serial.print(i);
        Serial.print(" ... ");
        Camera.getImage(CHANNEL, &a, &l);
        Serial.print("addr=");
        Serial.print(a);
        Serial.print(" len=");
        Serial.println(l);
        delay(200);
    }
    Serial.println("[W] Warmup done");

    Serial.println("[C4] Camera ready.");

    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(100);
        digitalWrite(LED_BUILTIN, LOW);  delay(100);
    }
    delay(1000);
}

bool postFrame(uint32_t img_addr, uint32_t img_len) {
    WiFiClient client;
    if (!client.connect(SERVER_IP, SERVER_PORT)) return false;

    String boundary = "----AMB82Boundary7d91";
    String head = "--" + boundary + "\r\n"
                  "Content-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\n"
                  "Content-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--" + boundary + "--\r\n";
    size_t contentLength = head.length() + img_len + tail.length();

    client.println("POST /image HTTP/1.1");
    client.println("Host: " + String(SERVER_IP));
    client.println("Content-Type: multipart/form-data; boundary=" + boundary);
    client.print("Content-Length: ");
    client.println(contentLength);
    client.println("Connection: close");
    client.println();

    client.print(head);
    client.write((uint8_t*)img_addr, img_len);
    client.print(tail);
    client.stop();
    return true;
}

void loop() {
    uint32_t img_addr = 0, img_len = 0;
    Camera.getImage(CHANNEL, &img_addr, &img_len);

    if (img_addr && img_len) {
        if (postFrame(img_addr, img_len)) {
            digitalWrite(LED_BUILTIN, HIGH); delay(LED_PULSE_MS);
            digitalWrite(LED_BUILTIN, LOW);
        }
    }
    delay(FRAME_INTERVAL);
}
