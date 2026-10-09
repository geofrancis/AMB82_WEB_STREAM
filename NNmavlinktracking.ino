#include <WiFi.h>
#include "StreamIO.h"
#include "VideoStream.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"
#include <MAVLink.h>

// ── Forward declarations ───────────────────────────────────
void sendGimbalCommand(float pitch_deg, float yaw_deg);
void sendTrackingStatus(float x, float y, float radius);
void updateTrackingPID(float targetX, float targetY);
void mavlinkInput();

// ============================================================
// CONFIGURATION
// ============================================================
#define WIFI_SSID          "2.4"
#define WIFI_PASSWORD      "password"

#define SERVER_IP          "192.0.0.13"
#define SERVER_PORT        14080

// Camera channels
#define CHANNEL            0     // JPEG stream (MJPEG)
#define CHANNEL_NN         3     // RGB low-res (NN input)

// NN resolution
#define NN_WIDTH           320
#define NN_HEIGHT          240

// Performance
#define FRAME_INTERVAL     200   // 5 FPS
#define LED_PULSE_MS       20

// MAVLink
#define MAV_SYSID          1
#define MAV_COMPID         197

// PID tuning
float kp = 0.5f;
float ki = 0.05f;
float kd = 0.1f;

float yawIntegral   = 0, yawLastError   = 0;
float pitchIntegral = 0, pitchLastError = 0;

// ============================================================
// CAMERA / NN CONFIG
// ============================================================
VideoSetting config(VIDEO_VGA, 30, VIDEO_JPEG, 1);
VideoSetting configNN(NN_WIDTH, NN_HEIGHT, 10, VIDEO_RGB, 0);

NNObjectDetection ObjDet;
StreamIO videoStreamerNN(1, 1);

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n=== AMB82 NN Webstream + Gimbal ===");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // ── MAVLink UART to Pixhawk (D14=RX, D15=TX) ──
  Serial3.begin(115200);
  Serial.println("[MAV] Serial3 ready");

  // ── WiFi ──
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected.");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  delay(2000);
  config.setJpegQuality(5);

  // ── Camera ──
  Serial.println("[C1] Configuring channels");
  Camera.configVideoChannel(CHANNEL, config);
  Camera.configVideoChannel(CHANNEL_NN, configNN);
  Camera.videoInit();

  // ── NN ──
  Serial.println("[C2] Configuring NN");
  ObjDet.configVideo(configNN);
  ObjDet.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV4TINY, NA_MODEL, NA_MODEL);
  ObjDet.begin();

  // ── OSD ──
  Serial.println("[C3] Configuring OSD");
  OSD.configVideo(CHANNEL, config);
  OSD.begin();

  // ── Channels ──
  Serial.println("[C4] Starting channels");
  Camera.channelBegin(CHANNEL);
  delay(500);
  Camera.channelBegin(CHANNEL_NN);
  delay(500);

  // ── NN pipeline ──
  videoStreamerNN.registerInput(Camera.getStream(CHANNEL_NN));
  videoStreamerNN.setStackSize();
  videoStreamerNN.setTaskPriority();
  videoStreamerNN.registerOutput(ObjDet);
  if (videoStreamerNN.begin() != 0) {
    Serial.println("[C5] NN stream link FAILED");
  } else {
    Serial.println("[C5] NN stream linked");
  }

  Serial.println("Ready.");
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(100);
    digitalWrite(LED_BUILTIN, LOW);  delay(100);
  }
}

// ============================================================
// MAVLink: send gimbal pitch/yaw command
// ============================================================
void sendGimbalCommand(float pitch_deg, float yaw_deg) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_gimbal_manager_set_pitchyaw_pack(
      MAV_SYSID, MAV_COMPID, &msg,
      1,                        // target_system
      1,                        // target_component
      0,                        // flags
      0,                        // gimbal_device_id
      pitch_deg * DEG_TO_RAD,   // pitch (radians)
      yaw_deg * DEG_TO_RAD,     // yaw (radians)
      0.0f,                     // pitch_rate
      0.0f                      // yaw_rate
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial3.write(buf, len);
}

// ============================================================
// MAVLink: send tracking status
// ============================================================
void sendTrackingStatus(float x, float y, float radius) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_camera_tracking_image_status_pack(
      MAV_SYSID, MAV_COMPID, &msg,
      1,                  // tracking_status (1 = active)
      0,                  // tracking_mode (0 = point)
      0,                  // target_data
      x,                  // point_x
      y,                  // point_y
      radius,             // radius
      0.0f, 0.0f, 0.0f,   // rec_top_x, rec_top_y, rec_top_z
      0.0f,               // rec_bottom_x
      0                   // rec_bottom_y (uint8_t)
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial3.write(buf, len);
}

// ============================================================
// PID controller → gimbal rate command
// ============================================================
void updateTrackingPID(float targetX, float targetY) {
  float yawError   = targetX - 0.5f;
  float pitchError = targetY - 0.5f;

  yawIntegral += yawError;
  float yawDerivative = yawError - yawLastError;
  float yawOutput = kp * yawError + ki * yawIntegral + kd * yawDerivative;
  yawLastError = yawError;

  pitchIntegral += pitchError;
  float pitchDerivative = pitchError - pitchLastError;
  float pitchOutput = kp * pitchError + ki * pitchIntegral + kd * pitchDerivative;
  pitchLastError = pitchError;

  // Scale PID output to degrees/second
  sendGimbalCommand(pitchOutput * 30.0f, yawOutput * 30.0f);
}

// ============================================================
// MAVLink: parse incoming messages
// ============================================================
void mavlinkInput() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial3.available()) {
    uint8_t c = Serial3.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
          // Hook for RC override — currently unused
          break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT: {
          // FC alive — could track connection health here
          break;
        }
      }
    }
  }
}

// ============================================================
// DRAW NN OVERLAY + SEND MAVLINK TRACKING
// ============================================================
void drawOverlay() {
  std::vector<ObjectDetectionResult> results = ObjDet.getResult();
  uint16_t im_w = config.width();
  uint16_t im_h = config.height();

  OSD.createBitmap(CHANNEL);

  int count = ObjDet.getResultCount();
  Serial.print("[NN] detections: ");
  Serial.println(count);

  for (int i = 0; i < count; i++) {
    int obj_type = results[i].type();
    int score    = results[i].score();

    const char* name = "OUT_OF_RANGE";
    int filter = -1;
    if (obj_type >= 0 && obj_type < 80) {
      name = itemList[obj_type].objectName;
      filter = itemList[obj_type].filter;
    }

    Serial.print("  [");
    Serial.print(i);
    Serial.print("] type=");
    Serial.print(obj_type);
    Serial.print(" name=");
    Serial.print(name);
    Serial.print(" score=");
    Serial.print(score);
    Serial.print(" filter=");
    Serial.println(filter);

    if (filter != 1) continue;

    ObjectDetectionResult item = results[i];
    int xmin = (int)(item.xMin() * im_w);
    int xmax = (int)(item.xMax() * im_w);
    int ymin = (int)(item.yMin() * im_h);
    int ymax = (int)(item.yMax() * im_h);

    OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);

    char label[32];
    snprintf(label, sizeof(label), "%s %d", name, score);
    OSD.drawText(CHANNEL, xmin, ymin - OSD.getTextHeight(CHANNEL), label, OSD_COLOR_CYAN);

    // ── Gimbal tracking: only for "person" (type 0) ──
    if (obj_type == 0) {
      float cx = (item.xMin() + item.xMax()) / 2.0f;
      float cy = (item.yMin() + item.yMax()) / 2.0f;

      float w = item.xMax() - item.xMin();
      float h = item.yMax() - item.yMin();
      float trackedRadius = (w > h ? w : h) / 2.0f;

      sendTrackingStatus(cx, cy, trackedRadius);
      updateTrackingPID(cx, cy);
    }
  }
  OSD.update(CHANNEL);
}

// ============================================================
// POST FRAME TO LOCAL MJPEG SERVER
// ============================================================
bool postFrame(uint32_t img_addr, uint32_t img_len) {
  WiFiClient client;
  if (!client.connect(SERVER_IP, SERVER_PORT)) return false;

  String b = "----AMB82Boundary7d91";
  String head = "--" + b + "\r\n"
                "Content-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\n"
                "Content-Type: image/jpeg\r\n\r\n";
  String tail = "\r\n--" + b + "--\r\n";
  size_t total = head.length() + img_len + tail.length();

  client.println("POST /image HTTP/1.1");
  client.println("Host: " + String(SERVER_IP));
  client.println("Content-Type: multipart/form-data; boundary=" + b);
  client.println("Content-Length: " + String(total));
  client.println("Connection: close");
  client.println();

  client.print(head);
  client.write((uint8_t*)img_addr, img_len);
  client.print(tail);
  client.stop();
  return true;
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // 0. Parse incoming MAVLink
  mavlinkInput();

  // 1. Draw NN overlay + send gimbal tracking
  drawOverlay();

  // 2. Grab JPEG frame (with overlay baked in)
  uint32_t img_addr = 0, img_len = 0;
  Camera.getImage(CHANNEL, &img_addr, &img_len);

  // 3. Stream to MJPEG server
  if (img_addr && img_len) {
    if (postFrame(img_addr, img_len)) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(LED_PULSE_MS);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  delay(FRAME_INTERVAL);
}
