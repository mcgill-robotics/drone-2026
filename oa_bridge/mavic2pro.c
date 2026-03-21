/*
 * mavic2pro.c  –  Webots controller with Python OA bridge
 * =========================================================
 * Changes vs the original wiki version:
 *   1. Opens a TCP socket to the Python oa_server.py (localhost:65432)
 *   2. Each simulation step:
 *        a. Serialises the LiDAR point cloud + GPS position to JSON
 *        b. Sends to Python  (terminated with '\n')
 *        c. Receives back { vx, vy, vz, yaw_rate } JSON
 *        d. Applies those values to the motor mixer
 *   3. The original keyboard override still works (manual testing)
 *
 * Build note: link with Ws2_32.lib on Windows (already done when using
 * the Webots Makefile on Windows – it adds the flag automatically).
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── platform socket headers ───────────────────────────────────────── */
#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "Ws2_32.lib")
  typedef SOCKET sock_t;
  #define SOCK_INVALID INVALID_SOCKET
  #define sock_close   closesocket
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  typedef int sock_t;
  #define SOCK_INVALID (-1)
  #define sock_close   close
#endif

/* ── Webots headers ─────────────────────────────────────────────────── */
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/lidar.h>

/* ── helpers ────────────────────────────────────────────────────────── */
#define SIGN(x)  (((x) > 0) - ((x) < 0))
#define CLAMP(v,lo,hi)  ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))

/* ── bridge config ──────────────────────────────────────────────────── */
#define OA_HOST        "127.0.0.1"
#define OA_PORT        65432
#define SEND_BUF_SIZE  131072   /* 128 KB – fits ~4000 points */
#define RECV_BUF_SIZE  256

/* ── fixed target (change or extend to a waypoint list) ──────────────── */
#define TARGET_X  5.0
#define TARGET_Y  0.0
#define TARGET_Z  1.5

/* ──────────────────────────────────────────────────────────────────────
 * Socket helpers
 * ────────────────────────────────────────────────────────────────────── */

static sock_t oa_sock = SOCK_INVALID;

static int oa_connect(void) {
#ifdef _WIN32
  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
    fprintf(stderr, "[C] WSAStartup failed\n");
    return -1;
  }
#endif
  oa_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (oa_sock == SOCK_INVALID) {
    fprintf(stderr, "[C] socket() failed\n");
    return -1;
  }

  struct sockaddr_in srv;
  memset(&srv, 0, sizeof(srv));
  srv.sin_family      = AF_INET;
  srv.sin_port        = htons(OA_PORT);
  srv.sin_addr.s_addr = inet_addr(OA_HOST);

  if (connect(oa_sock, (struct sockaddr*)&srv, sizeof(srv)) < 0) {
    fprintf(stderr, "[C] connect() failed – is oa_server.py running?\n");
    sock_close(oa_sock);
    oa_sock = SOCK_INVALID;
    return -1;
  }
  printf("[C] Connected to Python OA server at %s:%d\n", OA_HOST, OA_PORT);
  return 0;
}

/* Send a null-terminated string followed by '\n'. */
static int oa_send(const char *msg) {
  int len = (int)strlen(msg);
  if (send(oa_sock, msg, len, 0) < 0) return -1;
  if (send(oa_sock, "\n",   1, 0) < 0) return -1;
  return 0;
}

/* Receive one '\n'-terminated line into buf (max len bytes).
   Returns number of bytes read (excluding the '\n'), or -1 on error. */
static int oa_recv_line(char *buf, int len) {
  int n = 0;
  char c;
  while (n < len - 1) {
    int r = recv(oa_sock, &c, 1, 0);
    if (r <= 0) return -1;
    if (c == '\n') break;
    buf[n++] = c;
  }
  buf[n] = '\0';
  return n;
}

/* ──────────────────────────────────────────────────────────────────────
 * JSON helpers (minimal – no external library needed)
 * ────────────────────────────────────────────────────────────────────── */

/* Parse a single float value after the key in a flat JSON string.
   E.g.  json_get_float(src, "vx")  finds  "vx": -0.5  → returns -0.5  */
static double json_get_float(const char *src, const char *key) {
  char search[64];
  snprintf(search, sizeof(search), "\"%s\":", key);
  const char *p = strstr(src, search);
  if (!p) return 0.0;
  p += strlen(search);
  while (*p == ' ') p++;
  return atof(p);
}

/* ──────────────────────────────────────────────────────────────────────
 * LiDAR → JSON serialiser
 *
 * Writes a JSON packet to buf (max buf_size bytes):
 * {
 *   "points":[{"x":…,"y":…,"z":…}, …],
 *   "drone_pos":{"x":…,"y":…,"z":…},
 *   "target":{"x":…,"y":…,"z":…}
 * }
 * Returns 0 on success, -1 if buf was too small.
 * ────────────────────────────────────────────────────────────────────── */
static int serialise_lidar(
    const WbLidarPoint *cloud, int total,
    const double *gps,
    double tx, double ty, double tz,
    char *buf, int buf_size)
{
  int pos = 0;

  /* Header */
  pos += snprintf(buf+pos, buf_size-pos,
    "{\"points\":[");
  if (pos >= buf_size) return -1;

  int first = 1;
  for (int i = 0; i < total; i++) {
    /* Skip invalid / infinite points */
    if (!isfinite(cloud[i].x) ||
    !isfinite(cloud[i].y) ||
    !isfinite(cloud[i].z)) continue;

    /* Skip points that are likely ground returns (too close or below drone) */
    float dist = sqrtf(cloud[i].x*cloud[i].x + 
                   cloud[i].y*cloud[i].y + 
                   cloud[i].z*cloud[i].z);

    /* Skip ground returns and very close points */
    if (dist < 1.0f) continue;   /* anything within 1m is likely ground */
    if (dist > 8.0f) continue;   /* ignore very far points too */

    
    int written = snprintf(buf+pos, buf_size-pos,
      "%s{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f}",
      first ? "" : ",",
      (double)cloud[i].x,
      (double)cloud[i].y,
      (double)cloud[i].z);
    if (pos + written >= buf_size) break;   /* truncate gracefully */
    pos  += written;
    first = 0;
  }

  pos += snprintf(buf+pos, buf_size-pos,
    "],\"drone_pos\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f}"
    ",\"target\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}}",
    gps[0], gps[1], gps[2],
    tx, ty, tz);

  return (pos < buf_size) ? 0 : -1;
}

/* ══════════════════════════════════════════════════════════════════════
 * main
 * ══════════════════════════════════════════════════════════════════════ */
int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  /* ── devices ─────────────────────────────────────────────────────── */
  WbDeviceTag imu   = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);

  WbDeviceTag gps   = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);

  WbDeviceTag gyro  = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);

  wb_keyboard_enable(timestep);

  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, timestep);
  wb_lidar_enable_point_cloud(lidar);

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);

  WbDeviceTag camera_roll  = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch = wb_robot_get_device("camera pitch");

  WbDeviceTag motors[4];
  motors[0] = wb_robot_get_device("front left propeller");
  motors[1] = wb_robot_get_device("front right propeller");
  motors[2] = wb_robot_get_device("rear left propeller");
  motors[3] = wb_robot_get_device("rear right propeller");
  for (int m = 0; m < 4; m++) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  /* ── wait for sensors to settle ──────────────────────────────────── */
  while (wb_robot_step(timestep) != -1)
    if (wb_robot_get_time() > 1.0) break;

  /* ── connect to Python OA server ─────────────────────────────────── */
  int bridge_ok = (oa_connect() == 0);
  if (!bridge_ok)
    printf("[C] WARNING: Running without OA bridge (manual keyboard only)\n");

  /* ── flight constants ────────────────────────────────────────────── */
  const double k_vertical_thrust = 68.5;
  const double k_vertical_offset = 0.6;
  const double k_vertical_p      = 3.0;
  const double k_roll_p          = 50.0;
  const double k_pitch_p         = 30.0;
  double target_altitude          = 1.0;

  /* OA-derived disturbances (updated each step from Python) */
  double oa_pitch = 0.0;   /* maps to forward/back */
  double oa_roll  = 0.0;   /* maps to left/right   */
  double oa_yaw   = 0.0;
  double oa_vz    = 0.0;   /* vertical velocity request */

  /* Buffers */
  static char send_buf[SEND_BUF_SIZE];
  static char recv_buf[RECV_BUF_SIZE];

  printf("[C] Mavic 2 Pro: Obstacle Avoidance Bridge Active\n");

  /* ══ main loop ═════════════════════════════════════════════════════ */
  while (wb_robot_step(timestep) != -1) {

    /* ── 1. Read sensors ─────────────────────────────────────────── */
    const double *rpy      = wb_inertial_unit_get_roll_pitch_yaw(imu);
    const double  roll     = rpy[0];
    const double  pitch    = rpy[1];
    const double  altitude = wb_gps_get_values(gps)[2];
    const double *gps_pos  = wb_gps_get_values(gps);
    const double  roll_vel = wb_gyro_get_values(gyro)[0];
    const double  pitch_vel= wb_gyro_get_values(gyro)[1];

    /* ── 2. LiDAR ────────────────────────────────────────────────── */
    const WbLidarPoint *cloud = wb_lidar_get_point_cloud(lidar);
    int width  = wb_lidar_get_horizontal_resolution(lidar);
    int height = wb_lidar_get_number_of_layers(lidar);
    int total  = width * height;

    /* ── 3. Send to Python / receive movement command ────────────── */
    if (bridge_ok && cloud) {
      int err = serialise_lidar(
          cloud, total, gps_pos,
          TARGET_X, TARGET_Y, TARGET_Z,
          send_buf, SEND_BUF_SIZE);

      if (err == 0 && oa_send(send_buf) == 0) {
        int n = oa_recv_line(recv_buf, RECV_BUF_SIZE);
        if (n > 0) {
          /* Parse  {"vx":…,"vy":…,"vz":…,"yaw_rate":…} */
          double vx  = json_get_float(recv_buf, "vx");
          double vy  = json_get_float(recv_buf, "vy");
          double vz  = json_get_float(recv_buf, "vz");
          double yaw = json_get_float(recv_buf, "yaw_rate");

          /*
           * Map Python world-frame velocities to Webots body-frame
           * disturbances that the existing PID mixer understands.
           *
           * Convention (Webots Mavic 2 Pro):
           *   pitch_disturbance < 0  → nose down  → forward
           *   roll_disturbance  > 0  → right tilt → right
           *
           * Scale factors are tuning knobs – start small.
           */
          oa_pitch = -vx * 0.5;
          oa_roll  =  vy * 0.5;
          oa_yaw   =  yaw;
          oa_vz    =  vz;            /* fed into altitude target below */

          /* Adjust altitude target from vertical velocity request */
          target_altitude += oa_vz * (timestep / 1000.0);
          target_altitude  = CLAMP(target_altitude, 0.3, 20.0);
        }
      }
    }

    /* ── 4. Keyboard override (manual testing) ───────────────────── */
    double roll_disturbance  = oa_roll;
    double pitch_disturbance = oa_pitch;
    double yaw_disturbance   = oa_yaw;

    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case 'Q':
          wb_robot_cleanup();
          if (oa_sock != SOCK_INVALID) sock_close(oa_sock);
          exit(0);
        case WB_KEYBOARD_UP:
          pitch_disturbance = -2.0; break;
        case WB_KEYBOARD_DOWN:
          pitch_disturbance =  2.0; break;
        case WB_KEYBOARD_RIGHT:
          yaw_disturbance   = -1.3; break;
        case WB_KEYBOARD_LEFT:
          yaw_disturbance   =  1.3; break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
          target_altitude  += 0.05; break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
          target_altitude  -= 0.05; break;
      }
      key = wb_keyboard_get_key();
    }

    /* ── 5. PID stabiliser + motor mixing ────────────────────────── */
    const double roll_input  = k_roll_p  * CLAMP(roll,  -1.0, 1.0)
                               + roll_vel  + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0)
                               + pitch_vel + pitch_disturbance;
    const double yaw_input   = yaw_disturbance;

    const double clamped_alt = CLAMP(
        target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vert_input  = k_vertical_p * pow(clamped_alt, 3.0);

    const double m[4] = {
       k_vertical_thrust + vert_input - roll_input + pitch_input - yaw_input,
      -(k_vertical_thrust + vert_input + roll_input + pitch_input + yaw_input),
      -(k_vertical_thrust + vert_input - roll_input - pitch_input + yaw_input),
       k_vertical_thrust + vert_input + roll_input - pitch_input - yaw_input
    };

    for (int i = 0; i < 4; i++)
      wb_motor_set_velocity(motors[i], m[i]);

    /* Camera stabilisation */
    wb_motor_set_position(camera_roll,  -0.115 * roll_vel);
    wb_motor_set_position(camera_pitch, -0.1   * pitch_vel);
  }

  if (oa_sock != SOCK_INVALID) sock_close(oa_sock);
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
