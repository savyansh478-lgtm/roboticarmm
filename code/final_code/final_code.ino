// WIFI_AP settings.
const char* AP_SSID = "ESP32_DEV";
const char* AP_PWD  = "12345678";

// WIFI_STA settings.
const char* STA_SSID = "OnePlus 8";
const char* STA_PWD  = "40963840";

// the MAC address of the device you want to ctrl.
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0x93, 0x5F, 0xA8};
// uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


typedef struct struct_message {
  int ID_send;
  int POS_send;
  int Spd_send;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// set the default role here.
// 0 as normal mode.
// 1 as leader, ctrl other device via ESP-NOW.
// 2 as follower, can be controled via ESP-NOW.
#define DEFAULT_ROLE 0

// set the default wifi mode here.
// 1 as [AP] mode, it will not connect other wifi.
// 2 as [STA] mode, it will connect to know wifi.
#define DEFAULT_WIFI_MODE 1

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#define S_SCL 22
#define S_SDA 21

// the GPIO used to control RGB LEDs.
// GPIO 23, as default.
#define RGB_LED   23
#define NUMPIXELS 10

// set the max ID.
int MAX_ID = 20;

// modeSelected.
// set the SERIAL_FORWARDING as true to control the servos with USB.
bool SERIAL_FORWARDING = false;

// OLED Screen Dispaly.
// Row1: MAC address.
// Row2: VCC --- IP address.
// Row3: MODE:Leader/Follower  [AP]/[STA][RSSI]
//       DEFAULT_ROLE: 1-Leader(L)/ 2-Follower(F).
//       DEFAULT_WIFI_MODE: 1-[AP]/ 2-[STA][RSSI] / 3-[TRY:SSID].
//       (no matter what wifi mode you select, you can always ctrl it via ESP-NOW.)
// Row4: the position of servo 1, 2 and 3.
String MAC_ADDRESS;
IPAddress IP_ADDRESS;
uint8_t   SERVO_NUMBER;
uint8_t   DEV_ROLE;
uint8_t   WIFI_MODE;
int    WIFI_RSSI;
int pamphlet = 2;

// set the interval of the threading.
#define threadingInterval 600
#define clientInterval    10

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include "RGB_CTRL.h"
#include "STSCTRL.h"
#include "CONNECT.h"
#include "BOARD_DEV.h"
#include <bits/stdc++.h>
using namespace std;

// Convert degrees ↔ radians
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / M_PI; }

constexpr double H_BASE_MM = 50.0;
constexpr double X_L_MM = 10.5;
constexpr double X_R_MM = -10.5;
constexpr double L1_MM = 50.0;
constexpr double L2_MM = 170.0;
constexpr double L3_MM = 170.0;
constexpr double D_BA_MM = 50.0;
constexpr double L_TIP_MM = 200.0;
constexpr double THETA_LIMIT_RAD = M_PI / 2.0;
constexpr double IK_EPS = 1e-6;

static bool withinJointLimit(double angleRad) {
    return fabs(angleRad) <= (THETA_LIMIT_RAD + 1e-6);
}

static double clampUnit(double x) {
    if (x > 1.0) {
        return 1.0;
    }
    if (x < -1.0) {
        return -1.0;
    }
    return x;
}

static bool solveTrigEquation(double a, double b, double c, vector<double>& roots) {
    roots.clear();
    const double R = hypot(a, b);
    if (R < IK_EPS) {
        return false;
    }

    const double ratio = c / R;
    if (ratio < -1.0 - IK_EPS || ratio > 1.0 + IK_EPS) {
        return false;
    }

    const double phi = atan2(b, a);
    const double gamma = acos(clampUnit(ratio));

    roots.push_back(phi + gamma);
    if (gamma > IK_EPS) {
        roots.push_back(phi - gamma);
    }
    return true;
}

static bool computeAnalyticalIK(double rTarget, double zTarget, double& thetaLeftDeg, double& thetaRightDeg) {
    bool foundSolution = false;
    double bestResidual = 0.0;

    const double outputLen = D_BA_MM + L_TIP_MM;
    vector<double> thetaRightCandidates;
    const double rightA = -2.0 * L3_MM * (rTarget - X_R_MM);
    const double rightB = -2.0 * L3_MM * zTarget;
    const double rightC = outputLen * outputLen - (rTarget - X_R_MM) * (rTarget - X_R_MM)
        - zTarget * zTarget - L3_MM * L3_MM;

    if (!solveTrigEquation(rightA, rightB, rightC, thetaRightCandidates)) {
        return false;
    }

    for (double thetaRight : thetaRightCandidates) {
        if (!withinJointLimit(thetaRight)) {
            continue;
        }

        const double br = X_R_MM + L3_MM * sin(thetaRight);
        const double bz = L3_MM * cos(thetaRight);

        const double dr = rTarget - br;
        const double dz = zTarget - bz;
        const double dNorm = hypot(dr, dz);
        if (dNorm < IK_EPS) {
            continue;
        }

        const double ar = br + D_BA_MM * dr / dNorm;
        const double az = bz + D_BA_MM * dz / dNorm;

        vector<double> thetaLeftCandidates;
        const double leftA = -2.0 * L1_MM * (ar - X_L_MM);
        const double leftB = -2.0 * L1_MM * az;
        const double leftC = L2_MM * L2_MM - (ar - X_L_MM) * (ar - X_L_MM)
            - az * az - L1_MM * L1_MM;

        if (!solveTrigEquation(leftA, leftB, leftC, thetaLeftCandidates)) {
            continue;
        }

        for (double thetaLeft : thetaLeftCandidates) {
            if (!withinJointLimit(thetaLeft)) {
                continue;
            }

            const double p1r = X_L_MM + L1_MM * sin(thetaLeft);
            const double p1z = L1_MM * cos(thetaLeft);

            const double p1aR = ar - p1r;
            const double p1aZ = az - p1z;
            const double abR = br - ar;
            const double abZ = bz - az;
            const double elbowCross = p1aR * abZ - p1aZ * abR;

            if (elbowCross <= 0.0) {
                continue;
            }

            const double leftResidual = fabs((ar - p1r) * (ar - p1r) + (az - p1z) * (az - p1z) - L2_MM * L2_MM);
            const double rightResidual = fabs((rTarget - br) * (rTarget - br) + (zTarget - bz) * (zTarget - bz) - outputLen * outputLen);

            const double residual = leftResidual + rightResidual;
            if (!foundSolution || residual < bestResidual) {
                bestResidual = residual;
                thetaLeftDeg = rad2deg(thetaLeft);
                thetaRightDeg = rad2deg(thetaRight);
                foundSolution = true;
            }
        }
    }

    return foundSolution;
}


uint8_t ID[2];
uint16_t Position[2];
uint16_t Speed[2];


void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  espNowInit();

  getMAC();
  
  boardDevInit();

  RGBcolor(0, 64, 255);

  servoInit();

  wifiInit();

  webServerSetup();

  RGBoff();

  delay(1000);
  pingAll(true);

  threadInit();

}

void smoothMove3Servos(uint8_t servo1ID, uint8_t servo2ID, uint8_t servo3ID, int16_t target1, int16_t target2, int16_t target3, int steps, int stepDelay) {
    
    // Get current positions
    int16_t current1 = posRead[servo1ID];
    int16_t current2 = posRead[servo2ID]; 
    int16_t current3 = posRead[servo3ID];
    
    // Calculate differences
    int16_t diff1 = target1 - current1;
    int16_t diff2 = target2 - current2;
    int16_t diff3 = target3 - current3;
    
    // Calculate step increments
    float step1 = (float)diff1 / steps;
    float step2 = (float)diff2 / steps;
    float step3 = (float)diff3 / steps;
    
    // Arrays for SyncWritePos
    uint8_t servoIDs[3] = {servo1ID, servo2ID, servo3ID};
    uint16_t positions[3];
    uint16_t speed[3];
    speed[0] = 1500;
    speed[1] = 1500;
    speed[2] = 1500; 
    
    // Execute smooth movement
    for(int i = 1; i <= steps; i++) {
        // Calculate intermediate positions
        positions[0] = current1 + (u16)(diff1*i / steps);
        positions[1] = current2 + (u16)(diff2*i / steps);
        positions[2] = current3 + (u16)(diff3*i / steps);
        
        // Send synchronized position command
        st.SyncWritePos(servoIDs, 3, positions, 0, speed);
        
        // Wait before next step
        delay(stepDelay);
        
        // Optional: Update position readings for feedback
        if(i % 5 == 0) { // Update every 5 steps to avoid too frequent reads
            getFeedBack(servo1ID);
            getFeedBack(servo2ID);
            getFeedBack(servo3ID);
        }
    }
    
    // Final position to ensure accuracy
    positions[0] = target1;
    positions[1] = target2;
    positions[2] = target3;
    st.SyncWritePos(servoIDs, 3, positions, 0, speed);
    
    // Update final positions
    delay(100);
    getFeedBack(servo1ID);
    getFeedBack(servo2ID);
    getFeedBack(servo3ID);
}

void adaptiveSmoothMove(uint8_t servo1ID, uint8_t servo2ID, uint8_t servo3ID, int16_t target1, int16_t target2, int16_t target3) {
    
    // Get current positions
    int16_t current1 = posRead[servo1ID];
    int16_t current2 = posRead[servo2ID];
    int16_t current3 = posRead[servo3ID];
    
    // Calculate absolute differences
    int16_t diff1 = abs(target1 - current1);
    int16_t diff2 = abs(target2 - current2);
    int16_t diff3 = abs(target3 - current3);
    
    // Find the largest movement
    int16_t maxDiff = max(diff1, max(diff2, diff3));
    
    // Calculate adaptive steps (more steps for larger movements)
    int adaptiveSteps = 5+ maxDiff/45;    
    // Calculate adaptive delay (slower for larger movements)
    int adaptiveDelay = 20;
    
    // Call smooth move with adaptive parameters
    smoothMove3Servos(servo1ID, servo2ID, servo3ID, target1, target2, target3, adaptiveSteps, adaptiveDelay);
}


void loop() {
  delay(20000);
}


// > > > > > > > > > DOC < < < < < < < < <
// === Develop Board Ctrl ===
// get the MAC address and save it in MAC_ADDRESS;
// getMAC();

// Init GPIO.
// pinMode(PIN_NUM, OUTPUT);
// pinMode(PIN_NUM, INPUT_PULLUP);

// set the level of GPIO.
// digitalWrite(PIN_NUM, LOW);
// digitalWrite(PIN_NUM, HIGH);

// PWM output(GPIO).
// int freq = 50;
// resolution = 12;
// ledcSetup(PWM_NUM, frep, resolution);
// ledcAttachPin(PIN_NUM, PWM_NUM);
// ledcWrite(PWM_NUM, PWM);


// === Servo Ctrl ===
// GPIO 18 as RX, GPIO 19 as TX, init the serial and the servos.
// servoInit();

// set the position as middle pos.
// setMiddle(servoID);
// st.WritePosEx(ID, position, speed, acc);



// === Devices Ctrl ===
// ctrl the RGB.
// 0 < (R, G, B) <= 255
// setSingleLED(LED_ID, matrix.Color(R, G, B));

// init the OLED screen, RGB_LED.
// boardDevInit();

// dispaly the newest information on the screen.
// screenUpdate();
void dance() {
    int theta = -90;
    int r = 250;
    int z = 200;
    double thetaLeft = 0.0;
    double thetaRight = 0.0;
    if (computeAnalyticalIK(r, z - H_BASE_MM, thetaLeft, thetaRight)) {
      int off_1 = 517;
      int off_2 = 766;
      int off_3 = 283;
      int s1 = off_1 - theta*1024/180;
      int s2 = off_2 - (thetaLeft-90)*1024/180;
      int s3 = off_3 - (thetaRight)*1024/180;
      adaptiveSmoothMove(1,2,3, static_cast<int16_t>(s1),static_cast<int16_t>(s2),static_cast<int16_t>(s3));
    }
}
void ik(int cmdI, int cmdA,int cmdB){
    int theta = cmdA;
    int r = cmdI;
    int z = cmdB;
    double thetaLeft = 0.0;
    double thetaRight = 0.0;
    if (!computeAnalyticalIK(r, z - H_BASE_MM, thetaLeft, thetaRight)) {
      Serial.println("Analytical IK failed: target unreachable or violates elbow-up constraint.");
      return;
    }

    int off_1 = 462;
    int off_2 = 771;
    int off_3 = 710;
    int s1 = off_1 - theta*1024/180;
    int s2 = off_2 - (thetaLeft-90)*1024/180;
    int s3 = off_3 + (thetaRight)*1024/180;

    Serial.print("Analytical IK solution: theta_left=");
    Serial.print(thetaLeft);
    Serial.print(" deg theta_right=");
    Serial.println(thetaRight);

    adaptiveSmoothMove(1,2,3, static_cast<int16_t>(s1),static_cast<int16_t>(s2),static_cast<int16_t>(s3));
}
