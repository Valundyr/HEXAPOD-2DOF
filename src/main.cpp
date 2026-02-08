#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h> 
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "HEXAPOD_ROBOT"; // Nome da rede WiFi do Pico W
const char* password = "12345678";  // Senha

WebServer server(80);

// --- HTML ATUALIZADO: Painel de Controlo Responsivo ---
const char index_html[] = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HEXAPOD CONTROL</title>
  <style>
    body { background-color: #111; color: rgb(255, 187, 0); font-family: monospace; text-align: center; margin: 0; padding: 5px; }
    h3 { margin: 10px 0; border-bottom: 1px solid #333; padding-bottom: 5px; }
    
    /* Grelha de Sensores */
    .grid { display: flex; flex-wrap: wrap; justify-content: center; gap: 5px; margin-bottom: 15px; }
    .card { background: #222; border: 1px solid #444; padding: 5px; width: 90px; border-radius: 4px; }
    .lbl { font-size: 10px; color: #aaa; }
    .val { font-size: 14px; font-weight: bold; }
    .warn { color: red; }

    /* Painel de Controlo */
    .ctrl-panel { max-width: 400px; margin: 0 auto; background: #1a1a1a; padding: 10px; border-radius: 8px; border: 1px solid #333; }
    .btn-group { display: flex; gap: 5px; justify-content: center; margin-bottom: 10px; }
    
    button {
      flex: 1; padding: 12px; font-family: monospace; font-weight: bold; cursor: pointer;
      border: none; border-radius: 4px; color: white; transition: 0.2s;
    }
    
    .btn-gait { background-color: #e7b923; color: #111; } 
    .btn-spd { background-color: #3c46ca; }  
    .btn-act { background-color: #ee2805; font-size: 18px; } 
    .btn-start { background-color: #09d317; font-size: 16px; color: #fff;} 
    .btn-calib { background-color: #fff; color: #000; font-size: 16px; }

    button:active { opacity: 0.7; transform: scale(0.98); }
    .active-sel { border: 2px solid rgb(255, 38, 0); filter: brightness(1.3); }

    hr { border-color: #333; margin: 15px 0; }
    #status-txt { color: White; font-weight: bold; margin-bottom: 10px; display: block;}
  </style>
</head>
<body>
  <h3>HEXAPOD COMMAND CENTER</h3>
  
  <div class="grid">
    <div class="card"><span class="lbl">YAW</span><br><span class="val" id="y">0.0</span></div>
    <div class="card"><span class="lbl">PITCH</span><br><span class="val" id="p">0.0</span></div>
    <div class="card"><span class="lbl">ROLL</span><br><span class="val" id="r">0.0</span></div>
    <div class="card"><span class="lbl">DIST</span><br><span class="val" id="d">0</span></div>
    <div class="card"><span class="lbl">BATIDA</span><br><span class="val" id="i">OK</span></div>
  </div>

  <div class="ctrl-panel">
    <span id="status-txt">AGUARDAR...</span>
    
    <div class="btn-group">
        <button class="btn-gait" onclick="cmd('g1', this)">TRIPOD</button>
        <button class="btn-gait" onclick="cmd('g2', this)">CENTOP.</button>
    </div>
    <div class="btn-group">
        <button class="btn-gait" onclick="cmd('g3_left', this)">&lt;&lt; LAT. ESQ</button>
        <button class="btn-gait" onclick="cmd('g3_right', this)">LAT. DIR &gt;&gt;</button>
    </div>

    <div class="btn-group">
        <button class="btn-spd" onclick="cmd('s1', this)">LENTO</button>
        <button class="btn-spd" onclick="cmd('s2', this)">MEDIO</button>
        <button class="btn-spd" onclick="cmd('s3', this)">RAPIDO</button>
    </div>

    <hr>
    
    <div class="btn-group">
        <button class="btn-spd" style="background:#555;" onclick="cmd('rot-90', this)">-90&deg;</button>
        <button class="btn-spd" style="background:#555;" onclick="cmd('rot0', this)">0&deg; (HOME)</button>
        <button class="btn-spd" style="background:#555;" onclick="cmd('rot90', this)">+90&deg;</button>
    </div>

    <hr>
    
    <div class="btn-group">
        <button class="btn-calib" onclick="cmd('calib', this)">1. CALIBRAR</button>
        <button class="btn-start" onclick="cmd('walk', this)">2. MARCHAR</button>
    </div>
    
    <div class="btn-group">
        <button class="btn-act" onclick="cmd('stop', this)">PARAR TUDO</button>
    </div>
  </div>

  <script>
    function cmd(val, btn) {
        // Feedback visual
        if(val.startsWith('g')) document.querySelectorAll('.btn-gait').forEach(b => b.classList.remove('active-sel'));
        if(val.startsWith('s')) document.querySelectorAll('.btn-spd').forEach(b => b.classList.remove('active-sel'));
        if(btn && !val.startsWith('rot')) btn.classList.add('active-sel'); // Botões de rotação não fixam cor

        fetch('/cmd?c=' + val).then(r => console.log('Cmd sent: ' + val));
    }

    setInterval(() => {
      fetch('/data').then(r => r.json()).then(d => {
        document.getElementById('y').innerText = parseFloat(d.y).toFixed(1);
        document.getElementById('p').innerText = parseFloat(d.p).toFixed(1);
        document.getElementById('r').innerText = parseFloat(d.r).toFixed(1);
        document.getElementById('d').innerText = d.d;
        
        let imp = document.getElementById('i');
        imp.innerText = d.i ? "CRASH!" : "OK";
        imp.className = d.i ? "val warn" : "val";
        
        // Traduz estados
        let states = [
            "MENU GAIT",        // 1
            "MENU VEL",         // 2
            "A CALIBRAR",       // 3
            "PRONTO (STANDBY)", // 4
            "A MARCHAR",        // 5
            "CHOQUE",           // 6
            "PAUSA",            // 7
            "A VIRAR",          // 8
            "PAUSA",            // 9
            "A RODAR (PRECISO)",// 10
            "ERRO HAR",         // 11
            "ERRO: CALIBRAR!",  // 12 
            "ERRO: MOV!"        // 13 
        ];
        let stText = states[d.s] || "UNK";
        document.getElementById('status-txt').innerText = "STATUS: " + stText;
      });
    }, 500); 
  </script>
</body>
</html>
)rawliteral";

// =============================================================
// 0. UTILITY: TIMER 
// =============================================================
class Timer {
  private: 
    unsigned long lastTime;
  
  public:
    Timer() { 
      reset(); 
    }
    
    void reset() { 
      lastTime = millis(); 
    }
    
    bool hasExpired(unsigned long ms) {
      if (millis() - lastTime >= ms) {
        lastTime = millis();
        return true;
      }
      return false;
    }
};

// =============================================================
// 1. CONFIGURATION
// =============================================================
// --- Pinos I2C ---
const uint8_t PIN_SDA = 8;
const uint8_t PIN_SCL = 9;

const uint8_t PIN_LIDAR_SDA = 6; 
const uint8_t PIN_LIDAR_SCL = 7;
const int     LIDAR_OFFSET  = 30; 

// --- Configuração de Impacto (4 NÍVEIS) ---
const int THRESHOLD_SENSITIVE      = 9000;  // Tripé Lento/Médio
const int THRESHOLD_MEDIUM         = 12000; // Tripé Rápido
const int THRESHOLD_CENTIPEDE_SLOW = 21000; // Centopeia Lenta/Média
const int THRESHOLD_HARD           = 24000; // Centopeia Rápida e Lateral

const int IMPACT_CONFIRMATIONS = 3; 

// --- Velocidades ---
const float SPEED_SLOW = 0.020;
const float SPEED_MED  = 0.030;
const float SPEED_FAST = 0.040;

// --- Definições de Movimento ---
const int   UPDATE_RATE_MS   = 20;    
const float HEADING_P_GAIN   = 3.0;   
const float DRIFT_CORRECTION = 4.0; 

// --- Obstáculos (TEMPOS ADAPTATIVOS) ---
const int   MIN_DISTANCE_MM     = 130;  
const int   CLEAR_DISTANCE_MM   = 250;  
const float TURN_INTENSITY      = 40.0; 

// Tempos de viragem diferentes por andar
const unsigned long TURN_TIME_TRIPOD    = 2000; 
const unsigned long TURN_TIME_CENTIPEDE = 4000; 
const unsigned long TURN_TIME_SIDEWAYS  = 1500;

const unsigned long STABILIZE_TIME      = 200; 

// --- Geometria ---
const float STEP_HEIGHT     = 100.0;
const float STRIDE_LENGTH   = 50.0;  
const int   FLOOR_LEVEL     = 150;   
const unsigned long WARMUP_TIME_MS = 1500; 

// --- Estados do Sistema ---
enum SystemState {
  STATE_MENU_GAIT,       // 1. Menu Inicial
  STATE_MENU_SPEED,      // 2. Menu Velocidade
  STATE_CALIBRATING,     // 3. Calibração
  STATE_READY_TO_START,  // 4. Standby
  STATE_WALKING,         // 5. Marcha
  STATE_IMPACT_REACTION, // 6. Reação a Choque
  STATE_WAIT_TO_TURN,    // 7. Pausa -> Virar
  STATE_AVOIDING,        // 8. Virar
  STATE_WAIT_TO_WALK,    // 9. Pausa -> Marcha
  STATE_PRECISE_ROTATION,// 10. rodar
  STATE_ERROR,           // 11. Erro: sistema
  STATE_ERR_CALIB,       // 12. Erro: Falta Calibrar
  STATE_ERR_NO_GAIT      // 13. Erro: Falta escolher Gait
};

enum GaitType {
  GAIT_NONE,
  GAIT_TRIPOD,    
  GAIT_CENTIPEDE,
  GAIT_SIDEWAYS 
};

// --- Variáveis Globais ---
SystemState currentState = STATE_MENU_GAIT; 
GaitType currentGait = GAIT_NONE; 
float currentSpeed = SPEED_MED; 
int avoidDirection = 1; 
int sidewaysDirection = 1; // 1 = Direita, -1 = Esquerda
float targetHeading = 0.0;
// =============================================================
// 2. CLASSE IMU
// =============================================================
class IMU {
  private:
    float gyroOffsetZ;
    float accOffsetX, accOffsetY; // Zeros
    float currentYaw, currentPitch, currentRoll; // Angulos
    unsigned long prevTime;
    long calibSumZ, calibSumX, calibSumY;
    int sampleCount;
    const int REQUIRED_SAMPLES = 500; 
    bool isCalibrated;
    
    float filterX, filterY, filterZ;
    int shockCounter; 
    bool impactDetected;
    unsigned long walkStartTime;

    void writeRegister(uint8_t reg, uint8_t val) {
      Wire.beginTransmission(0x68); 
      Wire.write(reg); 
      Wire.write(val); 
      Wire.endTransmission();
    }

  public:
    IMU() : gyroOffsetZ(0), accOffsetX(0), accOffsetY(0), currentYaw(0), currentPitch(0), currentRoll(0), 
            prevTime(0), sampleCount(0), isCalibrated(false), filterX(0), filterY(0), filterZ(0), 
            shockCounter(0), impactDetected(false), walkStartTime(0) {}

    // --- GETTERS PÚBLICOS PARA O WEBSITE ---
    float getAccelX() { return filterX; }
    float getAccelY() { return filterY; }
    float getYaw() { return currentYaw; }
    float getPitch() { return currentPitch; }
    float getRoll()  { return currentRoll; }
    bool  isImpacted() { return impactDetected; }
    bool  isReady() { return isCalibrated; } 
    // ---------------------------------------

    bool init() {
      Wire.beginTransmission(0x68);
      if (Wire.endTransmission() != 0) return false;
      writeRegister(0x6B, 0x00); 
      prevTime = millis();
      return true;
    }

    void forceRecalibration() {
      isCalibrated = false; sampleCount = 0;
      calibSumZ = 0; calibSumX = 0; calibSumY = 0;
      currentYaw = 0; currentPitch = 0; currentRoll = 0;
      filterX = 0; filterY = 0; filterZ = 0; shockCounter = 0;
    }
    
    void notifyWalkStart() {
      walkStartTime = millis(); 
      impactDetected = false; 
      shockCounter = 0;
    }

    bool performCalibrationStep() {
      if (isCalibrated) return true;
      
      // Ler Accel para tarar o zero
      Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
      Wire.requestFrom(0x68, 6, true); 
      int16_t rawAx = (Wire.read() << 8 | Wire.read());
      int16_t rawAy = (Wire.read() << 8 | Wire.read());
      Wire.read(); Wire.read(); // Lemos Az mas nao usamos aqui
      
      calibSumX += rawAx;
      calibSumY += rawAy;

      // Ler Gyro Z
      Wire.beginTransmission(0x68); Wire.write(0x47); Wire.endTransmission(false);
      Wire.requestFrom(0x68, 2, true);
      int16_t rawGz = (Wire.read() << 8 | Wire.read());
      calibSumZ += rawGz;

      sampleCount++;
      
      if (sampleCount >= REQUIRED_SAMPLES) {
        gyroOffsetZ = (float)calibSumZ / REQUIRED_SAMPLES;
        accOffsetX  = (float)calibSumX / REQUIRED_SAMPLES;
        accOffsetY  = (float)calibSumY / REQUIRED_SAMPLES;
        isCalibrated = true;
        return true;
      }
      return false;
    }

    void readAccel(int dynamicThreshold) {
      Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
      Wire.requestFrom(0x68, 6, true); 
      
      int16_t rawAx = (Wire.read() << 8 | Wire.read());
      int16_t rawAy = (Wire.read() << 8 | Wire.read());
      int16_t rawAz = (Wire.read() << 8 | Wire.read());

      // 1. Aplicar Tara (Zero)
      float ax_cal = (float)rawAx - accOffsetX; 
      float ay_cal = (float)rawAy - accOffsetY;
      float az_cal = (float)rawAz; 

      // 2. Filtro Passa-Baixo
      filterX = (0.90 * filterX) + (0.10 * ax_cal);
      filterY = (0.90 * filterY) + (0.10 * ay_cal);
      filterZ = (0.90 * filterZ) + (0.10 * az_cal);

      // 3. Calculo Pitch/Roll
      currentRoll  = atan2(filterY, filterZ) * 180.0 / PI;
      currentPitch = atan2(-filterX, sqrt(filterY*filterY + filterZ*filterZ)) * 180.0 / PI;

      // 4. Detetar Impacto
      long shockX = abs(ax_cal - filterX);
      long shockY = abs(ay_cal - filterY);
      
      if (isCalibrated && (millis() - walkStartTime > 1000)) {
        if (shockX > dynamicThreshold || shockY > dynamicThreshold) {
          shockCounter++; 
        } else {
          shockCounter = 0; 
        }
        
        if (shockCounter >= IMPACT_CONFIRMATIONS) {
          impactDetected = true; 
          shockCounter = 0; 
        } else {
          impactDetected = false;
        }
      } else {
        impactDetected = false; shockCounter = 0;
      }
    }

    void update(int dynamicThreshold) {
      if (!isCalibrated) return;
      readAccel(dynamicThreshold);
      unsigned long now = millis();
      float dt = (now - prevTime) / 1000.0;
      prevTime = now;
      
      Wire.beginTransmission(0x68); Wire.write(0x47); Wire.endTransmission(false);
      Wire.requestFrom(0x68, 2, true);
      int16_t rawZ = (Wire.read() << 8 | Wire.read());
      
      float gyroZ = ((float)rawZ - gyroOffsetZ) / 131.0;
      if (abs(gyroZ) > 0.1) {
        currentYaw += gyroZ * dt;
      }
    }

    bool hasCrashed() { return impactDetected; }
    void resetYaw() { currentYaw = 0.0; }
};

// =============================================================
// 3. CLASS: LIDAR SENSOR
// =============================================================
class LidarSensor {
  private:
    VL53L0X sensor;
    int currentDist;
    int rawDist;
    bool active;

  public:
    LidarSensor() : currentDist(8888), rawDist(8888), active(false) {}

    bool init() {
      Wire1.setSDA(PIN_LIDAR_SDA);
      Wire1.setSCL(PIN_LIDAR_SCL);
      Wire1.begin();
      
      sensor.setBus(&Wire1); 
      sensor.setTimeout(500);
      
      if (!sensor.init()) { 
        active = false; 
        return false; 
      }
      
      sensor.setMeasurementTimingBudget(20000); 
      sensor.startContinuous();
      active = true;
      return true;
    }

    void update() {
      if (!active) {
        return;
      }
      
      int leitura = sensor.readRangeContinuousMillimeters();
      
      if (sensor.timeoutOccurred() || leitura > 8000) {
        return; 
      }
      if (leitura == 0) {
        return; 
      }
      
      rawDist = leitura; 
      int corrected = leitura - LIDAR_OFFSET;
      
      if (corrected < 0) {
        corrected = 0;
      }
      currentDist = corrected;
    }

    int getDistance() { return currentDist; }
    int getRaw() { return rawDist; }
    bool isActive() { return active; }
};

// =============================================================
// 4. CLASS: HEXAPOD
// =============================================================
class Hexapod {
  private:
    Adafruit_PWMServoDriver pwm;
    int servoOffsets[16]; 
    float gaitPhase;
    Timer moveTimer; 

    void setServo(int id, float value) {
      if (value < 100) {
        value = 100;
      }
      if (value > 500) {
        value = 500;
      }
      pwm.setPWM(id, 0, (int)value);
    }

    float getTriangleWave(float t) {
      if (t < 0.5) {
        return 2.0 * t;
      }
      return 2.0 - (2.0 * t);
    }

    /*float getSineArch(float t) { 
      return sin(t * PI); 
    }*/
    
    // Onda "Quadrada Suave" para levantar patas rápido
    float getSquareLift(float t) {
      if (t < 0.1) {
        return t * 10.0; // Sobe rapido
      }
      if (t > 0.9) {
        return (1.0 - t) * 10.0; // Desce rapido
      }
      return 1.0; // Mantém em cima
    }

  public:
    Hexapod() : pwm(Adafruit_PWMServoDriver()), gaitPhase(0.0) {}

    bool init() {
      // Verifica PCA
      Wire.beginTransmission(0x40);
      if (Wire.endTransmission() != 0) {
        return false; 
      }

      pwm.begin();
      pwm.setOscillatorFrequency(27000000);
      pwm.setPWMFreq(50);
      
      servoOffsets[5] = 160; 
      servoOffsets[3] = 180; 
      servoOffsets[1] = 290;  
      
      servoOffsets[11]= 300; 
      servoOffsets[9] = 230; 
      servoOffsets[7] = 140;  
      
      stop();
      return true;
    }

    void stop() {
      int verticals[] = {0, 2, 4, 6, 8, 10};
      for(int id : verticals) {
        pwm.setPWM(id, 0, FLOOR_LEVEL);
      }
      
      int horizontals[] = {1, 3, 5, 7, 9, 11};
      for(int id : horizontals) {
        setServo(id, servoOffsets[id]);
      }
    }

    void updateWalking(float headingCorrection, float strideLen, GaitType type, float speedVal, int sideDir) {
      if (!moveTimer.hasExpired(UPDATE_RATE_MS)) {
        return; 
      }

      // --- LÓGICA DE DISTRIBUIÇÃO DA CORREÇÃO ---
      
      if (type == GAIT_TRIPOD) {
        gaitPhase += speedVal;
        if (gaitPhase >= 1.0) gaitPhase -= 1.0;
        
        float totalCorrection = headingCorrection + DRIFT_CORRECTION;
        processTripod(gaitPhase, totalCorrection, strideLen);
      } 
      else if (type == GAIT_CENTIPEDE) {
        gaitPhase += (speedVal * 0.85); 
        if (gaitPhase >= 1.0) gaitPhase -= 1.0;
        
        float totalCorrection = headingCorrection + DRIFT_CORRECTION;
        processCentipede(gaitPhase, totalCorrection * 0.5, strideLen * 0.8);
      }
      else if (type == GAIT_SIDEWAYS) {
        gaitPhase += (speedVal * 0.8); 
        if (gaitPhase >= 1.0) gaitPhase -= 1.0;
        
        // --- LIMITADOR DE TAMANHO DE PASSO ---
        float safeStride = strideLen;

        // Médio: 85% do passo
        if (speedVal >= 0.020 && speedVal < 0.030) {
           safeStride = strideLen * 0.85; 
        }
        // Rápido: 65% do passo
        else if (speedVal >= 0.030) {
           safeStride = strideLen * 0.65; 
        }

        processSideways(gaitPhase, safeStride, sideDir, headingCorrection, speedVal);
      }
    }

    void processTripod(float phase, float correction, float strideLen) {
      float phaseA = phase;
      float phaseB = phase + 0.5;
      if (phaseB >= 1.0) {
        phaseB -= 1.0;
      }
      
      moveLegTripod(5, phaseA, true, correction, strideLen);  
      moveLegTripod(1, phaseA, true, correction, strideLen);
      moveLegTripod(9, phaseA, false, correction, strideLen); 
      
      moveLegTripod(11, phaseB, false, correction, strideLen); 
      moveLegTripod(7,  phaseB, false, correction, strideLen); 
      moveLegTripod(3,  phaseB, true, correction, strideLen);
    }

    void moveLegTripod(int id, float phase, bool isLeft, float correction, float strideLen) {
      float lift = 0; 
      float forward = 0;
      float amp = (isLeft) ? strideLen + correction : strideLen - correction;
      float sign = (isLeft) ? -1.0 : 1.0; 
      
      if (phase < 0.5) { 
        float t = phase * 2.0;
        lift = getTriangleWave(t) * STEP_HEIGHT; 
        forward = -1.0 + (t * 2.0);              
      } 
      else { 
        float t = (phase - 0.5) * 2.0;
        lift = 0; 
        forward = 1.0 - (t * 2.0);              
      }
      
      setServo(id, servoOffsets[id] + (sign * forward * amp)); 
      setServo(id-1, FLOOR_LEVEL - lift);
    }

    void processCentipede(float mainPhase, float correction, float strideLen) {
      float ampLeft = strideLen + correction;
      float ampRight = strideLen - correction;
      
      float phBack = mainPhase; 
      float phMid = mainPhase - 0.333; 
      if (phMid < 0) {
        phMid += 1.0;
      }
      
      float phFront = mainPhase - 0.666; 
      if (phFront < 0) {
        phFront += 1.0;
      }
      
      moveLegCentipede(1, 7,  phBack,  ampLeft, ampRight); 
      moveLegCentipede(3, 9,  phMid,   ampLeft, ampRight); 
      moveLegCentipede(5, 11, phFront, ampLeft, ampRight); 
    }

    void moveLegCentipede(int leftID, int rightID, float localPhase, float ampL, float ampR) {
        // --- LÓGICA PARA A PATA ESQUERDA ---
        float liftL = 0; 
        float forwardL = 0;
        
        // Calcula movimento Esquerdo normal
        if (localPhase < 0.333) { 
            float t = localPhase * 3.0; 
            liftL = getTriangleWave(t) * (STEP_HEIGHT * 1.3); 
            forwardL = -1.0 + (t * 2.0); 
        } else { 
            float t = (localPhase - 0.333) * 1.5; 
            liftL = 0;
            forwardL = 1.0 - (t * 2.0); 
        }
        
        // --- LÓGICA PARA A PATA DIREITA (COM DESFASAMENTO) ---
        // Adicionamos 0.5 à fase para inverter o ciclo na direita
        float phaseR = localPhase + 0.5; 
        if (phaseR >= 1.0) phaseR -= 1.0; // Garante que fica entre 0.0 e 1.0

        float liftR = 0;
        float forwardR = 0;

        if (phaseR < 0.333) { 
            float t = phaseR * 3.0; 
            liftR = getTriangleWave(t) * (STEP_HEIGHT * 1.3); 
            forwardR = -1.0 + (t * 2.0); 
        } else { 
            float t = (phaseR - 0.333) * 1.5; 
            liftR = 0;
            forwardR = 1.0 - (t * 2.0); 
        }

        // --- APLICAR AOS SERVOS ---
        setServo(leftID,   servoOffsets[leftID]  - (forwardL * ampL)); 
        setServo(leftID-1, FLOOR_LEVEL - liftL);                      
        
        setServo(rightID,   servoOffsets[rightID] + (forwardR * ampR)); 
        setServo(rightID-1, FLOOR_LEVEL - liftR);                      
    }

    void processSideways(float phase, float strideLen, int sideDir, float driftFix, float speedVal) {
      // Usamos a mesma lógica de fases do Tripé para estabilidade
      float phaseA = phase;
      float phaseB = phase + 0.5;
      if (phaseB >= 1.0) phaseB -= 1.0;
      
      // Ajustamos a amplitude do "empurrão" lateral
      // sideDir: 1 = Direita, -1 = Esquerda
      
      // Grupo A (Tripé 1)
      moveLegSideways(5, phaseA, strideLen, sideDir, driftFix, true);  // Esq
      moveLegSideways(1, phaseA, strideLen, sideDir, driftFix, true);  // Esq
      moveLegSideways(9, phaseA, strideLen, sideDir, driftFix, false); // Dir
      
      // Grupo B (Tripé 2)
      moveLegSideways(11, phaseB, strideLen, sideDir, driftFix, false); // Dir
      moveLegSideways(7,  phaseB, strideLen, sideDir, driftFix, false); // Dir
      moveLegSideways(3,  phaseB, strideLen, sideDir, driftFix, true);  // Esq
    }

    void moveLegSideways(int id, float phase, float amp, int sideDir, float correction, bool isLeftLeg) {
      float lift = 0;
      float push = 0; // O movimento de abrir/fechar
      
      // --- 1. CÁLCULO DA ONDA (IGUAL AO TRIPÉ) ---
      if (phase < 0.5) { 
        // FASE AÉREA (A repor a pata)
        float t = phase * 2.0;
        lift = getTriangleWave(t) * STEP_HEIGHT; 
        
        // No ar, a pata move-se no sentido CONTRÁRIO ao movimento do corpo
        // Se quero ir para a Direita (sideDir=1), a pata no ar vai para a Direita.
        push = -1.0 + (t * 2.0); 
      } 
      else { 
        // FASE TERRESTRE (A empurrar o chão)
        float t = (phase - 0.5) * 2.0;
        lift = 0; 
        
        // No chão, a pata empurra o corpo.
        // Se quero ir para a Direita, a pata no chão empurra para a Esquerda.
        push = 1.0 - (t * 2.0); 
      }
      
      // --- 2. LÓGICA DE EMPURRÃO ---
      // Se a perna é ESQUERDA e quero ir para DIREITA: Ela tem de EMPURRAR (Esticar/Abrir)
      // Se a perna é DIREITA e quero ir para DIREITA: Ela tem de PUXAR (Contrair/Fechar)
      
      float actionDir = 0;
      
      if (sideDir == 1) { // Mover para a Direita
         if (isLeftLeg) actionDir = 1.0;  // Esq estica (Empurra corpo para lá >>>)
         else           actionDir = -1.0; // Dir contrai (Puxa corpo para lá >>>)
      } 
      else { // Mover para a Esquerda
         if (isLeftLeg) actionDir = -1.0; // Esq contrai (Puxa corpo para lá <<<)
         else           actionDir = 1.0;  // Dir estica (Empurra corpo para lá <<<)
      }

      // Aplicamos o movimento de "Push" calculado
      float verticalMove = push * amp * actionDir;

      // --- 3. APLICAÇÃO NOS SERVOS ---
      
      // SERVO HORIZONTAL (ID):
      // Fica PARADO no centro, mexe APENAS se o MPU detetar rotação indesejada.
      float rotCalc = (isLeftLeg) ? correction : -correction;
      setServo(id, servoOffsets[id] + rotCalc); 

      // SERVO VERTICAL (ID-1):
      // Faz o Lift (subir pata) + o VerticalMove (Empurrar chão)
      setServo(id-1, FLOOR_LEVEL - lift + verticalMove);
    }
};

// =============================================================
// 5. VARIÁVEIS GLOBAIS E OBJETOS
// =============================================================
Hexapod robot;
IMU imu; 
LidarSensor lidar;
Timer debugTimer; 

unsigned long bootTimestamp = 0;
unsigned long waitTimestamp = 0; 
unsigned long clearPathTimestamp = 0; 
unsigned long impactTimestamp = 0; 

// =============================================================
// 6. FUNÇÕES DO SERVIDOR WEB
// =============================================================

// Envia a página HTML principal
void handleRoot() {
  server.send(200, "text/html", index_html);
}

// Envia os dados dos sensores em formato JSON para o JavaScript
void handleData() {
  String json = "{";
  json += "\"y\":" + String(imu.getYaw()) + ",";
  json += "\"p\":" + String(imu.getPitch()) + ","; 
  json += "\"r\":" + String(imu.getRoll()) + ","; 
  json += "\"d\":" + String(lidar.getDistance()) + ",";
  json += "\"ax\":" + String(imu.getAccelX()) + ","; 
  json += "\"ay\":" + String(imu.getAccelY()) + ","; 
  json += "\"i\":" + String(imu.isImpacted()) + ",";
  json += "\"s\":" + String(currentState); 
  json += "}";
  server.send(200, "application/json", json);
}

// Recebe comandos dos botões da página Web
void handleCommand() {
  if (!server.hasArg("c")) { server.send(400, "text/plain", "Bad Request"); return; }
  String cmd = server.arg("c");
  
  // --- DEFINIÇÕES DE GAIT/SPEED (Mantêm-se iguais) ---
  // --- CÓDIGO NOVO ---
  if (cmd == "g1") {
      currentGait = GAIT_TRIPOD;
      Serial.println("CMD: TRIPOD");
  }
  else if (cmd == "g2") {
      currentGait = GAIT_CENTIPEDE;
      Serial.println("CMD: CENTOPEIA");
  }
  // --- NOVOS COMANDOS LATERAIS ---
  else if (cmd == "g3_left") { 
      currentGait = GAIT_SIDEWAYS; 
      sidewaysDirection = 1; // Define direção ESQUERDA
      Serial.println("CMD: LATERAL ESQUERDA");
  }
  else if (cmd == "g3_right") { 
      currentGait = GAIT_SIDEWAYS; 
      sidewaysDirection = -1;  // Define direção DIREITA
      Serial.println("CMD: LATERAL DIREITA");
  }
  else if (cmd == "s1") currentSpeed = SPEED_SLOW;
  else if (cmd == "s2") currentSpeed = SPEED_MED;
  else if (cmd == "s3") currentSpeed = SPEED_FAST;
  
  // --- COMANDOS DE ROTAÇÃO ABSOLUTA ---
  else if (cmd == "rot90") {
      targetHeading = 90.0; 
      currentState = STATE_PRECISE_ROTATION; // Vai direto para a rotação
      Serial.println("CMD: RODA PARA 90");
  }
  else if (cmd == "rot-90") {
      targetHeading = -90.0; 
      currentState = STATE_PRECISE_ROTATION;
      Serial.println("CMD: RODA PARA -90");
  }
  else if (cmd == "rot0") {
      targetHeading = 0.0; 
      currentState = STATE_PRECISE_ROTATION;
      Serial.println("CMD: VOLTA PARA 0 (CENTRO)");
  }

  // --- COMANDOS DE AÇÃO ---
  else if (cmd == "calib") {
      // 1. Só calibra. Não anda.
      imu.forceRecalibration(); 
      bootTimestamp = millis(); 
      currentState = STATE_CALIBRATING; 
      Serial.println("CMD: CALIBRAR...");
  }
  else if (cmd == "walk") {
      // 1. Verificação de Calibração
      if (!imu.isReady()) {
          currentState = STATE_ERR_CALIB; // FORÇA O ESTADO 11
          Serial.println("ERRO: PRECISA CALIBRAR!");
      }
      // 2. Verificação de Movimento Escolhido
      else if (currentGait == GAIT_NONE) {
          currentState = STATE_ERR_NO_GAIT; // FORÇA O ESTADO 12
          Serial.println("ERRO: ESCOLHA UM MOVIMENTO!");
      }
      // 3. Tudo OK -> Anda
      else {
          imu.resetYaw(); 
          imu.notifyWalkStart();
          currentState = STATE_WALKING; // ESTADO 4
          Serial.println("CMD: MARCHAR");
      }
  }
  else if (cmd == "stop") {
      robot.stop();
      currentState = STATE_READY_TO_START; 
      Serial.println("CMD: STOP");
  }

  server.send(200, "text/plain", "OK");
}

// =============================================================
// 7. SETUP (CONFIGURAÇÃO INICIAL)
// =============================================================
void setup() {
  Serial.begin(115200);
  
  // --- INICIAR WIFI ---
  WiFi.softAP(ssid, password);
  Serial.print("AP Created: "); 
  Serial.println(ssid);
  Serial.print("IP Address: "); 
  Serial.println(WiFi.softAPIP());

  // --- CONFIGURAR ROTAS DO SERVIDOR ---
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/cmd", handleCommand); 
  server.begin();
  
  // --- INICIAR I2C ---
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  
  // --- INICIAR HEXAPOD (PCA9685) ---
  if (!robot.init()) {
    Serial.println("\nERRO CRITICO: CONTROLADOR DE SERVOS (PCA9685) NAO DETETADO!");
    Serial.println("Verifique os cabos SDA/SCL e a alimentacao.");
    while(1); 
  }
  
  // --- INICIAR SENSORES (IMU E LIDAR) ---
  if(imu.init() && lidar.init()) {
     Serial.println("Sensores OK.");
  } else {
     Serial.println("Erro Sensores.");
     currentState = STATE_ERROR;
  }

  // Define estado inicial
  currentState = STATE_MENU_GAIT;
  Serial.println("SISTEMA ONLINE. USE A PAGINA WEB PARA CONTROLAR.");
}

// =============================================================
// 8. LOOP PRINCIPAL (ESTRUTURADO)
// =============================================================
void loop() {
  // 1. Manter serviços essenciais ativos
  server.handleClient(); 
  lidar.update();
  int distancia = lidar.getDistance();

  // 2. Failsafe USB (Escrever 'x' no monitor para parar tudo)
  if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') { 
        robot.stop(); 
        currentState = STATE_READY_TO_START; 
      }
  }

  // ===========================================================
  // MÁQUINA DE ESTADOS
  // ===========================================================
  
  // -----------------------------------------------------------
  // ESTADO 1: CALIBRAÇÃO (1. Calibra -> 2. Fica em Standby)
  // -----------------------------------------------------------
  if (currentState == STATE_CALIBRATING) {
    robot.stop();
    
    // Executa amostragem do MPU
    bool calibrationDone = imu.performCalibrationStep();
    // Garante tempo de aquecimento (1.5s)
    bool timeDone = (millis() - bootTimestamp > WARMUP_TIME_MS);

    if (calibrationDone && timeDone) {
      Serial.println("SISTEMA CALIBRADO. A AGUARDAR ORDEM...");
      imu.resetYaw(); // Define este ponto como ZERO (Frente)
      
      // CRÍTICO: Não vai para WALKING. Vai para STANDBY.
      currentState = STATE_READY_TO_START; 
    }
  }

  // -----------------------------------------------------------
  // ESTADO 2: STANDBY / MENUS (Robot Parado à espera de clique)
  // -----------------------------------------------------------
  else if (currentState == STATE_READY_TO_START || currentState == STATE_MENU_GAIT || currentState == STATE_MENU_SPEED) {
      robot.stop();
      // Fica aqui indefinidamente até receberes um comando na Web
  }
  // -----------------------------------------------------------
  // ESTADO 3: ROTAÇÃO PRECISA (ATUALIZADO PARA 0.5 GRAUS)
  // -----------------------------------------------------------
  else if (currentState == STATE_PRECISE_ROTATION) {
      imu.update(99999); 
      float currentYaw = imu.getYaw();
      float error = targetHeading - currentYaw;
      
      // DEBUG: Acompanha a aproximação
      Serial.print("Erro: "); Serial.println(error);

      // --- MUDANÇA 1: Tolerância apertada para 0.5 ---
      if (abs(error) <= 0.01) {
          robot.stop();
          Serial.println(">>> ALVO ATINGIDO (PRECISAO 0.5) <<<");
          currentState = STATE_READY_TO_START; 
      } 
      else {
          float dir = (error > 0) ? 1.0 : -1.0;
          
          // --- MUDANÇA 2: Velocidade Variável (Travagem) ---
          // Para não passar o alvo de 0.5, temos de abrandar no fim
          float realIntensity = TURN_INTENSITY;

          if (abs(error) < 10.0) {
             realIntensity = TURN_INTENSITY * 0.6; // 60% da força se faltar pouco
          }
          if (abs(error) < 3.0) {
             realIntensity = TURN_INTENSITY * 0.4; // 40% da força nos ultimos milimetros
          }

          // Mantemos o sinal negativo (-dir) que corrigiu a rotação invertida
          robot.updateWalking(-dir * realIntensity, 0.0, GAIT_TRIPOD, SPEED_MED, 1);
      }
  }

  // -----------------------------------------------------------
  // ESTADO 4: MARCHA (Só entra aqui se clicares em "MARCHAR")
  // -----------------------------------------------------------
  else if (currentState == STATE_WALKING) {
    
    // A. Definir sensibilidade a choques baseada no andar/velocidade
    int activeThreshold = THRESHOLD_SENSITIVE; 
    if (currentGait == GAIT_CENTIPEDE && currentSpeed == SPEED_FAST) activeThreshold = THRESHOLD_HARD;
    else if (currentGait == GAIT_SIDEWAYS) activeThreshold = THRESHOLD_HARD;
    else if (currentGait == GAIT_TRIPOD && currentSpeed == SPEED_FAST) activeThreshold = THRESHOLD_MEDIUM;

    // B. Verificar Impacto (Acelerómetro)
    if (imu.hasCrashed()) {
      Serial.println("!!! IMPACTO DETETADO !!!");
      robot.stop(); 
      impactTimestamp = millis();
      currentState = STATE_IMPACT_REACTION; 
      return; // Sai para reagir imediatamente
    }

    // C. Verificar Obstáculo (Lidar) - Ignorado no modo Lateral
    if (currentGait != GAIT_SIDEWAYS) {
        if (lidar.isActive() && distancia < MIN_DISTANCE_MM && distancia > 0) {
           Serial.println("Obstaculo Detetado!");
           robot.stop(); 
           // Decide direção aleatória para desvio de obstáculo
           avoidDirection = (random(0, 2) == 0) ? -1 : 1; 
           waitTimestamp = millis(); 
           currentState = STATE_WAIT_TO_TURN;
           return; 
        }
    }

    // D. Calcular Correção de Rumo (Manter linha reta)
    imu.update(activeThreshold); 
    float yawError = imu.getYaw(); 
    float headingCorrection = yawError * HEADING_P_GAIN;
    
    // E. Executar Movimento
    robot.updateWalking(headingCorrection, STRIDE_LENGTH, currentGait, currentSpeed, sidewaysDirection);
  }

  // -----------------------------------------------------------
  // ESTADO 5: REAÇÃO A CHOQUE (Recua)
  // -----------------------------------------------------------
  else if (currentState == STATE_IMPACT_REACTION) {
    if (currentGait == GAIT_SIDEWAYS) {
       sidewaysDirection *= -1; // Se for lateral, inverte lado
       imu.notifyWalkStart();
       currentState = STATE_WALKING;
    } else {
       // Se for frontal, recua durante 2 segundos
       if (millis() - impactTimestamp < 2000) {
          robot.updateWalking(0, -STRIDE_LENGTH, currentGait, currentSpeed, 1);
       } else {
          // Depois de recuar, prepara para virar
          avoidDirection = 1; 
          clearPathTimestamp = 0; 
          imu.notifyWalkStart(); 
          currentState = STATE_AVOIDING; 
       }
    }
  }

  // -----------------------------------------------------------
  // ESTADO 6: PAUSA ANTES DE VIRAR (Obstáculo)
  // -----------------------------------------------------------
  else if (currentState == STATE_WAIT_TO_TURN) {
    if (millis() - waitTimestamp > STABILIZE_TIME) { 
       clearPathTimestamp = 0; 
       currentState = STATE_AVOIDING; 
    }
  }

  // -----------------------------------------------------------
  // ESTADO 7: A VIRAR (Desvio de Obstáculo)
  // -----------------------------------------------------------
  else if (currentState == STATE_AVOIDING) {
    unsigned long activeTurnTime = TURN_TIME_TRIPOD;
    if (currentGait == GAIT_CENTIPEDE) activeTurnTime = TURN_TIME_CENTIPEDE;
    if (currentGait == GAIT_SIDEWAYS) activeTurnTime = TURN_TIME_SIDEWAYS;

    // Se ainda houver obstáculo ou tempo não tiver acabado
    bool obstacleClear = (distancia > CLEAR_DISTANCE_MM);
    // Se o caminho limpou agora, inicia timer de segurança
    if (obstacleClear && clearPathTimestamp == 0) clearPathTimestamp = millis();

    // Se ainda tem obstáculo OU (já não tem mas o tempo de segurança não acabou)
    if (!obstacleClear || (millis() - clearPathTimestamp < activeTurnTime)) {
       // Roda sem andar para a frente (Stride 0)
       robot.updateWalking(avoidDirection * TURN_INTENSITY, 0.0, GAIT_TRIPOD, currentSpeed, 1); 
    } 
    else { 
       // Desvio concluído
       robot.stop(); 
       waitTimestamp = millis(); 
       imu.notifyWalkStart(); 
       currentState = STATE_WAIT_TO_WALK; 
    }
  }

  // -----------------------------------------------------------
  // ESTADO 8: PAUSA ANTES DE VOLTAR A ANDAR
  // -----------------------------------------------------------
  else if (currentState == STATE_WAIT_TO_WALK) {
    if (millis() - waitTimestamp > STABILIZE_TIME) { 
       imu.resetYaw(); // Assume nova direção como frente
       currentState = STATE_WALKING; 
    }
  }

  // -----------------------------------------------------------
  // ESTADO 9: ERRO HARDWARE
  // -----------------------------------------------------------
  else if (currentState == STATE_ERROR) {
     if (debugTimer.hasExpired(2000)) Serial.println("ERRO HARDWARE.");
  }
}