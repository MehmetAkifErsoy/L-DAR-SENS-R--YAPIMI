// STM32 Kodu - TF Luna Lidar Tarayıcı (Komut Destekli, Orijinal Hesaplamalar)
#include <Servo.h>

// TF-Mini veya TF-Luna
#define TFMINI_BAUDRATE 115200
#define TFMINI_DATARATE 10.0f // ms (200Hz)  
int distance = 0;
int strength = 0;
float temp = 0;

// Servolar
Servo servo_g, servo_d;
#define SERVO_G_PIN PB0
#define SERVO_D_PIN PB1
#define SERVO_POS_NEUTRAL 90
#define SERVO_POS_MIN SERVO_POS_NEUTRAL - 40
#define SERVO_POS_MAX SERVO_POS_NEUTRAL + 40
int servo_angle = SERVO_POS_MIN; // mevcut servo pozisyonu

// Step Motor
#define DIR_PIN PB15 
#define STEP_PIN PA8
#define PULSE_PER_REV 4800
float TIME_PER_REV = 500.0f; // ms (değiştirilebilir)
int TIME_PER_PULSE;          // us (hesaplanacak)
int PULSE_PER_DATAPOINT;     // (hesaplanacak)

// Dış haberleşme
#define EXTERNAL_BAUDRATE 115200
char serial_buffer[32];
float theta, phi, rho;

// Tarama Kontrolü
bool scanning = false;       // Tarama durumu
int scanSpeedLevel = 3;      // 1-5 arası (5 en hızlı)
const float speedFactors[] = {3.0, 1.5, 1.0, 0.7, 0.4}; // Hız faktörleri

void setup() {
  // Step motor
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  
  // Servolar
  servo_g.attach(SERVO_G_PIN);
  servo_d.attach(SERVO_D_PIN);
  servo_pos(servo_angle);
  
  // Seri portlar
  Serial.begin(EXTERNAL_BAUDRATE);
  Serial3.begin(TFMINI_BAUDRATE);
  flushSerial3();
  
  // Tarama parametrelerini hesapla
  updateScanningParams();
  
  // Başlangıç mesajı
  Serial.println("LIDAR_READY");
}

uint16_t pulses = 0;
int8_t servo_dir = 1;
int8_t offset = 0;

void loop() {
  // Gelen komutları kontrol et
  checkCommands();
  
  // Tarama aktifse işleme devam et
  if (scanning) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(TIME_PER_PULSE);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(TIME_PER_PULSE);
    pulses++;

    if ((pulses + offset) % PULSE_PER_DATAPOINT == 0) {
      getTFminiData(&distance, &strength, &temp);
      send_pos();
    }

    if (pulses == PULSE_PER_REV) {
      pulses = 0;
      servo_angle += servo_dir;
      if (servo_angle > SERVO_POS_MAX || servo_angle < SERVO_POS_MIN) {
        servo_dir = -servo_dir;
        servo_angle += servo_dir;
      }
      servo_pos(servo_angle);
    }
  }
}

// Gelen komutları kontrol et
void checkCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch(cmd) {
      case 'b': // Taramayı başlat
        scanning = true;
        Serial.println("SCANNING_START");
        break;
      case 'd': // Taramayı durdur
        scanning = false;
        Serial.println("SCANNING_STOP");
        break;
      case 'r': // Taramayı yeniden başlat
        resetScanner();
        Serial.println("SCANNING_RESET");
        break;
      case '1': // Hız seviyesi 1 (en yavaş)
      case '2':
      case '3':
      case '4':
      case '5': // Hız seviyesi 5 (en hızlı)
        scanSpeedLevel = cmd - '0';
        updateScanningParams();
        Serial.print("SPEED_LEVEL_");
        Serial.println(scanSpeedLevel);
        break;
    }
  }
}

// Tarama parametrelerini güncelle
void updateScanningParams() {
  TIME_PER_REV = 500.0f * speedFactors[scanSpeedLevel-1];
  TIME_PER_PULSE = (TIME_PER_REV * 1000.0f) / PULSE_PER_REV;
  PULSE_PER_DATAPOINT = max(1, (TFMINI_DATARATE * 1000.0f) / TIME_PER_PULSE);
}

// Tarayıcıyı sıfırla
void resetScanner() {
  scanning = true;
  pulses = 0;
  servo_angle = SERVO_POS_MIN;
  servo_dir = 1;
  servo_pos(servo_angle);
}

// Pozisyon gönder (X, Y, Z koordinatları)
void send_pos() {
  if (distance == 0) return;
  
  theta = (float)pulses * 360.0f / PULSE_PER_REV; // motor açısı
  theta *= PI / 180.0f; // radyana çevir
  
  // Orijinal formüller
  phi = (float)servo_angle * -0.3f + 71.0f;
  phi = 2.0f * phi - 90.0f;
  phi *= PI / 180.0f; // radyana çevir
  
  rho = distance - 5.5f; // lidar mesafesi
  
  snprintf(serial_buffer, sizeof(serial_buffer), "%d\t%d\t%d\t%d\n",
           (int)(rho * cos(phi) * cos(theta)),
           (int)(rho * cos(phi) * sin(theta)),
           (int)(rho * sin(phi)),
           strength);
  Serial.print(serial_buffer);
}

// Servo pozisyonu ayarla
void servo_pos(int angle) {
  angle = constrain(angle, SERVO_POS_MIN, SERVO_POS_MAX);
  servo_g.write(180 - angle);
  servo_d.write(angle);
}

// TF Mini'den veri al
void getTFminiData(int* distance, int* strength, float* temp) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  
  unsigned long startTime = millis();
  while ((millis() - startTime) < 100) { // En fazla 100ms bekle
    if (Serial3.available()) {
      rx[i] = Serial3.read();
      if (rx[0] != 0x59) {
        i = 0;
      } else if (i == 1 && rx[1] != 0x59) {
        i = 0;
      } else if (i == 8) {
        for (j = 0; j < 8; j++) {
          checksum += rx[j];
        }
        if (rx[8] == (checksum % 256)) {
          *distance = rx[2] + rx[3] * 256;
          *strength = rx[4] + rx[5] * 256;
          *temp = (rx[6] + rx[7] * 256.0f) / 8 - 256;
        }
        i = 0;
        break;
      } else {
        i++;
        if (i > 8) i = 0;
      }
    }
  }
  flushSerial3();
}

// Seri buffer boşalt
void flushSerial3() {
  while (Serial3.available()) { Serial3.read(); }
}