import processing.serial.*;
import peasy.*;

Serial serial;
PeasyCam cam;
ArrayList<PVector> pointList;
final int SERIAL_SPEED = 115200;

boolean scanActive = false;
int speedLevel = 3;
float pointSize = 2.0;
boolean showSurface = true;

void setup() {
  size(900, 700, P3D);
  surface.setTitle("3D Lidar Viewer");
  surface.setResizable(true);

  colorMode(RGB, 255, 255, 255);
  pointList = new ArrayList<PVector>();

  cam = new PeasyCam(this, 500);
  cam.rotateZ(-PI/4);
  cam.rotateX(-PI/4);

  initSerialConnection();
}

void draw() {
  perspective();
  background(33);

  fill(50);
  ellipse(0, 0, 10, 10);
  drawAxes(200);

  readSerialData();

  // Nokta bulutunu çiz
  strokeWeight(pointSize);
  stroke(255);
  for (PVector v : pointList) {
    point(v.x, v.y, v.z);
  }

  // Mesh çizimi
  if (showSurface && pointList.size() > 100) {
    drawSurfaceMesh();
  }

  displayInfo();
}

void drawSurfaceMesh() {
  int step = 10;
  float maxDist = 150;
  stroke(0, 180, 255, 120);
  strokeWeight(3);
  fill(0, 120, 255, 40);
  beginShape(TRIANGLES);
  for (int i = 0; i < pointList.size() - step - 1; i += step) {
    PVector v1 = pointList.get(i);
    PVector v2 = pointList.get(i + step);
    PVector v3 = pointList.get(i + 1);
    if (dist(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z) < maxDist &&
        dist(v1.x, v1.y, v1.z, v3.x, v3.y, v3.z) < maxDist &&
        dist(v2.x, v2.y, v2.z, v3.x, v3.y, v3.z) < maxDist) {
      vertex(v1.x, v1.y, v1.z);
      vertex(v2.x, v2.y, v2.z);
      vertex(v3.x, v3.y, v3.z);
    }
  }
  endShape();
}

void readSerialData() {
  if (serial == null) return;
  while (serial.available() > 0) {
    String serialString = serial.readStringUntil('\n');
    if (serialString != null) {
      serialString = trim(serialString);
      if (serialString.startsWith("LIDAR_") || 
          serialString.startsWith("SCANNING_") || 
          serialString.startsWith("SPEED_")) {
        println(serialString);
        if (serialString.equals("SCANNING_START")) scanActive = true;
        if (serialString.equals("SCANNING_STOP")) scanActive = false;
      } else {
        String[] coordinates = split(serialString, '\t');
        if (coordinates.length >= 3) {
          try {
            float x = float(coordinates[0]);
            float y = float(coordinates[1]);
            float z = float(coordinates[2]);
            boolean addPoint = abs(x) < 10000 && abs(y) < 10000 && abs(z) < 10000;
            if (addPoint && pointList.size() > 0) {
              PVector last = pointList.get(pointList.size()-1);
              if (dist(x, y, z, last.x, last.y, last.z) > 200) addPoint = false;
            }
            if (addPoint) {
              pointList.add(new PVector(x, y, z));
              if (pointList.size() > 100000) pointList.remove(0);
            }
          } catch (Exception e) {
            // Veri hatası, atla
          }
        }
      }
    }
  }
}

void displayInfo() {
  cam.beginHUD();
  fill(255);
  textSize(16);
  textAlign(LEFT);
  text("Nokta Sayısı: " + pointList.size(), 10, 20);
  text("Tarama: " + (scanActive ? "AÇIK" : "KAPALI"), 10, 40);
  text("Hız: " + speedLevel + "/5", 10, 60);
  text("Nokta Boyutu: " + nf(pointSize, 1, 1), 10, 80);
  text("Mesh: " + (showSurface ? "AÇIK (M ile kapat)" : "KAPALI (M ile aç)"), 10, 100);
  textAlign(LEFT);
  text("Fare: Sağ tık sürükle = Döndür", 10, height-80);
  text("Fare: Sol tık sürükle = Kaydır", 10, height-60);
  text("Fare tekerleği = Yakınlaştır/Uzaklaştır", 10, height-40);
  text("Çift tıklama = Bakış açısını sıfırla", 10, height-20);
  textAlign(RIGHT);
  text("B: Başlat, D: Durdur, R: Yeniden Başlat", width-20, height-100);
  text("1-5: Hız Ayarı, X: Temizle, S: Kaydet", width-20, height-80);
  text("+ / -: Nokta Boyutunu Büyüt/Küçült", width-20, height-60);
  text("M: Mesh Aç/Kapat", width-20, height-40);
  text("ESC: Çıkış", width-20, height-20);
  cam.endHUD();
}

void drawAxes(float size) {
  strokeWeight(2);
  stroke(255, 0, 0); line(0, 0, 0, size, 0, 0); // X
  stroke(0, 255, 0); line(0, 0, 0, 0, size, 0); // Y
  stroke(0, 0, 255); line(0, 0, 0, 0, 0, size); // Z
}

void initSerialConnection() {
  String[] portNames = Serial.list();
  for (int i = 0; i < portNames.length; i++) {
    println(i + ": " + portNames[i]);
  }
  try {
    if (portNames.length == 0) {
      println("Seri port bulunamadı!");
      return;
    } else if (portNames.length == 1) {
      serial = new Serial(this, portNames[0], SERIAL_SPEED);
      println("Bağlandı: " + portNames[0]);
    } else {
      serial = new Serial(this, portNames[0], SERIAL_SPEED);
      println("Varsayılan port kullanıldı: " + portNames[0]);
      println("Farklı bir port için program yeniden başlatılmalı");
    }
  } catch (Exception e) {
    println("Seri port hatası: " + e.getMessage());
  }
}

void keyPressed() {
  if (key =='x' || key == 'X') {
    pointList.clear();
    println("Tüm noktalar temizlendi");
  } else if (key == 'b' || key == 'B') {
    if (serial != null) serial.write('b');
    println("Taramayı başlat komutu gönderildi");
  } else if (key == 'd' || key == 'D') {
    if (serial != null) serial.write('d');
    println("Taramayı durdur komutu gönderildi");
  } else if (key == 'r' || key == 'R') {
    if (serial != null) serial.write('r');
    println("Yeniden başlat komutu gönderildi");
  } else if (key >= '1' && key <= '5') {
    speedLevel = key - '0';
    if (serial != null) serial.write(key);
    println("Hız seviyesi " + speedLevel + " olarak ayarlandı");
  } else if (key == 's' || key == 'S'){
    savePoints("lidar_points.txt");
  } else if (key == '+' || key == '=') {
    pointSize = min(pointSize + 0.5, 10.0);
    println("Nokta boyutu: " + pointSize);
  } else if (key == '-' || key == '_') {
    pointSize = max(pointSize - 0.5, 0.5);
    println("Nokta boyutu: " + pointSize);
  } else if (key == 'm' || key == 'M') {
    showSurface = !showSurface;
    println("Mesh gösterimi: " + (showSurface ? "AÇIK" : "KAPALI"));
  }
}

void windowResized() {
  cam.setViewport(0, 0, width, height);
}

void savePoints(String filename) {
  PrintWriter pw = createWriter(filename);
  for(int i = 0; i < pointList.size(); i++) {
    PVector v = pointList.get(i);
    pw.println((int)v.x + " " + (int)v.y + " " + (int)v.z);
  }
  pw.flush();
  pw.close();
  println("Nokta bulutu kaydedildi: " + filename);
}
