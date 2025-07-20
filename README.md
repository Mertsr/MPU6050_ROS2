# MPU6050_ROS2

**MPU6050_ROS2**, Arduino ile kullanılan MPU6050 IMU sensöründen veri alıp, bu verileri ROS2 ortamında IMU veya Odometry mesajı olarak yayınlamanızı sağlayan bir projedir. Python (ROS2 düğümleri) ve Arduino (seri veri gönderimi) kodlarını içerir.

---

## Özellikler

- **Arduino üzerinden IMU veri akışı** (MPU6050 açılarının seri porttan gönderimi)
- **ROS2 IMU ve Odom Düğümleri:**  
  - `/imu/data` veya `/odom` topic'lerine gerçek zamanlı veri yayını
  - Euler açılarından quaternion dönüşümü
  - TF yayını ile çerçeve ilişkileri
- **Gerçek zamanlı grafik gösterim** (isteğe bağlı)
- **Tamamen açık kaynak ve ayarlanabilir port/dönüşüm parametreleri**

---

## Klasör ve Dosya Yapısı

- `GetAngle.ino`: Arduino üzerinde çalışan, MPU6050 sensöründen açıları (X, Y, Z) seri port üzerinden gönderen kod.
- `imu_msg.py`: Python/ROS2 düğümü. Arduino'dan gelen açısal verileri okuyup `/imu/data` topic'ine IMU mesajı olarak yayınlar. Aynı zamanda TF mesajı da yollar.
- `imu2.py`: Alternatif olarak gelen verileri ROS2 ortamında Odometry mesajı olarak yayınlayan Python/ROS2 düğümü.
- `getAngle.py`: MPU6050 verilerini gerçek zamanlı olarak matplotlib ile görselleştiren bağımsız Python scripti.
- Diğer yardımcı dosyalar, dönüşüm fonksiyonları ve örnekler.

---

## Arduino Tarafı

### MPU6050 Arduino Kodu

```c++
#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets();
}

void loop() {
  mpu.update();
  if((millis()-timer)>10){
    Serial.print("X : "); Serial.print(mpu.getAngleX());
    Serial.print("\tY : "); Serial.print(mpu.getAngleY());
    Serial.print("\tZ : "); Serial.println(mpu.getAngleZ());
    timer = millis();
  }
}
```

---

## ROS2 Tarafı

### IMU Mesajı Yayını (`imu_msg.py`)

- Arduino seri portundan gelen açı verileri alınır.
- Euler açıları quaternion'a çevrilir.
- `/imu/data` topic'ine IMU mesajı ve TF yayını yapılır.

Başlatmak için:
```bash
python3 imu_msg.py
```
veya
```bash
ros2 run <paket_adı> imu_msg.py
```

### Odom Mesajı Yayını (`imu2.py`)

- Gelen veri Odometry mesajına çevrilip `/odom` topic'ine yayınlanır.

Başlatmak için:
```bash
python3 imu2.py
```

---

### Dinamik Grafik Gösterimi (`getAngle.py`)

Sensörden gelen açı verilerini dinamik olarak çizmek için:
```bash
python3 getAngle.py
```
> Port adını ve baudrate'i kendi Arduino'nuzun bağlantısına göre değiştirin.

---

## Gereksinimler

- Arduino + MPU6050 modülü
- Python 3.8+ (veya ROS2 ortamınıza uygun sürüm)
- Gerekli Python kütüphaneleri:
    ```bash
    pip install rclpy sensor_msgs nav_msgs tf2_ros serial matplotlib numpy
    ```

---

## Katkı ve Lisans

Katkıda bulunmak için fork'layın, yeni bir branch açın ve pull request gönderin.

---

## İletişim

Proje sahibi: [Mertsr](https://github.com/Mertsr)

---
