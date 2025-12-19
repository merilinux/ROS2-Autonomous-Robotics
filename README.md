# ROS 2 Otonom Robotik ve Görüntü İşleme Portfolyosu 🤖

Bu proje, TurtleBot3 (Waffle Pi) robotunun Gazebo simülasyon ortamında Lidar ve Kamera sensörlerini kullanarak otonom görevleri yerine getirmesini sağlar.

## 🛠 Ödev 1: Belirlenen renkteki hedefe gitmek ve eşzamanlı hafif engelden kaçış algoritması
Bu aşamada robot, Lidar sensöründen gelen `/scan` verilerini anlık olarak analiz eder. Belirli bir mesafe eşiğinin altındaki engelleri tespit ettiğinde, çarpmayı önlemek için otonom olarak yön değiştirir.

### Odev 1 videosu
[![Ödev 1 Video](https://img.youtube.com/vi/31eWgMP5QUo/0.jpg)](https://www.youtube.com/watch?v=31eWgMP5QUo)
---
haberleşme mimarisi ve mesaj Yapısı
Topic: /vision/target_infoMesaj 
Bileşenleri:float64 distance: Robotun ilerleme istikametinde 
Lidar tarafından saptanan en yakın objenin metre cinsinden net uzaklığıdır.f
loat64 angle: Algılanan engelin robotun merkez eksenine göre radyan cinsinden konumudur.
string target_type: Algılanan nesnenin sınıfını (Örn: "obstacle") belirten etikettir.

3. Algoritma DetaylarıLidar Segmentasyonu: 360 derecelik LaserScan verisi;
4.  Sağ-Ön ve Sol-Ön sektörlerine ayrılarak analiz edilir.Karar Mekanizması: Ön sektördeki herhangi bir ışın metre cinsinden eşik değerinin altına düştüğünde reaktif kaçınma algoritması tetiklenir.
5.  Manevra Stratejisi: Engel saptandığında linear.x hızı düşürülür ve robot, Lidar verisinde daha fazla boşluk olan tarafa doğru angular.z değerini artırarak otonom dönüş yapar.

## Ödev 2: Otonom Şerit Takip ve Tabela Algılama
Bu çalışma, robotun kamera kullanarak karmaşık bir parkurda ilerlemesini sağlar. 
Haberleşme Mimarisi ve Mesaj Yapısı
Topic: /vision/lane_error Tip: std_msgs/msg/Float64
Tanım: Şeridin ağırlık merkezi ile görüntünün orta çizgisi arasındaki piksel farkını (hata payı) iletir.
Topic: /vision/stop_detected Tip: std_msgs/msg/Bool Tanım: Tabela algılama düğümü (StopSignNode) dur tabelasını onayladığında sisteme True bayrağı fırlatır. 
Görüntü İşleme Hattı (Image Pipeline)HSV Renk Filtreleme: Işık ve gölge değişimlerinden etkilenmemek adına görüntü BGR'dan HSV uzayına dönüştürülür ve beyaz/kırmızı maskeler oluşturulur.
ROI (İlgi Alanı): İşlem yükünü azaltmak için görüntünün sadece alt %40'lık kısmı (yol alanı) analize dahil edilir.Moments Analizi: Maskelenmiş görüntüdeki beyaz piksellerin cv2.moments yardımıyla ağırlık merkezi bulunur.
Kontrol SistemiP-Controller (Oransal Kontrolcü): Hesaplanan lane_error değeri, bir kp katsayısı ile çarpılarak robotun açısal hızına atanır. Bu, robotun virajlara yumuşak bir şekilde girmesini sağlar.

Şerit Takibi:OpenCV ile ROI (Region of Interest) belirlenerek beyaz şeritlerin ağırlık merkezi hesaplanır ve robot yolu ortalar.
Tabela Algılama:Hiyerarşik kontur analizi yöntemiyle kırmızı zemin üzerindeki beyaz dikdörtgen (Dur Tabelası) tespit edilir.
Otonom Durma:Tabela algılandığı anda `stop_detected` sinyali üretilir ve kontrolcü robotu tamamen durdurur.

### Ödev 2 Videosu
[![Ödev 2 Video](https://img.youtube.com/vi/Jt4lpHe4IVQ/0.jpg)](https://www.youtube.com/watch?v=Jt4lpHe4IVQ)



---




-------------------ADIM ADIM CALISTIRALIM-------------------

# Workspace kurulumu
cd ~/ROS2_Robotics_Assignments
colcon build
source install/setup.bash

# Simülasyonu Başlatma
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch gazebo_ros gazebo.launch.py world:=/home/meri/seritbeyaz.world

# Düğümleri Çalıştırma
ros2 run goruntu_isleme_odev2 lane_node
ros2 run goruntu_isleme_odev2 stop_sign_node
ros2 run goruntu_isleme_odev2 lane_follow_controller
