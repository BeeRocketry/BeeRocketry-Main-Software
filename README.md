# BeeRocketry Roket Takımı

Bu repo, BeeRocketry Roket Takımı'nın ana aviyonik sistem, görev yükü ve yer istasyonu sistemlerinin kodlarını ve kullanılan kütüphanelerin kodlarını içermektedir.

## BeeRocketry Roket Takımı Kimdir?
BeeRocketry Roket Takımı, Çankaya Üniversitesi öğrencilerinden oluşan ve Mühendislik ve Teknoloji Topluluğu altında görevlerini yürüten bir proje takımıdır. Bu repoda bulunan kodlar, BeeRocketry Roket Takımı'nın 2024 Teknofest Roket Yarışması Yüksek İrtifa Kategorisi için yazılmıştır.

## Repo Amacı
Bu repodaki kodlar, takımın yarışmada kullanacağı test kodlarını ve tüm diğer özel kütüphaneleri içermektedir. AltitudeEstimation kütüphanesi dışında kullanılan tüm özel kütüphaneler takımımız tarafından tasarlanmış ve yazılmıştır.

## Katkıda Bulunanlar
Kod bloklarını yazan kişiler aşağıda belirtilmiştir:

- **Bartu Özcan** ([GitHub Profili](https://github.com/baftii))  
  Deneme Kodları, BME280, BMP2xx, BMP388, BNO055, Debug Printer, I2C, ICM20948, Magnetometer - MMC5603, MPU9250, RF
- **Cemanur Adıgüzel**  
  GPS
- **Juan Gallostra Acín** ([AltitudeEstimation](https://github.com/juangallostra/AltitudeEstimation/commits?author=juangallostra))  
  AltitudeEstimation

## Ön Koşul Kütüphaneler
Reponun çalışması için gerekli olan ön koşul kütüphaneler:

- Arduino.h
- Wire.h
- HardwareSerial.h
- Reefwing_AHRS.h
- Reefwing_imutypes.h
- SPI.h
- TinyGPSPlus.h

## Klasör Düzeni
Klasör düzeni aşağıda belirtilmiştir. Belirtilen dosyalar dışında kalan dosyalar, PlatformIO tarafından eklenen otomatik dosyalardır. Klasörlerin temel içerikleri hakkında bilgi verilmiştir; detaylı bilgiler için klasörlerde bulunan README dosyalarına bakılabilir. Ayrıca, özel kütüphanelerin detaylı bilgisi ayrı repolar halinde paylaşılıp buraya link eklemesi zamanla yapılacaktır.

### BeeRocketry Ana Yazılım

- **Deneme Kodları**  
  Bu klasör, yarışmanın AHR aşamasında kullanılmak üzere yazılmış tüm test kodlarını içermektedir.
  - **Arayüz Testi**  
    - Gönderici Devre
      - `gonderici.cpp`

    - Yer İstasyonu
      - `yeristasyonu.cpp`

    - `README.md`

  - **Fonksiyonellik Aviyonik Testi**  
    - Aviyonik Testi

      - `aviyonik.cpp`
    - Fonksiyonellik

      - `Fonksiyonellik_Alici.cpp`
      - `Fonksiyonellik_Verici.cpp`

    - `README.md`

  - **Haberleşme Testi**  
    - Ana Kart
      - `Anakart.cpp`

    - Görev Yükü
      - `Gorevyuku.cpp`

    - Yer İstasyonu
      - `Yeristasyonu.cpp`

    - `README.md`

  - **Kurtarma Sistem Testi**  
    - `AnaAviyonik.cpp`
    - `KurtarmaSistemi.cpp`
    - `README.md`

- **lib**  
  Bu klasör, tüm özel kütüphaneleri saklamaktadır.
  - AltitudeEstimation
    - `algebra.cpp`
    - `algebra.h`
    - `altitude.cpp`
    - `altitude.h`
    - `filters.cpp`
    - `filters.h`

  - BME280
    - `bme280.cpp`
    - `bme280.h`

  - BMP2xx
    - `bmp2xx.cpp`
    - `bmp2xx.h`

  - BMP388
    - `bmp388.cpp`
    - `bmp388.h`

  - BNO055
    - `BNO055.cpp`
    - `BNO055.h`

  - Debug Printer
    - `debugprinter.h`

  - GPS
    - `gps.cpp`
    - `gps.h`

  - I2C
    - `I2C.cpp`
    - `I2C.h`

  - ICM20948
    - `ICM20948.cpp`
    - `ICM20948.h`

  - Magnetometer - MMC5603
    - `MMC5603.cpp`
    - `MMC5603.h`

  - MPU9250
    - `Mahony.cpp`
    - `Mahony.h`
    - `MPU9250.cpp`
    - `MPU9250.h`

  - RF
    - `rf.cpp`
    - `rf.h`

  - `README.md`

- `README.md`
