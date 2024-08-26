ÖZET
----
Bu klasör takımımızın haberleşme testi sürecinde 3 ana birimimizin (Ana Aviyonik, Görev Yükü, Yer İstasyonu) kullanacağı
kod bloklarını içermektedirler.

HABERLEŞME TESTİ NEDİR?
-----------------------
Yer   İstasyonları,   görev   yükü   ve   ana   aviyonik   sistem   arasındaki   haberleşmenin  en az 5 kilometre öteden
gerçekleşebildiğinin kanıtlanması gerektiren testtir. Test içeriği olarak  haberleşmede  RF  modülü  kullanılacaktır  ve
gönderilen veri paketinde GPS verilerinin bulunması zorunludur.
    Test gereği görev yükü ve ana aviyonik sistemimiz GPS verilerini toplayacak ve yer istasyonunun sahip olduğu   adres
ve frekansa bu verileri göndereceklerdir. Yer istasyonumuz ise düzenli olarak gelen veri paketlerini paketlerin
bulundurduğu CRC-8 byte'ı ile kontrol edecektir. Ayrıca yine veri paketinin bulundurduğu tanımlama karakteri ile verinin
hangi gönderici tarafından gönderildiğini teyit edecek ve buna göre değişkenleri güncelleyecektir.