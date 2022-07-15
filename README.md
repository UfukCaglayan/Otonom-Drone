# Otonom-Drone
Simülasyon ortamının içinde belli bir alanda şekil ve renklerine göre rastgele yerleştirilmiş hedefleri kapsama yolu planlama algoritmalarıyla tarayarak yerlerinin tespit edilmesini sağladık. 
Hedefler arasında geçişler yaparak belirtilen görevleri otonom olarak uygulatmış olduk.

-Simülasyon ortamı:
![sim1](https://user-images.githubusercontent.com/22642119/179230042-5698b094-8af1-45fa-b374-e73db6ac06f8.png)

SITL simülatörünü çalıştırarak mavlink komutlarıyla Arducopter’in simülasyon ortamında hareket edebilmesi sağlandı.

![image](https://user-images.githubusercontent.com/22642119/179230373-b3c9d200-4901-4270-a7a1-837b65c5f4ac.png)

DroneKit kütüphanesi ve python kodları kullanılarak paralel tarama yöntemiyle alanın taranması sağlandı.

![image](https://user-images.githubusercontent.com/22642119/179231504-da8386a4-24d2-4e27-b7ab-f1dbb8ac821c.png)

Diğer bir tarama yöntemi olan spiral tarama yöntemiyle alanın taranması sağlandı.

![image](https://user-images.githubusercontent.com/22642119/179230507-e1982bb5-dff5-4361-81c7-0278d3722473.png)

Arducopter’in altındaki kamera ile taranan alan izlenirken görüntü işleme yöntemleriyle kırmızı hedef tespit edildi.

![image](https://user-images.githubusercontent.com/22642119/179230552-66f3de81-8ab4-4ff8-b5d7-a94be32966b9.png)

Arducopter hedeflere yakın bir şekilde uçarken her daha yakın noktaya geldiğinde mevcut koordinatları hafızada tutuldu.

![image](https://user-images.githubusercontent.com/22642119/179230577-c121a1dd-f520-42b6-9bbb-40734c6c9453.png)

Alan tarama ve hedef tespiti adımlarından sonra mavi hedefin üstüne gelinerek hedef ortalama işlemi yapıldı. 

![image](https://user-images.githubusercontent.com/22642119/179230629-a9722d6c-6ee7-4900-806d-7eab544bde04.png)

Arducopter ortalama işlemini yaparken mavi hedefin üstünde 5 metreye kadar alçaldı. 
Alçalma işlemi yapılırken ortalama işlemi de yapılmaya devam edilerek tam ortasına gelecek şekilde alçalması sağlandı.

![image](https://user-images.githubusercontent.com/22642119/179230681-8bb6e1a7-f8fb-4384-9b41-901214d57635.png)

Benzer şekilde kırmızı hedefin üstüne aynı işlemler uygulandı.

![image](https://user-images.githubusercontent.com/22642119/179230706-6b20f089-f780-4345-bdbe-57a6903e832a.png)

Görev tekrarının yapılmasından sonra Arducopter’e eve dönüş komutu verildi ve ilk kalktığı noktaya dönmesi sağlandı.

![image](https://user-images.githubusercontent.com/22642119/179230756-a94af4aa-aea6-44f8-ab41-2905020bf600.png)


