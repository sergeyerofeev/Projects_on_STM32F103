## i2c_mlx90393
#### Код для получения данных с датчика MLX90393 по трём осям XYZ

#### Описание
Для проекта использована следующая плата с датчиком:

<p align="center">
    <img src="git_image/image2.png" style="height: 306; width: 525; object-fit: contain">
</p>

Подключение к микроконтроллеру STM32 выполняется следующим образом:

>PB7 -> SDA  
>PB6 -> SCL  
>PB1 -> INT
___
Представленный код реализует манипулятор следующего исполнения:  
<p align="center">
    <img src="git_image/image1.png" style="height: 357; width: 405; object-fit: contain">
</p>  

**Датчик необходимо размещать в центре сферы описываемой магнитом.**  

В коде производится вычисление среднего значения из восьми измерений каждой оси.

---