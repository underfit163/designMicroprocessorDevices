# designMicroprocessorDevices
Design of microprocessor devices based on single-chip microcontrollers.

## ЗАДАНИЕ
Спроектировать микропроцессорное устройство, реализующее следующие функции:

Измерение пути движения автомобиля: датчик через каждый метр формирует отрицательный импульс длительностью 1 мс, максимальный пробег – 1000 км;
Измерение расхода топлива: датчик формирует импульсный сигнал ТТЛ-уровня типа меандр, частота которого пропорциональна расходу топлива в диапазоне (1-500) Гц. Допустимая погрешность измерения – 0,1%.
Начало пути – перепад напряжения из 1 в 0, конец – из 0 в 1. Данные измерения каждую секунду выводить на ЖКИ-индикатор с указанием типа параметра и записывать во внешнюю флеш-память с определенной периодичностью (например, через каждые 10 км). Внешняя память может быть выполнена с любым последовательным интерфейсом. 

## ИТОГИ
В ходе выполнения курсового проекта был разработан алгоритм работы, структура и принципиальная схема системы, предназначенной для приема данных по прерыванию, измерения импульсных сигналов счетчиком- таймером и их обработки, в соответствии с техническим заданием, программное обеспечение. 
Программа была написана на языке программирования Си, система реализована средствами разработки и отладки Proteus 8 с использованием компилятора Atmel Studio 7.0.
Результаты работы контроллера отображаются на индикаторе ЖКИ и выводятся во флеш-память по интерфейсу I2C.

![image](https://user-images.githubusercontent.com/81982349/131558271-928ca8c1-e6d0-4cc8-9c42-ce60b9d5a421.png)
