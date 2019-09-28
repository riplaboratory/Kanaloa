# Librería Arduino QuickMedian
Librería para Arduino que realiza el cálculo rápido de la mediana de un array en Arduino aplicando el algoritmo QuickSelect modificado por Wirth.

Más información https://www.luisllamas.es/libreria-arduino-quickmedian/

## Instrucciones de uso
La librería QuickMedian dispone de un único método estático para el cálculo de la mediana. No es necesario, por tanto, instanciar un objeto.

Se usan templates para que funcionen con distintos tipos de variables (int, long, float…). La librería incorpora dos ejemplos, uno para int y otro para float.

Para el cálculo de la mediana de un vector se emplea la función GetMedian

```c++
int med = QuickMedian<int>::GetMedian(items, numItems);
```


## Ejemplos
La librería QuickMedian incluye los siguientes ejemplos para ilustrar su uso.
* QuickMedianInt: Ejemplo de uso para variables integer.

```c++
#include "QuickMedianLib.h"

int values100[] = { 3, 53, 70, 56, 18, 85, 27, 14, 37, 94, 9, 55, 40, 60, 52, 61, 15, 65, 13, 8, 57, 97, 69, 4, 35, 82, 22, 73, 59, 68, 78, 24, 21, 36, 71, 80, 74, 39, 17, 12, 29, 76, 49, 51, 30, 90, 88, 2, 84, 50, 62, 28, 77, 43, 5, 16, 58, 26, 32, 34, 1, 75, 66, 95, 38, 89, 67, 87, 100, 54, 92, 81, 25, 83, 46, 33, 23, 45, 96, 99, 79, 48, 11, 31, 7, 6, 19, 91, 93, 44, 47, 98, 86, 41, 63, 20, 72, 10, 42, 64 };
int values100Length = sizeof(values100) / sizeof(int);

void setup()
{
	Serial.begin(115200);

	Serial.println("Mediana de 100 integers");
	long timeCount = micros();
	int med = QuickMedian<int>::GetMedian(values100, values100Length);
	timeCount = micros() - timeCount;
  Serial.print(med);
	Serial.println();
	Serial.print(timeCount);
	Serial.println("us");
}


void loop()
{
}
```

* QuickMedianFloat: Ejemplo de uso para variables float.

```c++
#include "QuickMedianLib.h"

float values100[] = { 3, 53, 70, 56, 18, 85, 27, 14, 37, 94, 9, 55, 40, 60, 52, 61, 15, 65, 13, 8, 57, 97, 69, 4, 35, 82, 22, 73, 59, 68, 78, 24, 21, 36, 71, 80, 74, 39, 17, 12, 29, 76, 49, 51, 30, 90, 88, 2, 84, 50, 62, 28, 77, 43, 5, 16, 58, 26, 32, 34, 1, 75, 66, 95, 38, 89, 67, 87, 100, 54, 92, 81, 25, 83, 46, 33, 23, 45, 96, 99, 79, 48, 11, 31, 7, 6, 19, 91, 93, 44, 47, 98, 86, 41, 63, 20, 72, 10, 42, 64 };
float values100Length = sizeof(values100) / sizeof(float);

void setup()
{
	Serial.begin(115200);

	Serial.println("Mediana de 100 integers");
	long timeCount = micros();
	float med = QuickMedian<float>::GetMedian(values100, values100Length);
	timeCount = micros() - timeCount;
  Serial.print(med);
	Serial.println();
	Serial.print(timeCount);
	Serial.println("us");
}


void loop()
{
}
```
