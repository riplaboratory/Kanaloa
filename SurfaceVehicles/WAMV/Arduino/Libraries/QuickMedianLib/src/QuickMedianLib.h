/***************************************************
Copyright (c) 2017 Luis Llamas
(www.luisllamas.es)

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/

#ifndef _QuickMedianLib_h
#define _QuickMedianLib_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
	#include "WProgram.h"
#endif

template <typename T>
class QuickMedian
{
public:
	static T GetMedian(T* data, int dataLength);

private:
	static T kthSmallest(T* data, int dataLength, int kth);
};

template <typename T>
T QuickMedian<T>::GetMedian(T* data, int dataLength)
{
	int medianIndex = dataLength & 1 ? dataLength / 2 : (dataLength / 2) - 1;
	return kthSmallest(data, dataLength, medianIndex);
}

template <typename T>
T QuickMedian<T>::kthSmallest(T* data, int dataLength, int kth)
{
	int i, j, l, m;
	T x;
	l = 0;
	m = dataLength - 1;
	while (l < m)
	{
		x = data[kth];
		i = l;
		j = m;
		do
		{
			while (data[i] < x) i++;
			while (x < data[j]) j--;
			if (i <= j)
			{
				T t = data[j];
				data[j] = data[i];
				data[i] = data[j];
				i++;
				j--;
			}
		}
		while (i <= j);
		if (j < kth) l = i;
		if (kth < i) m = j;
	}
	return data[kth];
}

#endif

