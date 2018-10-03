// ValColl.h

#ifndef _VALCOLL_h
#define _VALCOLL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class ValColl
{
 public:
	ValColl(int elements);

	void Add(int value);

	int Value();

private:
	int * arr;			// tablica na valuesy int[]
	 int _elements;		// wielkoas tablicy arr
	 int _cursor;		// biezacy index tablicy
	 int _value;		// wynik usredniowy ze wszystkich elementow w arr
	 bool _przelicz;	// cz wymaga przeliczenia
	 
};


#endif

