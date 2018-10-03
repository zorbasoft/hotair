// Kolekcja-ring do usredniania wyniku
// 2018 Pawel Kaminski
// zorbasoft.pk@gmail.com

#include "ValColl.h"

// konstruktor + rozmiar tablicy
ValColl::ValColl(int elements)
{
	_elements = elements;
	arr = new int[elements];
}

// wstawia nowa wartosc zastepujac najstarsza
void ValColl::Add(int value) {
	arr[_cursor] = value;
	_cursor++;
	if (_cursor >= _elements) _cursor = 0;
	_przelicz = true;
}

// Zwraca wartosc srednia
int ValColl::Value() {
	if (!_przelicz) return _value;
	int suma = 0;
	for (int i = 0; i < _elements; i++)
		suma += arr[i];
	_value = suma / _elements;
	_przelicz = false;
	return _value;
}



