#!/usr/bin/env python

#Converts a dictionary to a string in the form 'key1:type1:value1;key2:type2:value2;'
def toString(dictionary):
    string = ''
    typeCode = ''
    for key in dictionary:
        valueType = type(dictionary[key])

        if valueType is int:
            typeCode = 'i'
        elif valueType is float:
            typeCode = 'f'
        elif valueType is long:
            typeCode = 'l'
        elif valueType is bool:
            typeCode = 'b'
        elif valueType is None:
            typeCode = 'n'
        else:
            typeCode = 's'

        string += '{}:{}:{};'.format(key, typeCode, str(dictionary[key]))

    return string

#Converts a string of the form 'key1:type1:value1;key2:type2:value2;' to a dictionary of string keys and float values
def toDict(string):
    dictionary = {}
    pairs = string.split(';')
    del pairs[-1]
		for word in pairs:
			keyValue = word.split(':')
			typeCode = keyValue[1]

			if typeCode == 'i':
				dictionary[keyValue[0]] = int(keyValue[2])
			elif typeCode == 'f':
				dictionary[keyValue[0]] = float(keyValue[2])
			elif typeCode == 'l':
				dictionary[keyValue[0]] = long(keyValue[2])
			elif typeCode == 'b':
				dictionary[keyValue[0]] = bool(keyValue[2])
			elif typeCode == 'n':
				dictionary[keyValue[0]] = None
			else:
				dictionary[keyValue[0]] = keyValue[2]

    return dictionary
