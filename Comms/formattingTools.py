#!/usr/bin/env python

#Converts a dictionary to a string in the form 'key1:value1;key2:value2;'
def toString(dictionary):
    string = ''
    for key in dictionary:
        string += '{}:{};'.format(key, str(dictionary[key]))
    return string

#Converts a string of the form 'key1:value1;key2:value2;' to a dictionary of string keys and float values
def toDict(string):
    dictionary = {}
    pairs = string.split(';')
    del pairs[-1]
    for word in pairs:
        keyValue = word.split(':')
        dictionary[keyValue[0]] = float(keyValue[1])
    return dictionary
