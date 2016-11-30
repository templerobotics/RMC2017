def toString(dictionary):
    string = ''
    for key in dictionary:
        string += '{}:{};'.format(key, str(dictionary[key]))
    return string

def toDict(string):
    dictionary = {}
    pairs = string.split(';')
    del pairs[-1]
    for word in pairs:
        keyValue = word.split(':')
        dictionary[keyValue[0]] = float(keyValue[1])
    return dictionary
