#!/usr/bin/env python

import socket as sock


def toDict(string):
    dict = {}
    pairs = string.split(';')
    del pairs[-1]
    
    for word in pairs:
        keyValue = word.split(':')
        dict[keyValue[0]] = float(keyValue[1])
    return dict

def main():
    s = sock.socket(sock.AF_INET, sock.SOCK_DGRAM)
    s.bind(("", 12000))
    
    while True:
        message, address = s.recvfrom(1024)
        data = toDict(message)
        print(data)
        


if __name__ == '__main__':
    main()
