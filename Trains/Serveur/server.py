#!/usr/bin/env python3.7
#coding:utf-8

import socket
import threading
from speedlib.trains import dcc
from speedlib.trains.dcc import Train

class thread(threading.Thread) :

    def __init__(self, connect) :
        threading.Thread.__init__(self)
        self.connect = connect

    def run(self):
        data = self.connect.recv(1024)
        ##data = data.decode("utf8")
        print(data)

host, port = ('', 9600)

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

socket.bind((host, port))

print("Le serveur est initialisé")

while True:
    socket.listen(5)
    connection, address = socket.accept()

    print("Un client vient de se connecter")

    my_thread = thread(connection)
    my_thread.start()

connection.close()
socket.close()
