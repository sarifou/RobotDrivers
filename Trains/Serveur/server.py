#!/usr/bin/env python3.7
#coding:utf-8

import socket
import threading
import json
from speedlib.trains import dcc
from speedlib.trains.dcc import Train

train_1 = Train("DCC1", 1)
train_2 = Train("DCC2", 2)
train_3 = Train("DCC3", 3)
train_4 = Train("DCC4", 4)

dict_train = {1 : train_1, 2 : train_2, 3: train_3, 4: train_4}

dcc.start()

class thread(threading.Thread) :

    def __init__(self, client, address) :
        
        threading.Thread.__init__(self)
        
        self.client = client
        
        self.address = address

    def run(self):
        data = self.client.recv(1024)
        
        data = data.decode("utf8")

        dict_data = json.loads(data)
        
        value=0
        state = False
        
        if dict_data["value"] :
            value = dict_data["value"]
        if dict_data["state"] :
            state = dict_data["state"]
            
        self.command(dict_data["train"] ,dict_data["command"], value, state)
        
        client.close()
        
        print("Fin de la communication ")
        
    def command(self, number, command, value=0, state=False) :
        if command == "faster" :
            dict_train[number].faster()
            self.response("Le train {} avance".format(number))
            print(dict_train[number])
        elif command == "slower":
            dict_train[number].slower()
            self.response("Le train {} ralentit".format(number))
        elif command == "reverse":
            dict_train[number].reverse()
            self.response("Le train {} change de direction".format(number))
        elif command == "stop":
            dict_train[number].speed=0
            self.response("Le train {} s'est arrêté".format(number))
        elif command=="speed":
            dict_train[number].speed = value
            self.response("La vitesse du train {} est maintenant {}".format(number, value))
            
    def response(self, message):
        
        encoded_resp = message.encode()
        
        client.sendall(encoded_resp)

host, port = ('', 5556)

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

socket.bind((host, port))

print("Le serveur est initialisé")

while True:
    socket.listen(5)
    client, address = socket.accept()

    print("Un client vient de se connecter {}" .format(address))

    my_thread = thread(client, address)
    my_thread.start()


dcc.stop()
socket.close()