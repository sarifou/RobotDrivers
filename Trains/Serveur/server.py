#!/usr/bin/env python3.7
#coding:utf-8

import socket
import threading
import json
from speedlib.trains import dcc
from speedlib.trains.dcc import Train

dict_train = {} # Dictionnaire des trains
MAX_NUMBER = 5

for i in range(1, MAX_NUMBER):
    dict_train[i] = Train("DCC"+str(i), i)

dcc.start() # Initialisation du controleur DCC

class Thread(threading.Thread):
    """
    Permet d'exécuter un processus lorsqu'un client se connecte et envoi une commande
    """

    def __init__(self, client, address):
        """Fonction pour initialiser les variables de la classe"""
        threading.Thread.__init__(self)
        self.client = client #
        self.address = address #

    def run(self):
        """ Fonction qui exécute le processus client """
        data = self.client.recv(1024) # Reception des données
        data = data.decode("utf8") # Encodage au format uft8
        dict_data = json.loads(data) # Transformation au format dictionnaire
        if "value" in dict_data:
            self.command_with_value(dict_data["train"], dict_data["value"])
        elif "state" in dict_data:
            self.command_with_state(dict_data["train"], dict_data["command"], dict_data["state"]) 
        else:
            self.command_simple(dict_data["train"], dict_data["command"])
        self.client.close() # Fermeture de la communication avec le client
        
    def command_simple(self, number, command):
        """ Fonction pour exécuter les mouvements de base du train"""
        if command == "faster":
            dict_train[number].faster()
            self.send_response("Fin")
        if command == "slower":
            dict_train[number].slower()
            self.send_response("Fin")
        if command == "reverse":
            dict_train[number].reverse()
            self.send_response("Fin")
        if command == "stop":
            dict_train[number].speed = 0
            self.send_response("Fin")

    def command_with_value(self, number, value):
        """Focntion pour les commandes avec valeur"""
        dict_train[number].speed = value
        self.send_response("Fin")

    def command_with_state(self, number, command, state):
        """Fonction pour commander les accessoires des trains"""
        if command == "fl":
            dict_train[number].fl = state
            self.send_response("Fin")
        elif command == "f1":
            dict_train[number].f1 = state
            self.send_response("Fin")
        elif command == "f2":
            dict_train[number].f2 = state
            self.send_response("Fin")
        elif command == "f3":
            dict_train[number].f3 = state
            self.send_response("Fin")
        elif command == "f4":
            dict_train[number].f4 = state
            self.send_response("Fin")

    def send_response(self, message):
        """Fonction qui renvoie une reponse au client"""
        encoded_resp = message.encode() # encodage du message au format binaire
        self.client.sendall(encoded_resp) # envoi du message

HOST, PORT = ('', 5020)
SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
SOCKET.bind((HOST, PORT))
print("Le serveur est initialisé")
while True:
    SOCKET.listen(5)
    CLIENT, ADDRESS = SOCKET.accept()
    print("Un client vient de se connecter {}" .format(ADDRESS))
    MY_THREAD = Thread(CLIENT, ADDRESS) # Instanciation d'un Thread de traitement
    MY_THREAD.start() # Lancement de la séquence du Thread

dcc.stop()
SOCKET.close() # Fermeture de la communication serveur