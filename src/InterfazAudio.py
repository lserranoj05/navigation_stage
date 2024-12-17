#!/usr/bin/env python3

# Reconocimiento de voz
import sounddevice
import speech_recognition as sr
from enum import Enum

import rospy
from std_msgs.msg import String
from set_goal import send_goal

class Posicion:
    def __init__(self, nombre, x, y):
        self.nombre = nombre
        self.x = x
        self.y = y

class InterfazAudio:
    '''
    # Posiciones para el mapa de la EPS
    __POSICIONES = [
        Posicion("baño", -11.48, 7.4),
        Posicion("clase", -8.40, -13.5),
        Posicion("robótica", 2.31, -5.42),
        Posicion("electrónica", 4.51, -0.58)
    ]
    '''
    # Posiciones para el mapa del hospital
    __POSICIONES = [
        Posicion("entrada", -5.65, -6.97),
        Posicion("información", -13.04, -4.10),
        Posicion("espera", -5.50, 0.70),
        Posicion("urgencias", 6.24, 5.19),
        Posicion("cafetería", 7.63, -4.29)
    ]

    __PALABRA_CLAVE = "llévame"


    def __init__(self,cliente, device_index=0):
        self.__microphone = sr.Microphone(device_index=device_index)
        self.__recognizer = sr.Recognizer()
        self.client = cliente

    def escuchar(self, segundos_de_espera=3):
        with self.__microphone as source:
            print("Escuchando...")
            audio = self.__recognizer.listen(source, segundos_de_espera, 3)
            print("Procesando...")

            text_result = self.__recognizer.recognize_google(audio, language='es-ES')
            print(f"Texto escuchado: {text_result}")
        return text_result

    def escucharAccion(self, segundos_de_espera=3):
        texto = self.escuchar(segundos_de_espera=segundos_de_espera)
        posicion_objetivo = self.__procesar_texto(texto.split())
        return posicion_objetivo

    def __procesar_texto(self, texto):
        ir = False
        posicion_objetivo = None
        for palabra in texto:
            if palabra == self.__PALABRA_CLAVE and ir==False:
                ir = True
            elif ir==True:
                for pose in self.__POSICIONES:
                    if palabra == pose.nombre :
                        posicion_objetivo = pose
                        break
        
        if posicion_objetivo:
            print(f"Te llevo a {posicion_objetivo.nombre}")
            send_goal(self.client, posicion_objetivo.x, posicion_objetivo.y, 1)
            return posicion_objetivo
        else:
            print("No se han detectado acciones")
            return None