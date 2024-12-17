import cv2
import mediapipe as mp
import time

from set_goal import send_goal

class InterfazVision:
    def __init__(self, cliente):
        # Inicialización de MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.client = cliente

        # Definir los destinos según las combinaciones de dedos extendidos y sus coordenadas
        self.fingers_to_goals = {
            "thumb": (-11.472, 7.4),                 # Baño
            "index": (-8.48, -13.5),                # Recepción
            "middle": (-11.9190034866333, 3.212221145629883),  # Pasillo Principal
            "thumb_index": (2.31, -5.42),  # Lab Robótica
            "index_middle": (4.51, -0.58),   # Lab Electrónica
        }

        # Nombres de los destinos
        self.goal_names = {
            "thumb": "Baño",
            "index": "Recepción",
            "middle": "Pasillo Principal",
            "thumb_index": "Lab Robótica",
            "index_middle": "Lab Electrónica"
        }

        self.destino_seleccionado = None  # Estado para saber si ya se eligió un destino

        # Variables de control de tiempo para la captura y validación de gestos
        self.last_capture_time = time.time()  # Tiempo de la última captura
        self.gesture_count = 0  # Contador para la validación del gesto
        self.current_gesture = None  # Gesto actual detectado

    def go_to_goal(self, destino):
        """
        Muestra el nombre del destino al que se movería el robot.
        """
        print(f"Moviéndose al destino: {destino}")
        if destino == "Baño":
            x = -11.472
            y = 7.4
        elif destino == "Recepción":
            x = -8.48
            y = -13.5
        elif destino == "Pasillo Principal":
            x = -8.48
            y = -13.5
        elif destino == "Lab Robótica":
            x = 2.31
            y = -5.42
        elif destino == "Lab Electrónica":
            x = 4.51
            y = -0.58

        send_goal(self.client, x, y, 1)

    def detect_extended_fingers(self, landmarks):
        """
        Detecta si los dedos están extendidos en función de las coordenadas de los puntos clave.
        """
        fingers_extended = {
            "thumb": False,
            "index": False,
            "middle": False,
            "ring": False,
            "pinky": False
        }
        # Determina si cada dedo está extendido basado en sus puntos clave
        fingers_extended["thumb"] = landmarks[4].x > landmarks[2].x
        fingers_extended["index"] = landmarks[8].y < landmarks[6].y
        fingers_extended["middle"] = landmarks[12].y < landmarks[10].y
        fingers_extended["ring"] = landmarks[16].y < landmarks[14].y
        fingers_extended["pinky"] = landmarks[20].y < landmarks[18].y
        return fingers_extended

    def interpretar_gestos(self):
        """
        Inicia la detección de gestos manuales usando la cámara.
        """
        cap = cv2.VideoCapture(0)

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            # Control de la frecuencia de captura a 1 Hz
            current_time = time.time()
            if current_time - self.last_capture_time >= 1.0:  # Si ha pasado 1 segundo
                self.last_capture_time = current_time

                # Convertir la imagen a RGB para MediaPipe
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.hands.process(rgb_frame)

                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        # Dibuja las marcas en la mano detectada
                        self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                        # Detecta cuáles dedos están extendidos
                        fingers_extended = self.detect_extended_fingers(hand_landmarks.landmark)

                        # Determinar el destino según los dedos extendidos
                        if fingers_extended["thumb"] and not fingers_extended["index"]:
                            destino = self.fingers_to_goals["thumb"]
                            destino_nombre = self.goal_names["thumb"]
                        elif fingers_extended["index"] and not fingers_extended["thumb"] and not fingers_extended["middle"]:
                            destino = self.fingers_to_goals["index"]
                            destino_nombre = self.goal_names["index"]
                        elif fingers_extended["middle"] and not fingers_extended["index"]:
                            destino = self.fingers_to_goals["middle"]
                            destino_nombre = self.goal_names["middle"]
                        elif fingers_extended["thumb"] and fingers_extended["index"] and not fingers_extended["middle"]:
                            destino = self.fingers_to_goals["thumb_index"]
                            destino_nombre = self.goal_names["thumb_index"]
                        elif fingers_extended["index"] and fingers_extended["middle"] and not fingers_extended["thumb"]:
                            destino = self.fingers_to_goals["index_middle"]
                            destino_nombre = self.goal_names["index_middle"]
                        else:
                            destino = None  # No se detectó un gesto válido

                        if destino:
                            # Verificar si el gesto es válido por 2 segundos (2 detecciones consecutivas)
                            if self.current_gesture == destino_nombre:
                                self.gesture_count += 1
                            else:
                                self.current_gesture = destino_nombre
                                self.gesture_count = 1  # Reiniciar contador si cambia el gesto

                            # Si se ha detectado el mismo gesto durante 2 segundos, es válido
                            if self.gesture_count >= 2:
                                print(f"Gesto detectado - Moviéndose al destino: {destino_nombre}")
                                self.go_to_goal(destino_nombre)  # Mostrar el nombre del destino
                                self.destino_seleccionado = destino
                                self.gesture_count = 0  # Reiniciar contador
                                break  # Mueve el robot solo una vez por ciclo

            # Muestra el video en tiempo real
            cv2.imshow('Hand Gesture Control', frame)
            
            # Salir con la tecla 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
