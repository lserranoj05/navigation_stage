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

        # Variables para controlar la lógica de selección de objeto
        self.object_selection_enabled = False
        self.selected_object = None

        # Definir los destinos según las combinaciones de dedos extendidos y sus coordenadas
        '''
        # Coordenadas para el mapa de la EPS
        "thumb": (-11.48, 7.4),                 # Baño
        "index": (-8.40, -13.5),                # Recepción
        "middle": (-11.9190034866333, 3.212221145629883),  # Pasillo Principal
        "thumb_index": (2.31, -5.42),  # Lab Robótica
        "index_middle": (4.51, -0.58),   # Lab Electrónica
        '''
        self.goals_to_coords = {
            # Coordenadas para el mapa del hospital
            "Entrada": (-5.65, -6.97),                 # Thumb
            "Información": (-13.04, -4.10),                # Index
            "Sala de espera": (-5.50, 0.70),                 # middle
            "Urgencias": (6.24, 5.19),             # thumb_index
            "Cafetería": (7.63, -4.29),           # index_middle
        }

        # Nombres de los destinos
        '''
        # Nombres para el mapa de la EPS
        "thumb": "Baño",
        "index": "Recepción",
        "middle": "Pasillo Principal",
        "thumb_index": "Lab Robótica",
        "index_middle": "Lab Electrónica"
        '''
        self.goal_names = {
            # Nombres para el mapa del hospital
            "thumb": "Entrada",
            "index": "Información",
            "middle": "Sala de espera",
            "thumb_index": "Urgencias",
            "index_middle": "Cafetería",
        }

        # Objetos disponibles y sus gestos asociados
        self.objects_to_gestures = {
            "comida": {"thumb": True, "index": False, "middle": False, "ring": False, "pinky": False},
            "agua": {"thumb": False, "index": True, "middle": False, "ring": False, "pinky": False},
            "enfermera": {"thumb": False, "index": True, "middle": True, "ring": False, "pinky": False},
        }

        # Variables de control de tiempo y gestos
        self.last_capture_time = time.time()
        self.gesture_count = 0
        self.current_gesture = None

        # Cliente de ROS
        self.client = cliente
    
    def go_to_goal(self, destino):
        """
        Muestra el nombre del destino al que se movería el robot.
        """

        print(f"Moviéndose al destino: {destino}")
        # Obtener coordenadas del destino
        coordenadas = self.goals_to_coords.get(destino, None)
        if coordenadas:
            x,y = coordenadas
            send_goal(self.client, x, y, 1)
            

    def detect_extended_fingers(self, landmarks):
        """
        Detecta si los dedos están extendidos en función de las coordenadas de los puntos clave.
        """
        fingers_extended = {
            "thumb": landmarks[4].x > landmarks[2].x,
            "index": landmarks[8].y < landmarks[6].y,
            "middle": landmarks[12].y < landmarks[10].y,
            "ring": landmarks[16].y < landmarks[14].y,
            "pinky": landmarks[20].y < landmarks[18].y,
        }
        return fingers_extended

    def interpretar_gestos(self):
        """
        Inicia la detección de gestos manuales usando la cámara.
        """
        cap = cv2.VideoCapture(0)

        # Inicialización auxiliar
        last_destino = None
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Control de la frecuencia de captura a 1 Hz
            current_time = time.time()
            if current_time - self.last_capture_time >= 1.0:
                self.last_capture_time = current_time

                # Convertir la imagen a RGB para MediaPipe
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.hands.process(rgb_frame)

                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        # Dibuja las marcas en la mano detectada
                        self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                        # Detecta qué dedos están extendidos
                        fingers_extended = self.detect_extended_fingers(hand_landmarks.landmark)

                        # Salir si los cinco dedos están extendidos
                        if all(fingers_extended.values()):
                            print("Todos los dedos extendidos detectados. Saliendo de la aplicación.")
                            cap.release()
                            cv2.destroyAllWindows()
                            return

                        # Detectar el gesto de habilitación de objetos
                        if fingers_extended["thumb"] and not fingers_extended["index"] and not fingers_extended["middle"] and not fingers_extended["ring"] and fingers_extended["pinky"]:
                            print("Gesto de habilitación de objetos detectado.")
                            self.object_selection_enabled = not self.object_selection_enabled
                            if self.object_selection_enabled:
                                print("Modo de selección de objeto activado.")
                            else:
                                print("Modo de selección de destino activado.")
                            continue

                        # Si estamos en modo de selección de objeto, reconocer solo gestos de objetos
                        if self.object_selection_enabled:
                            for obj, gesture in self.objects_to_gestures.items():
                                if fingers_extended == gesture:
                                    print(f"Objeto seleccionado: {obj}")
                                    self.selected_object = obj
                                    # Mantener habilitado el modo de selección de objeto hasta que se repita el gesto de habilitación
                                    break
                                # Salir si los cinco dedos están extendidos
                                if all(fingers_extended.values()):
                                    print("Todos los dedos extendidos detectados. Saliendo de la aplicación.")
                                    cap.release()
                                    cv2.destroyAllWindows()
                                    break
                            continue

                        # Si no estamos en modo de selección de objeto, reconocer gestos de destino
                        if not self.object_selection_enabled:
                            #destino = None
                            if fingers_extended["thumb"] and not fingers_extended["index"]:
                                destino = self.goal_names["thumb"]
                            elif fingers_extended["index"] and not fingers_extended["thumb"] and not fingers_extended["middle"]:
                                destino = self.goal_names["index"]
                            elif fingers_extended["middle"] and not fingers_extended["index"]:
                                destino = self.goal_names["middle"]
                            elif fingers_extended["thumb"] and fingers_extended["index"] and not fingers_extended["middle"]:
                                destino = self.goal_names["thumb_index"]
                            elif fingers_extended["index"] and fingers_extended["middle"] and not fingers_extended["thumb"]:
                                destino = self.goal_names["index_middle"]

                            if destino != last_destino:
                                print(f"Gesto detectado - Moviéndose al destino: {destino}")
                                self.go_to_goal(destino)

                            last_destino = destino

            ## Mostrar texto en la ventana según el modo
            if self.object_selection_enabled:
                text = f"Cargando: {self.selected_object}" if self.selected_object else "Modo de seleccion de objeto"
            else:
                text = "Modo de seleccion de destino"


            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            font_thickness = 1
            color_text = (255, 255, 255)
            color_background = (0, 0, 0)
            padding = 10

            (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, font_thickness)
            top_left_corner = (10, 10)
            bottom_right_corner = (top_left_corner[0] + text_width + padding * 2,
                                   top_left_corner[1] + text_height + padding * 2)
            cv2.rectangle(frame, top_left_corner, bottom_right_corner, color_background, -1)
            text_position = (top_left_corner[0] + padding, top_left_corner[1] + text_height + padding - 2)
            cv2.putText(frame, text, text_position, font, font_scale, color_text, font_thickness)

            # Mostrar el video
            cv2.imshow('Hand Gesture Control', frame)

            # Salir con la tecla 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
