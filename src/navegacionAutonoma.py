# -*- coding: utf-8 -*-
# from __future__ import print_function

# Importación de librerías
import rospy
import smach_ros
import math
import actionlib
import sys
import speech_recognition as sr
from enum import Enum
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

# Renombrar tópicos para mejor visibilidad
TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = '/base_scan'

"""class Preguntar(State):
    def __init__(self):
        State.__init__(self, outcomes=['ROB', 'ELEC', 'ENTRADA', 'AUTO', 'REC', 'ERROR'])
        self.base = False

    def execute(self, userdata):
        llamadaVoz = raw_input("¿Adonde quiere ir? ").strip().upper()
        if llamadaVoz == 'ROBOTICA' or llamadaVoz == 'robotica':
            return 'ROB'
        elif llamadaVoz == 'ELECTRONICA' or llamadaVoz == 'electronica':
            return 'ELEC'
        elif llamadaVoz == 'ENTRADA' or llamadaVoz == 'entrada':
            return 'ENTRADA'
        elif llamadaVoz == 'AUTOMATICA' or llamadaVoz == 'automatica':
            return 'AUTO'
        elif llamadaVoz == 'RECEPCION' or llamadaVoz == 'recepcion':
            return 'REC'
        else:
            return 'ERROR'"""
        
class GoToROB(State):
    def __init__(self):
        State.__init__(self, outcomes=['tROB'])
        self.base = False
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 12
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        state = self.client.get_state()

        while state == GoalStatus.ACTIVE or state == GoalStatus.PENDING:
            rospy.Rate(10)
            state = self.client.get_state()

        return 'tROB'

class GoToELEC(State):
    def __init__(self):
        State.__init__(self, outcomes=['tELEC'])
        self.base = False
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 2.5
        goal.target_pose.pose.position.y = 0.5
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        state = self.client.get_state()

        while state == GoalStatus.ACTIVE or state == GoalStatus.PENDING:
            rospy.Rate(10)
            state = self.client.get_state()

        return 'tELEC'

class GoToENTRADA(State):
    def __init__(self):
        State.__init__(self, outcomes=['tENTRADA'])
        self.base = False
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 13
        goal.target_pose.pose.position.y = -9.5
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        state = self.client.get_state()

        while state == GoalStatus.ACTIVE or state == GoalStatus.PENDING:
            rospy.Rate(10)
            state = self.client.get_state()

        return 'tENTRADA'


class InterfazAudio(State):
    class Acciones(Enum):
        ENTRADA = "ENTRADA"
        RECEPCION = "REC"
        LAB_ROBOTICA = "ROB"
        LAB_AUTOMATICA = "AUTO"
        LAB_ELECTRONICA = "ELEC"
        ERROR = "ERROR"

    __PALABRA_CLAVE = "llévame"


    def __init__(self,State, device_index=0):
        self.__microphone: sr.Microphone = sr.Microphone(device_index=device_index)
        self.__recognizer: sr.Recognizer = sr.Recognizer()
        State.__init__(self, outcomes=['ROB', 'ELEC', 'ENTRADA', 'AUTO', 'REC', 'ERROR'])
        self.base = False

    def escuchar(self, segundos_de_espera=3) -> str:
        with self.__microphone as source:
            print("Escuchando...")
            audio = self.__recognizer.listen(source, timeout=segundos_de_espera)
            print("Procesando...")
            text_result: str = self.__recognizer.recognize_google(audio, language='es-ES')
            print(f"Texto escuchado: {text_result}")
        return text_result

    def escucharAccion(self, segundos_de_espera=3) -> Acciones:
        texto: str = self.escuchar(segundos_de_espera=segundos_de_espera)
        accion: InterfazAudio.Acciones = self.__procesar_texto(texto.split())
        return accion

    def __procesar_texto(self, texto) -> Acciones:
        ir: bool = False
        accion: Optional[InterfazAudio.Acciones] = None
        for palabra in texto:
            if palabra == self.__PALABRA_CLAVE and ir==False:
                ir = True
            elif palabra == InterfazAudio.Acciones.ENTRADA.value and ir==True:
                accion = InterfazAudio.Acciones.ENTRADA
            elif palabra == InterfazAudio.Acciones.RECEPCION.value and ir==True:
                accion = InterfazAudio.Acciones.RECEPCION
            elif palabra == InterfazAudio.Acciones.LAB_ROBOTICA.value and ir==True:
                accion = InterfazAudio.Acciones.LAB_ROBOTICA
            elif palabra == InterfazAudio.Acciones.LAB_AUTOMATICA.value and ir==True:
                accion = InterfazAudio.Acciones.LAB_AUTOMATICA
            elif palabra == InterfazAudio.Acciones.LAB_ELECTRONICA.value and ir==True:
                accion = InterfazAudio.Acciones.LAB_ELECTRONICA
            
        
        if accion:
            print(f"Te llevo a {accion.value}")
            return accion
        else:
            print("No se han detectado acciones")
            return InterfazAudio.Acciones.ERROR

class GoToAUTO(State):
    def __init__(self):
        State.__init__(self, outcomes=['tAUTO'])
        self.base = False
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 4
        goal.target_pose.pose.position.y = 18
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        state = self.client.get_state()

        while state == GoalStatus.ACTIVE or state == GoalStatus.PENDING:
            rospy.Rate(10)
            state = self.client.get_state()

        return 'tAUTO'

class GoToREC(State):
    def __init__(self):
        State.__init__(self, outcomes=['tREC'])
        self.base = False
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 23.5
        goal.target_pose.pose.position.y = 13.5
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        state = self.client.get_state()

        while state == GoalStatus.ACTIVE or state == GoalStatus.PENDING:
            rospy.Rate(10)
            state = self.client.get_state()

        return 'tREC'

if __name__ == '__main__':
    rospy.init_node("practica3")
    sm = StateMachine(outcomes=['stop'])
    with sm:
        StateMachine.add('InterfazAudio', InterfazAudio(), transitions={'ROB': 'GoToROB', 
                                'ELEC': 'GoToELEC', 
                                'ENTRADA': 'GoToENTRADA',
                                'REC': 'GoToREC',
                                'AUTO': 'GoToAUTO',
                                'ERROR': 'stop'})
        StateMachine.add('GoToROB', GoToROB(), transitions={'tROB': 'Preguntar'})
        StateMachine.add('GoToELEC', GoToELEC(), transitions={'tELEC': 'Preguntar'})
        StateMachine.add('GoToENTRADA', GoToENTRADA(), transitions={'tENTRADA': 'Preguntar'})
        StateMachine.add('GoToAUTO', GoToAUTO(), transitions={'tAUTO': 'Preguntar'})
        StateMachine.add('GoToREC', GoToREC(), transitions={'tREC': 'Preguntar'})
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()
