#!/usr/bin/env python3
from set_goal import get_client, send_goal
import rospy
from std_msgs.msg import String
import time

import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
from tkinter import PhotoImage
from InterfazAudio import InterfazAudio
from InterfazVision import InterfazVision

# Función para mostrar el menú inicial
def mostrar_inicio():
    # Limpiar la ventana
    for widget in root.winfo_children():
        widget.destroy()

    # Crear la pantalla inicial
    welcome_label = tk.Label(root, text="INTERFAZ DE CONTROL TURTLEBOT\n\nBIENVENIDO", 
                              font=("Helvetica", 22, "bold"), bg="#FFFFFF", fg="#333333", wraplength=480, justify="center")
    welcome_label.pack(pady=100)

    # Crear un marco inferior para los botones
    bottom_frame = tk.Frame(root, bg="#FFFFFF")
    bottom_frame.pack(side="bottom", fill="x", pady=20)

    # Botón "Siguiente"
    next_button = tk.Button(bottom_frame, text="Siguiente", command=mostrar_menu, bg="#007BFF", fg="white", font=("Helvetica", 10, "bold"))
    next_button.pack(pady=5, fill="x", padx=50)

    # Botón "Salir"
    exit_button = tk.Button(bottom_frame, text="Salir", command=root.quit, bg="#FF5555", fg="white", font=("Helvetica", 10, "bold"))
    exit_button.pack(pady=5, fill="x", padx=50)

# Función para mostrar el menú principal
def mostrar_menu():
    # Limpiar la ventana
    for widget in root.winfo_children():
        widget.destroy()

    # Crear un marco principal para organizar widgets
    main_frame = tk.Frame(root, bg="#FFFFFF", padx=20, pady=20)
    main_frame.pack(fill="both", expand=True)

    # Label de bienvenida
    welcome_label = tk.Label(main_frame, text="MENÚ DE MANDO", font=("Helvetica", 20, "bold"), bg="#FFFFFF", fg="#333333")
    welcome_label.pack(pady=10)

    # Botón para escuchar comandos de voz
    listen_button = tk.Button(main_frame, text="Comandos de voz", command=escuchar_comando, bg="#007BFF", fg="white", font=("Helvetica", 10, "bold"))
    listen_button.pack(pady=10, fill="x", padx=50)

    # Botón para ejecutar comandos gestuales
    execute_button = tk.Button(main_frame, text="Comandos gestuales", command=vision, bg="#007BFF", fg="white", font=("Helvetica", 10, "bold"))
    execute_button.pack(pady=10, fill="x", padx=50)

    # Botón "Escoja su destino"
    destination_button = tk.Button(main_frame, text="Escoja su destino", command=mostrar_destinos, bg="#007BFF", fg="white", font=("Helvetica", 10, "bold"))
    destination_button.pack(pady=10, fill="x", padx=50)

    # Botón "Ayuda para gestos"
    gestos_help_button = tk.Button(main_frame, text="Guia de gestos", command=ayuda_gestos, bg="#f78a3b", fg="white", font=("Helvetica", 10, "italic"))
    gestos_help_button.pack(pady=10, padx=50)

    # Botón "Ayuda para voz"
    audio_help_button = tk.Button(main_frame, text="Guia de comandos de voz", command=ayuda_voz, bg="#f78a3b", fg="white", font=("Helvetica", 10, "italic"))
    audio_help_button.pack(pady=10, padx=50)

    # Crear un marco inferior para los botones "Volver a Inicio" y "Salir"
    bottom_frame = tk.Frame(root, bg="#FFFFFF")
    bottom_frame.pack(side="bottom", fill="x", pady=20)

    # Botón "Volver a Inicio"
    back_button = tk.Button(bottom_frame, text="Volver a Inicio", command=mostrar_inicio, bg="#007BFF", fg="white", font=("Helvetica", 10, "bold"))
    back_button.pack(pady=5, fill="x", padx=50)

    # Botón "Salir"
    exit_button = tk.Button(bottom_frame, text="Salir", command=root.quit, bg="#FF5555", fg="white", font=("Helvetica", 10, "bold"))
    exit_button.pack(pady=5, fill="x", padx=50)

# Función para mostrar el menú de destinos
def mostrar_destinos():
    # Limpiar la ventana
    for widget in root.winfo_children():
        widget.destroy()

    # Crear un marco principal para organizar widgets
    main_frame = tk.Frame(root, bg="#FFFFFF", padx=20, pady=20)
    main_frame.pack(fill="both", expand=True)

    # Título y primer desplegable
    destination_label = tk.Label(main_frame, text="Destino del TurtleBot:", font=("Helvetica", 14), bg="#FFFFFF", fg="#333333")
    destination_label.pack(pady=(10, 5))  # Retiramos anchor para centrado automático por pack

    global primary_dest_combo
    primary_dest_combo = ttk.Combobox(main_frame, values=["Entrada", "Informacion", "Sala de espera", "Urgencias", "Cafeteria"], state="readonly")
    primary_dest_combo.set("Seleccione un destino principal")
    primary_dest_combo.pack(pady=5, fill="x", padx=50)

    # Salto de línea y segundo desplegable
    object_label = tk.Label(main_frame, text="Objeto que transportar:", font=("Helvetica", 14), bg="#FFFFFF", fg="#333333")
    object_label.pack(pady=(20, 5))  # Retiramos anchor para centrado automático por pack

    global secondary_dest_combo
    secondary_dest_combo = ttk.Combobox(main_frame, values=["Comida", "Agua", "Enfermera"], state="readonly")
    secondary_dest_combo.set("Seleccione que desea")
    secondary_dest_combo.pack(pady=5, fill="x", padx=50)

    # Boton para ir
    go_button = tk.Button(main_frame, text="Ir", command=destino_seleccionado)
    go_button.pack()

    # Crear un marco inferior para los botones
    bottom_frame = tk.Frame(root, bg="#FFFFFF")
    bottom_frame.pack(side="bottom", fill="x", pady=20)

    # Botón "Volver al menú de mando"
    back_to_menu_button = tk.Button(bottom_frame, text="Volver al menú de mando", command=mostrar_menu, bg="#007BFF", fg="white", font=("Helvetica", 10, "bold"))
    back_to_menu_button.pack(pady=5, fill="x", padx=50)

    # Botón "Volver al menú de inicio"
    back_to_start_button = tk.Button(bottom_frame, text="Volver al menú de inicio", command=mostrar_inicio, bg="#007BFF", fg="white", font=("Helvetica", 10, "bold"))
    back_to_start_button.pack(pady=5, fill="x", padx=50)

    # Botón "Salir"
    exit_button = tk.Button(bottom_frame, text="Salir", command=root.quit, bg="#FF5555", fg="white", font=("Helvetica", 10, "bold"))
    exit_button.pack(pady=5, fill="x", padx=50)

def destino_seleccionado():
    seleccion = primary_dest_combo.get()
    objeto = secondary_dest_combo.get()
    print("Yendo a ", seleccion, " con ", objeto)

    goals_to_coords = {
        # Coordenadas para el mapa del hospital
        "Entrada": (-5.65, -6.97),                 # Thumb
        "Información": (-13.04, -4.10),                # Index
        "Sala de espera": (-5.50, 0.70),                 # middle
        "Urgencias": (6.24, 5.19),             # thumb_index
        "Cafeteria": (7.63, -4.29),           # index_middle
    }

    coordenadas = goals_to_coords.get(seleccion, None)
    if coordenadas:
        x,y = coordenadas
        send_goal(cliente, x, y, 1)


# Muestra la imagen de ayuda para los gestos
def ayuda_gestos():
    # Crear una ventana secundaria
    new_window = tk.Toplevel(root)
    new_window.title("Guía de gestos")
    new_window.geometry("620x273")  # Ajusta el tamaño según necesites
    
    # Cargar la imagen
    img = PhotoImage(file="img/AYUDA.png")  # Reemplaza con tu ruta de imagen
    
    # Mostrar la imagen en la ventana secundaria
    label = tk.Label(new_window, image=img)
    label.image = img  # Mantén una referencia para evitar garbage collection
    label.pack()


# Muestra ejemplos de frases funcionales
def ayuda_voz():
    # Limpiar la ventana
    for widget in root.winfo_children():
        widget.destroy()

    # Crear un marco principal para organizar widgets
    main_frame = tk.Frame(root, bg="#FFFFFF", padx=20, pady=20)
    main_frame.pack(fill="both", expand=True)

    # Label para el titulo
    title_label = tk.Label(main_frame, text="Lista de comandos de voz adecuados")
    title_label.pack(pady=(10, 5))

    #Labels para ejemplos
    label1 = tk.Label(main_frame, text="Llevame a...", font=("Helvetica", 20, "bold"), bg="#FFFFFF", fg="#333333")
    label1.pack(pady=10)

    label2 = tk.Label(main_frame, text="...la entrada.", font=("Helvetica", 15, "italic"), bg="#FFFFFF", fg="#333333")
    label2.pack(pady=10)
    label3 = tk.Label(main_frame, text="...la sala de espera.", font=("Helvetica", 15, "italic"), bg="#FFFFFF", fg="#333333")
    label3.pack(pady=10)
    label4 = tk.Label(main_frame, text="...por informacion.", font=("Helvetica", 15, "italic"), bg="#FFFFFF", fg="#333333")
    label4.pack(pady=10)
    label5 = tk.Label(main_frame, text="...urgencias.", font=("Helvetica", 15, "italic"), bg="#FFFFFF", fg="#333333")
    label5.pack(pady=10)
    label6 = tk.Label(main_frame, text="...la cafetería.", font=("Helvetica", 15, "italic"), bg="#FFFFFF", fg="#333333")
    label6.pack(pady=10)
    

    # Crear un marco inferior para los botones
    bottom_frame = tk.Frame(root, bg="#FFFFFF")
    bottom_frame.pack(side="bottom", fill="x", pady=20)
    # Botón "Volver al menú de mando"
    back_to_menu_button = tk.Button(bottom_frame, text="Volver al menú de mando", command=mostrar_menu, bg="#007BFF", fg="white", font=("Helvetica", 10, "bold"))
    back_to_menu_button.pack(pady=5, fill="x", padx=50)
    # Botón "Salir"
    exit_button = tk.Button(bottom_frame, text="Salir", command=root.quit, bg="#FF5555", fg="white", font=("Helvetica", 10, "bold"))
    exit_button.pack(pady=5, fill="x", padx=50)

# Funciones para los botones
def escuchar_comando():    
    try:
        accion = interfaz_audio.escucharAccion(segundos_de_espera=5)
        if accion:
            messagebox.showinfo("Destino alcanzado", f"Has llegado a: {accion.nombre}")
        else:
            messagebox.showinfo("Sin destino", "Parece que no existe el destino que buscas. Repítelo más claramente o elige otro.")
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al escuchar: {e}")

def vision():
    try:
        messagebox.showinfo("Visión", "Interpretando gestos manuales. Abriendo cámara...")
        interfaz_vision.interpretar_gestos()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al interpretar gestos: {e}")
        print(e)

# Crear la ventana principal
root = tk.Tk()
root.title("Servicio de robótica para hospitales")
root.geometry("500x500")
root.resizable(False, False)
root.configure(bg="#FFFFFF")  # Fondo blanco para la ventana principal

# Instancia del cliente para ROS

cliente = get_client()

# Inicializar la instancia de InterfazAudio
interfaz_audio = InterfazAudio(cliente, device_index=0)

# Instancia de InterfazVision
interfaz_vision = InterfazVision(cliente)

# Mostrar la pantalla inicial al inicio
mostrar_inicio()

# Iniciar el bucle principal de la aplicación
root.mainloop()
