import subprocess
import time
import signal
import sys
import cv2
import os
import pika
import base64

IMAGE_DESEADA_PATH = 'img/imgDeseada.png'

def imagenDeseada():
    cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame from camera.")
        cap.release()
        return

    try:
        if os.path.exists('img/imgDeseada.png'):
            os.remove('img/imgDeseada.png')
        cv2.imwrite('img/imgDeseada.png', frame)
    except Exception as e:
        print(f"Error saving image: {e}")
    finally:
        cap.release()

def publicarImagenDeseada(imgPath):
    connection = None
    try:
        #Iniciar connection sincrona (bloqueante) con RabbitMQ en localhost. 
        connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        channel = connection.channel()
        # Declaramos la cola imagenActual, si no existe, la creamos.
        channel.queue_declare(queue='imagenDeseada')  
        # Convertimos el objeto imagen a base64, para poder mandar un string
        converted_string = encodeImage(imgPath)

        if converted_string is None:
            print("Error: Could not encode image, not publishing.")
            return
        
        # Publicamos la imagen en la cola imagenActual
        channel.basic_publish(exchange='', routing_key='imagenActual', body=converted_string)
        print("Sent imagenActual to Queue")

    except pika.exceptions.AMQPConnectionError as e:
        print(f"Error: Failed to connect to RabbitMQ: {e}")
    except Exception as e:
        print(f"Error: An unexpected error occurred: {e}")
    finally:
        if connection and connection.is_open:
            # Cerramos la conexión
            connection.close()

def encodeImage(imgPath):
    try:
        with open(imgPath, "rb") as image2string:
            converted_string = base64.b64encode(image2string.read())
        return converted_string
    except FileNotFoundError:
        print(f"Error: Image file not found at {imgPath}")
        return None
    except Exception as e:
        print(f"Error encoding image: {e}")
        return None

def signal_handler(sig, frame):
    print('Exiting main.py')
    sys.exit(0)

def arranca():
    signal.signal(signal.SIGINT, signal_handler)
    print('Press Ctrl+C to exit main.py')

    try:
        # --- Limpieza de archivos de la ejecución anterior ---
        # Borra el archivo de datos de error si existe para empezar desde cero.
        error_file_path = 'img/error_data.txt'
        if os.path.exists(error_file_path):
            os.remove(error_file_path)
            print(f"Archivo de error anterior '{error_file_path}' eliminado.")

        #Obtenemos la imágen deseada
        imagenDeseada()
        #Publicamos la imágen deseada
        #publicarImagenDeseada(IMAGE_DESEADA_PATH)

        # Correremos los 3 programas concurrentemente 
        # Start lecturaImagenActual.py
        lectura_process = subprocess.Popen(["python", "lecturaImagenActual.py"])

        # Start controladorImagen.py
        controlador_process = subprocess.Popen(["python", "controladorImagen.py"])

        # Mantener main.py corriendo, hasta que sea interrumpido
        while True:
            time.sleep(0.001)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Terminar todos los procesos antes de salir
        if 'lectura_process' in locals() and lectura_process.poll() is None:
            lectura_process.terminate()
            lectura_process.wait()
        if 'controlador_process' in locals() and controlador_process.poll() is None:
            controlador_process.terminate()
            controlador_process.wait()

if __name__ == "__main__":
    arranca()
