import cv2
import os
import pika
import base64
import time
import signal
import sys

# Constantes
IMAGE_ACTUAL_PATH = 'img/imgActual.png'
PUBLISH_INTERVAL = 0.0001  # Seconds

# Método para codificar la imagen en base 64, para poder mandarla a RabbitMQ como un String (text based API)
# Asi podemos representar una imagen bonaria como un string y manejarlo con el sistema de colas. 
# Base64 podría usarse con compresión, aunque no lo haremos
# Base64 convierte imagen (binario) a formato ASCII
# Agrega overhead 33% increase (6 bits enconded into 8 bits)
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


# Método para leer un solo frame o imagen actual que observa la cámara
def lecturaImagenActual():
    # Iniciamos la captura de video dando el indice de la camara que depende del puerto y es dependeinte de windows
    cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return None

    # Leemos un solo frame de la cámara. Ret (bool) nos dice si hay imagen o no y frame nos da el array de la imagen
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Could not read frame from camera.")
        cap.release()
        return None

    try:
        if os.path.exists('img/imgActual.png'):
            os.remove('img/imgActual.png')
        cv2.imwrite('img/imgActual.png', frame)
    except Exception as e:
        print(f"Error saving image: {e}")
        frame = None
    finally:
        cap.release()
    return frame

# Método para publicar la imagen actual que observa la cámara en RabbitMQ. Queue=imagenActual
def publicarImagenActual(imgPath):
    connection = None
    try:
        #Iniciar connection sincrona (bloqueante) con RabbitMQ en localhost. 
        connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        channel = connection.channel()
        # Declaramos la cola imagenActual, si no existe, la creamos.
        channel.queue_declare(queue='imagenActual')  
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

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    print('Press Ctrl+C to exit')
    while True:
        lecturaImagenActual()
        publicarImagenActual(IMAGE_ACTUAL_PATH)
        time.sleep(PUBLISH_INTERVAL)

if __name__ == '__main__':
    main()