import pika
import sys
import os
import cv2
import numpy as np
import base64
import math

# Yellow
b_yellow = 0
g_yellow = 156
r_yellow = 154

# Green
b_green = 67
g_green = 120
r_green = 77

# Blue
b_blue = 206
g_blue = 148
r_blue = 94


def consumirImagenActual():
    print("Centroid Deseado:")
    centroid_deseada, error_deseada = find_y_centroid('img/imgDeseada.png', (r_yellow, g_yellow, b_yellow))
    # Iniciar connection sincrona (bloqueante) con RabbitMQ en localhost.
    connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
    channel = connection.channel()

    # Nombre de la cola a cual conectarnos. El producer mandará aqui la imágen.
    channel.queue_declare(queue='imagenActual')

    # Funcion que se correrá cada vez que recibamos una imagen en la cola "body"
    def callback(ch, method, properties, body):
        try:
            if os.path.exists('img/imgRecuperada.png'):
                os.remove('img/imgRecuperada.png')
        except FileNotFoundError:
            print("No image to delete")
        except Exception as e:
            print(f"An error occurred while deleting the image: {e}")

        # Guardamos la imágen en formato binario en una variable temporal encode.bin
        try:
            with open('encode.bin', "wb") as file:
                file.write(body)

            # Creamos un archivo png vacio y hacemos un decoding base64 para guardar la imagen decodeada
            with open('encode.bin', 'rb') as file:
                byte = file.read()

            with open('img/imgRecuperada.png', 'wb') as decodedImg:
                decodedImg.write(base64.b64decode((byte)))

            print(" [x] Received ")
            # Call find_y_centroid and handle potential errors
            try:
                centroid_actual, error_actual = find_y_centroid('img/imgRecuperada.png', (r_yellow, g_yellow, b_yellow))
                #centroid_deseada, error_deseada = find_y_centroid('img/imgDeseada.png', (r_yellow, g_yellow, b_yellow))
                if centroid_actual and centroid_deseada:
                    publicarError(centroid_actual, centroid_deseada)
                else:
                    if error_actual:
                        print(f"Error with actual image: {error_actual}")
                    if error_deseada:
                        print(f"Error with desired image: {error_deseada}")

            except Exception as e:
                print(f"Error in find_y_centroid or publicarError: {e}")

            ch.basic_ack(delivery_tag=method.delivery_tag)
        except Exception as e:
            print(f"An error occurred while processing the image: {e}")
            ch.basic_nack(delivery_tag=method.delivery_tag, requeue=False)

    # Comenzamos a escuchar la cola de Rabbit MQ para procesar la imágen.
    channel.basic_consume(queue='imagenActual', on_message_callback=callback, auto_ack=False)

    print(' [*] Waiting for messages. To exit press CTRL+C')
    try:
        channel.start_consuming()
    except KeyboardInterrupt:
        channel.stop_consuming()
    finally:
        connection.close()


def intrinsicCamaraMatrix():
    diagFOV = 59
    hFOV = 51.3
    vFOV = 35.5
    resH = 640
    resV = 480

    u0 = resH / 2
    v0 = resV / 2
    fx = u0 / math.tan(math.radians(hFOV / 2))
    fy = v0 / math.tan(math.radians(vFOV / 2))

    matrix = np.matrix([
        [fx, 0, u0],
        [0, fy, v0],
        [0, 0, 1]
    ])

    return matrix


def pixel2meters(u, v):
    matrix = intrinsicCamaraMatrix()
    n = 2
    fx = matrix[0, 0]
    fy = matrix[1, 1]
    u0 = matrix[0, 2]
    v0 = matrix[1, 2]
    x = (u - u0) / fx
    y = (v - v0) / fy

    return [x, y]


def find_y_centroid(image_path, rgbColor):
    # Localización precisa del centroide del marcador amarillo.

    # Leemos la imágen a procesar
    img = cv2.imread(image_path)
    if img is None:
        error_message = f"Error: Could not read image: {image_path}"
        print(error_message)
        return None, error_message

    # Convertimos la imágen de RGB a HSV el cual tiene mejores resultados para segmentación por colore ya que separa la infromación
    # del color (hue), de las características adicionales como la intensidad (value) y la pureza del color (saturation)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # El color del marcador amarillo en RGB lo pasamos a HSV para buscarlo después en la imagen
    hue, saturation, value = rgb_to_hsv(rgbColor)
    # print(hue, saturation, value)

    # Creamos una imágen con el color HSV para visualizar y mostrar al usuario
    hsv_image = np.zeros((200, 200, 3), dtype=np.uint8)
    hsv_image[:] = (hue, saturation, value)

    # Mostramos también la imágen en HSV
    bgr_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
    """
    cv2.imshow("HSV Color Display", bgr_image)
    cv2.imshow("HSV Image", hsv)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    """

    # Ponemos valores a las tolerancias que permitiremos, para identificar el valor. Se obtienen de forma experimental.
    hueVar = 3
    satVar = 50
    valueVar = 50

    # Creamos el rango de busqueda
    lower_yellow = np.array([hue - hueVar, saturation - satVar, value - valueVar], dtype=np.uint8)
    upper_yellow = np.array([hue + hueVar, 255, 255], dtype=np.uint8)

    # Creamos una mascara binaria donde damos valor 1 (255 rgb) a los valores de los pixeles en el rango y 0 a los que no estén en el rango
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    """
    cv2.imshow("Mask", mask)
    """

    # Analisis de componentes conectados o clusters de los puntos blancos del punto anterior (blobs)
    # Num labels me dice cuantas regiones encontró
    # labels me da una matriz con el ID de cada pixel sobre el grupo en el que cayo. Mayor a 1 y 0e s background
    # Stats me da estadísitica de cada region (area, bordes, etc)
    # Centroid me da el centroide de cada región. Similar a nearest neighbors.
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)

    #  Ahora filtramos las regiones y vemos cual tiene la mayor area y asumimos que el marcador cae en esta región.
    largest_label = 0
    largest_area = 0
    for i in range(1, num_labels):  # Start from 1 to skip background
        area = stats[i, cv2.CC_STAT_AREA]
        # Checkar que el area sea circular, antes de usarla como candidato a
        x, y, w, h, _ = stats[i]  # (x,y) es bounding box, w,h es width and height
        if is_circular(w, h, area):
            if area > largest_area:
                largest_area = area
                largest_label = i

    # Si el valor es 0, todo es background y no encontramos label.
    if largest_label == 0:
        error_message = "Error: Yellow marker not found."
        return None, error_message

    # Tomamos el centroide de la región del marcador
    cx, cy = int(centroids[largest_label][0]), int(centroids[largest_label][1])

    # Calculamos ahora el centro de la imágen para cambiar eje de cordenadas de arriba-izquierda al centro de la imagen
    height, width = img.shape[:2]  # 640x480
    center_x = width // 2
    center_y = height // 2

    # Transformamos centroides de eje de referencia.
    centered_cx = cx - (width / 2)
    centered_cy = (cy - (height / 2)) * (-1)
    # print(f"Centroid: ({cx}, {cy})")
    print(f"Centroid (centered): ({centered_cx}, {centered_cy})")

    """
    # Pintamos de blanco el centroide
    cv2.circle(img, (cx, cy), 5, (255, 255, 255), -1)  # -1 fills the circle
    # Pintamos el eje x
    cv2.line(img, (0, center_y), (width, center_y), (255, 255, 255), 1)
    # Pintamos el eje y
    cv2.line(img, (center_x, 0), (center_x, height), (255, 255, 255), 1)
    cv2.imshow("Image with Centroid", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    """
    return (centered_cx, centered_cy), None


def rgb_to_hsv(rgb_color):
    """Converts an RGB color tuple to HSV."""

    # Convert RGB to a NumPy array (OpenCV expects a NumPy array)
    rgb = np.uint8([[list(rgb_color)]])  # Note: OpenCV uses BGR, not RGB!

    # Convert to HSV
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)  # Or cv2.COLOR_BGR2HSV if you got BGR from paint

    # Extract the HSV values
    hue = hsv[0][0][0]
    saturation = hsv[0][0][1]
    value = hsv[0][0][2]

    return hue, saturation, value


def is_circular(width, height, area, tolerance=0.2):
    """
    Checks if a shape is approximately circular based on its width, height, and area.

    Args:
        width: The width of the shape.
        height: The height of the shape.
        area: The area of the shape.
        tolerance: The tolerance for the circularity check (default: 0.2).

    Returns:
        True if the shape is approximately circular, False otherwise.
    """
    if width == 0 or height == 0:
        return False

    # Checkar que width y height son aproximadamente igual mas menos toleranica en %
    aspect_ratio = float(width) / height
    if aspect_ratio < 1 - tolerance or aspect_ratio > 1 + tolerance:
        return False

    # Check if the area is approximately equal to the area of a circle with radius = width/2
    radius = width / 2.0
    expected_area = math.pi * radius * radius
    area_ratio = float(area) / expected_area
    if area_ratio < 1 - tolerance or area_ratio > 1 + tolerance:
        return False

    return True


# Método para publicar posicion deseada y posicion actual
def publicarError(pa, pd):
    connection = None
    try:
        """
        connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        channel = connection.channel()
        channel.queue_declare(queue='error')
        """
        # Prepare the data to be sent
        data_to_send = [pa[0], pa[1], pd[0], pd[1]]
        message = str(data_to_send)  # Convert the array to a string

        # Publish the data
        """
        channel.basic_publish(exchange='', routing_key='error', body=message)
        print(f"Sent error data: {data_to_send} to Queue")
        """
        # Write the data to a file
        file_path = 'img/error_data.txt'
        # Check if the file exists
        if not os.path.exists(file_path):
            # Create the file if it doesn't exist
            with open(file_path, 'w') as file:
                file.write("Centroid Data:\n")  # Add a header if creating a new file

        with open(file_path, 'a') as file:
            file.write(message + "\n")

    except pika.exceptions.AMQPConnectionError as e:
        print(f"Error: Failed to connect to RabbitMQ: {e}")
    except Exception as e:
        print(f"Error: An unexpected error occurred: {e}")
    finally:
        if connection and connection.is_open:
            connection.close()


if __name__ == '__main__':
    consumirImagenActual()
