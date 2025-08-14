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

def find_y_centroid(image_path, rgbColor):
    # Localización precisa del centroide del marcador amarillo.

    # Leemos la imágen a procesar
    img = cv2.imread(image_path)
    if img is None:
        error_message = f"Error: Could not read image: {image_path}"
        print(error_message)
        return None, error_message

    # Convertimos la imágen de RGB a HSV el cual tiene mejores resultados para segmentación por colores ya que separa la infromación
    # del color (hue), de las características adicionales como la intensidad (value) y la pureza del color (saturation)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # El color del marcador amarillo en RGB lo pasamos a HSV para buscarlo después en la imagen
    hue, saturation, value = rgb_to_hsv(rgbColor)
    print(f"El color RGB {rgbColor} corresponde a HSV ({hue}, {saturation}, {value})")

    # Creamos una imágen con el color HSV para visualizar y mostrar al usuario
    hsv_image = np.zeros((200, 200, 3), dtype=np.uint8)
    hsv_image[:] = (hue, saturation, value)

    # Mostramos también la imágen en HSV
    bgr_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

    cv2.imshow("HSV Color Display", bgr_image)
    cv2.imshow("HSV Image", hsv)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    

    # Ponemos valores a las tolerancias que permitiremos, para identificar el valor. Se obtienen de forma experimental.
    hueVar = 8
    satVar = 100
    valueVar = 100
    

    # Creamos el rango de busqueda
    lower_yellow = np.array([hue - hueVar, saturation - satVar, value - valueVar], dtype=np.uint8)
    upper_yellow = np.array([hue + hueVar, 255, 255], dtype=np.uint8)

    # Creamos una mascara binaria donde damos valor 1 (255 rgb) a los valores de los pixeles en el rango y 0 a los que no estén en el rango
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    cv2.imshow("Mask", mask)
    

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

    
    # Pintamos de blanco el centroide
    cv2.circle(img, (cx, cy), 5, (255, 255, 255), -1)  # -1 fills the circle
    # Pintamos el eje x
    cv2.line(img, (0, center_y), (width, center_y), (255, 255, 255), 1)
    # Pintamos el eje y
    cv2.line(img, (center_x, 0), (center_x, height), (255, 255, 255), 1)
    cv2.imshow("Image with Centroid", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return (centered_cx, centered_cy), None


def rgb_to_hsv(rgb_color):
    """Converts an RGB color tuple to HSV."""

    # Convertir RGB a NumPy array 
    rgb = np.uint8([[list(rgb_color)]])  # Note: OpenCV uses BGR, not RGB!

    # Convertir a HSV
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)  

    # Extraer HSV
    hue = hsv[0][0][0]
    saturation = hsv[0][0][1]
    value = hsv[0][0][2]

    return hue, saturation, value

def is_circular(width, height, area, tolerance=0.2):

    if width == 0 or height == 0:
        return False

    # Checar que width y height son aproximadamente igual mas menos toleranica en %
    aspect_ratio = float(width) / height
    if aspect_ratio < 1 - tolerance or aspect_ratio > 1 + tolerance:
        return False

    # Checar si la area de un circulo con radius = width/2
    radius = width / 2.0
    expected_area = math.pi * radius * radius
    area_ratio = float(area) / expected_area
    if area_ratio < 1 - tolerance or area_ratio > 1 + tolerance:
        return False

    return True

if __name__ == '__main__':
    imagenDeseada()
    find_y_centroid("img/imgDeseada.png", (r_yellow, g_yellow, b_yellow))
    print("Finished calibrating")

