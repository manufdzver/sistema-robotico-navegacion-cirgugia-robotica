import matplotlib.pyplot as plt
import ast
import os
import math

def plot_trajectory(file_path='img/error_data.txt'):
    """
    Reads centroid data from a file and plots the actual vs. desired trajectory.

    The file is expected to have lines with a list of four numbers:
    [actual_x, actual_y, desired_x, desired_y]
    """
    actual_x_coords = []
    actual_y_coords = []
    desired_x_coords = []
    desired_y_coords = []
    error_magnitudes = []

    if not os.path.exists(file_path):
        print(f"Error: The file '{file_path}' was not found.")
        return

    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Skip the header and process each data line
    for line in lines[1:]:
        line = line.strip()
        if not line:
            continue
        try:
            # ast.literal_eval safely evaluates a string containing a Python literal
            coords = ast.literal_eval(line)
            if len(coords) == 4:
                ax, ay, dx, dy = coords[0], coords[1], coords[2], coords[3]
                actual_x_coords.append(ax)
                actual_y_coords.append(ay)
                desired_x_coords.append(dx)
                desired_y_coords.append(dy)

                # Calcular la magnitud del error (distancia Euclidiana)
                error = math.sqrt((dx - ax)**2 + (dy - ay)**2)
                error_magnitudes.append(error)
        except (ValueError, SyntaxError) as e:
            print(f"Warning: Could not parse line: '{line}'. Error: {e}")

    if not actual_x_coords:
        print("No valid data found to plot.")
        return

    # --- Gráfica 1: Trayectoria ---
    plt.figure(figsize=(10, 10))
    
    # Graficar la posición deseada (asumiendo que es constante, graficamos solo el primer punto)
    plt.plot(desired_x_coords[0], desired_y_coords[0], 'ro', label='Posición Deseada', markersize=12, zorder=5)
    
    # Graficar la trayectoria real del robot
    plt.plot(actual_x_coords, actual_y_coords, 'bo-', label='Trayectoria del Robot', markersize=4)

    plt.title('Trayectoria del Robot vs Posición Deseada')
    plt.xlabel('X Posición (pixels)')
    plt.ylabel('Y Posición (pixels)')
    plt.legend()
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box') # Ensure X and Y axes have the same scale
    plt.show()

    # --- Gráfica 2: Magnitud del Error a lo largo del tiempo ---
    if error_magnitudes:
        plt.figure(figsize=(12, 6))
        plt.plot(error_magnitudes, 'g-', label='Magnitud del Error')
        plt.title('Convergencia del Error a Cero')
        plt.xlabel('Muestra de Tiempo (iteración)')
        plt.ylabel('Error (distancia Euclidiana en píxeles)')
        plt.axhline(y=0, color='r', linestyle='--', label='Error Cero')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    plot_trajectory()
