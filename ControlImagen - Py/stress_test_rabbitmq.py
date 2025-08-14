import pika
import time
import numpy as np
import sys
import threading

RABBITMQ_HOST = 'localhost'
NUM_SOLICITUDES = 1000

CONFIG_COLAS = [1, 10, 50, 100, 200, 300, 400, 500, 1000, 10000]
MENSAJE_DUMMY = b'payload_prueba_estres'

def stress_worker(queue_name, num_requests, host, result_list, index):
    """
    Función de trabajo para cada hilo. Simula un cliente concurrente.
    Cada hilo establece su propia conexión a RabbitMQ, ya que no son thread-safe.
    """
    latencies = []
    success_count = 0
    connection = None
    try:
        parametros = pika.ConnectionParameters(host=host)
        connection = pika.BlockingConnection(parametros)
        channel = connection.channel()

        for _ in range(num_requests):
            tiempo_inicio = time.perf_counter()

            channel.basic_publish(exchange='', routing_key=queue_name, body=MENSAJE_DUMMY)
            metodo, _, _ = channel.basic_get(queue=queue_name, auto_ack=True)

            tiempo_fin = time.perf_counter()

            if metodo:
                latencies.append(tiempo_fin - tiempo_inicio)
                success_count += 1
    except Exception as e:
        print(f"Error en el worker para la cola {queue_name}: {e}", file=sys.stderr)
    finally:
        if connection and connection.is_open:
            connection.close()
        result_list[index] = (latencies, success_count)

def ejecutar_prueba_estres(num_colas, num_solicitudes):
    """
    Ejecuta una prueba de estrés CONCURRENTE.
    Crea 'num_colas' hilos, cada uno actuando como un cliente para su propia cola.
    """
    conexion_admin = None
    nombres_colas = [f'cola_prueba_estres_{i}' for i in range(num_colas)]

    try:
        # 1. Conexión principal para tareas administrativas (crear/borrar colas)
        parametros = pika.ConnectionParameters(host=RABBITMQ_HOST)
        conexion_admin = pika.BlockingConnection(parametros)
        canal_admin = conexion_admin.channel()

        # 2. Declarar todas las colas para esta prueba
        print(f"    Declarando {num_colas} colas...")
        for nombre_q in nombres_colas:
            canal_admin.queue_declare(queue=nombre_q, durable=False)
        print("    Colas declaradas.")

        # 3. Configurar y ejecutar los hilos
        threads = []
        # Una lista para guardar los resultados de cada hilo: ([latencias], conteo_exitoso)
        thread_results = [None] * num_colas
        
        # Distribuir las solicitudes entre los hilos
        requests_per_thread = num_solicitudes // num_colas
        remainder = num_solicitudes % num_colas

        print(f"    Iniciando {num_colas} hilos concurrentes...")
        for i in range(num_colas):
            cola_actual = nombres_colas[i]
            num_reqs = requests_per_thread + (1 if i < remainder else 0)
            
            if num_reqs == 0:
                thread_results[i] = ([], 0) # No hay trabajo que hacer
                continue

            thread = threading.Thread(
                target=stress_worker,
                args=(cola_actual, num_reqs, RABBITMQ_HOST, thread_results, i)
            )
            threads.append(thread)
            thread.start()

        # Esperar a que todos los hilos terminen
        for thread in threads:
            thread.join()
        print("    Hilos finalizados.")

        # 4. Agregar los resultados de todos los hilos
        all_latencies = []
        total_successes = 0
        for result in thread_results:
            if result:
                latencies, successes = result
                all_latencies.extend(latencies)
                total_successes += successes

    except pika.exceptions.AMQPConnectionError as e:
        print(f"Error Fatal: No se pudo conectar a RabbitMQ en {RABBITMQ_HOST}. Por favor, asegúrate de que esté en ejecución.", file=sys.stderr)
        return None # Indicar fallo
    except Exception as e:
        print(f"Ocurrió un error inesperado: {e}", file=sys.stderr)
        return None # Indicar fallo
    finally:
        # 5. Limpieza: Borrar las colas utilizadas
        if conexion_admin and conexion_admin.is_open:
            if not canal_admin.is_open:
                canal_admin = conexion_admin.channel()
            print(f"    Borrando {num_colas} colas...")
            for nombre_q in nombres_colas:
                canal_admin.queue_delete(queue=nombre_q)
            conexion_admin.close()

    # 5. Calcular estadísticas (convertir a milisegundos)
    tiempo_promedio_ms = np.mean(all_latencies) * 1000 if all_latencies else 0
    desviacion_estandar_ms = np.std(all_latencies) * 1000 if all_latencies else 0
    porcentaje_exito = (total_successes / num_solicitudes) * 100 if num_solicitudes > 0 else 0

    return {
        "colas": num_colas,
        "tiempo_promedio_ms": tiempo_promedio_ms,
        "desviacion_estandar_ms": desviacion_estandar_ms,
        "porcentaje_exito": porcentaje_exito
    }

def graficar_resultados(resultados):
    """
    Genera y muestra gráficos a partir de los resultados de la prueba de estrés.
    """
    if not resultados:
        print("No hay resultados para graficar.")
        return

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nAdvertencia: Matplotlib no está instalado. No se pueden generar las gráficas.")
        print("Para instalarlo, ejecuta: pip install matplotlib")
        return

    colas = [r['colas'] for r in resultados]
    tiempos_promedio = [r['tiempo_promedio_ms'] for r in resultados]
    desviaciones_estandar = [r['desviacion_estandar_ms'] for r in resultados]

    # Gráfica 1: Tiempo de Respuesta Promedio vs. Número de Colas
    plt.figure(figsize=(10, 6))
    plt.plot(colas, tiempos_promedio, marker='o', linestyle='-', color='b')
    plt.title('Tiempo de Respuesta Promedio vs. Número de Colas Concurrentes')
    plt.xlabel('Número de Colas (Clientes Concurrentes)')
    plt.ylabel('Tiempo Promedio de Respuesta (ms)')
    plt.grid(True, which="both", ls="--")
    plt.xscale('log')
    plt.yscale('log')
    plt.show()

    # Gráfica 2: Desviación Estándar vs. Número de Colas
    plt.figure(figsize=(10, 6))
    plt.plot(colas, desviaciones_estandar, marker='s', linestyle='--', color='r')
    plt.title('Desviación Estándar del Tiempo de Respuesta vs. Número de Colas')
    plt.xlabel('Número de Colas (Clientes Concurrentes)')
    plt.ylabel('Desviación Estándar (ms)')
    plt.grid(True, which="both", ls="--")
    plt.xscale('log')
    plt.yscale('log')
    plt.show()

def main():
    todos_los_resultados = []
    print("Iniciando Prueba de Estrés para RabbitMQ...")
    for conteo_q in CONFIG_COLAS:
        print(f"Probando con {conteo_q} cola(s) y {NUM_SOLICITUDES} solicitudes...")
        resultado = ejecutar_prueba_estres(conteo_q, NUM_SOLICITUDES)
        if resultado is None: break
        todos_los_resultados.append(resultado)
        print("Prueba completada.")

    print("\n" + "="*85)
    print("--- Resultados de la Prueba de Estrés ---")
    print("="*85)
    print(f"{'# Colas':<10} | {'Tiempo Promedio (ms)':<25} | {'Desv. Estándar (ms)':<25} | {'% Éxito':<15}")
    print("-" * 85)
    for res in todos_los_resultados:
        print(f"{res['colas']:<10} | {res['tiempo_promedio_ms']:<25.4f} | {res['desviacion_estandar_ms']:<25.4f} | {res['porcentaje_exito']:<15.2f}")
    print("="*85)

    # Graficar los resultados
    graficar_resultados(todos_los_resultados)

if __name__ == '__main__':

    main()
