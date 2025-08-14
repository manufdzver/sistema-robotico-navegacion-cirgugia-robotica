// PruebaConsola.cpp : Defines the entry point for the console application.
//


#include <HD/hd.h>// Library of haptic device
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <Windows.h>
#include <math.h>
#include <chrono>
#include "mmsystem.h"
#include <fstream>       // Necesario para E/S de archivos (para controlIbvs)
#include <string>        // Necesario para leer líneas de archivo (para controlIbvs)
#include <sstream>       // Necesario para parsear líneas de archivo (para controlIbvs)
#include <cstdio>        // Para getchar, EOF
#include <algorithm>     // Necesario para std::remove y std::replace


//Definiciones
#define pi 3.1415926535
#define MAX_GRAF_ROWS 40000

//Variables globales
HHD hHDm;
bool initialized = false, schedulerStarted = false, keepRunningIbvs = false; // user flags
HDdouble taum[3] = { 0,0,0 };//{-26.65,180.57,153.448};
HDSchedulerHandle servoLoopHandle;
const double T = 0.001; // Sample time
const int n = 3; // Number of joints


double qm[3] = { 0.0 }; //Variable global para almacenar los ángulos de cada joint y poder verlas en todos los métodos.
double qmDeseada[3] = { 0.0 }; //Aqui se almacenan los angulos de cada joint a los que queremos llegar
double pos[3] = { 0.0 }; //Variable global para almacenar las posiciones xyz del efector final 
const double angle_final_effector = 15.0 * pi / 180.0;
const double a2 = 0.145, a3 = sqrt(.135 * .135 + .04 * .04 - 2.0 * 0.135 * .04 * cos(pi - angle_final_effector));
const double gamma_final_effector = asin(0.04 * sin(pi - angle_final_effector) / a3);

double qPunto[MAX_GRAF_ROWS][3] = { 0.0 };
int vAct = 0;
int vIni = 0;
double qLast[3] = { 0.0 };

//Firmas de metodos
void inicializa();
void termina();
void escribePos();
void fija();
void fijaAX();
void control(double tf);
void suelta();
void escribeVact();
void controlIbvs();      // <<< Controlador IBVS (Image-Based Visual Servoing)
HDCallbackCode HDCALLBACK ServoLoopCallback(void*);

typedef struct
{
	hduVector3Dd position;
} DeviceStateStruct;

DeviceStateStruct state;

int main(int argc, char* argv[])
{
	int resp = 0;
	printf_s("HOLA. A continuacion se muestran las posibles operaciones. \nPresione 1 para inicializar el robot. \n");
	printf_s("Presione 2 para leer la posicion actual del robot. \n");
	printf_s("Presione 3 para fijar el robot en la posicion actual. \n");
	printf_s("Presione 4 para soltar el robot de la posicion actual. \n");
	printf_s("Presione 5 para apagar el robot y salir del programa. \n");
	printf_s("Presione 6 para fijar el robot a una posicion deseada. \n");
	printf_s("Presione 7 para escribir vAct. \n");
	printf_s("Presione 8 para ejecutar control IBVS desde archivo (continuo). \n");
	scanf_s("%d", &resp);
	while (resp != 5)
	{
		switch (resp)
		{
		case 1: inicializa(); break;
		case 2: escribePos(); break;
		case 3: fija(); break;
		case 4: suelta(); break;
		case 6: fijaAX(); break;
		case 7: escribeVact(); break;
		case 8: controlIbvs(); 
		}
		printf_s("Esperando siguiente orden...\n");
		scanf_s("%d", &resp);
	}
	termina();
	printf_s("Saliendo...\n");
	return 0;
}
void inicializa()
{
	printf_s("Inicializando robot. \n");
	HDErrorInfo error;
	HDstring MasterRobot = "Default Device";
	hHDm = hdInitDevice(MasterRobot);

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		printf_s("No se encontro el robot.");
		return;
	}

	servoLoopHandle = hdScheduleAsynchronous(ServoLoopCallback, &state, HD_MAX_SCHEDULER_PRIORITY);

	hdMakeCurrentDevice(hHDm);
	if (!hdIsEnabled(HD_FORCE_OUTPUT))
		hdEnable(HD_FORCE_OUTPUT);

	if (HD_DEVICE_ERROR(error = hdGetError()))
		printf_s("Force output enable error!");

	if (!schedulerStarted)
	{
		hdStartScheduler();
		schedulerStarted = true;
		Sleep(1500);
	}


	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		printf_s("Servo loop initialization error");
		hdDisableDevice(hHDm);
	}
	else
	{
		initialized = true;
		printf_s("*** Phantom Robot initialized ***\n");
	}

}
void fija()
{
	qmDeseada[0] = qm[0];
	qmDeseada[1] = qm[1];
	qmDeseada[2] = qm[2];
	control(2);
}
void fijaAX()
{
	double resp;
	scanf_s("%lf", &resp);
	qmDeseada[0] = resp * pi / 180;
	scanf_s("%lf", &resp);
	qmDeseada[1] = resp * pi / 180;
	scanf_s("%lf", &resp);
	qmDeseada[2] = -resp * pi / 180;
	vIni = 1;
	control(2);
	vIni = 0;
}
void suelta()
{
	taum[0] = 0;
	taum[1] = 0;
	taum[2] = 0;
}
void control(double tFinal)
{
	double ordenadas[n] = { 0.0 }, pendientes[n] = { 0.0 };
	static double bm0[n] = { 0.0 }, bm3[n] = { 0.0 }, bm4[n] = { 0.0 }, bm5[n] = { 0.0 };	//Coeficientes del polinomio
	int unaVez = 0;
	double qmActual[3] = { 0.0 };
	double t = 0.0;
	double error[n] = { 0.0 };
	double integralError[n] = { 0.0 };
	double sec = 0;
	const double kpm[n] = { 1.5,1.5,1.5 };	//Ganancias Proporcional
	const double kim[n] = { .25,.25,.25 }; 	//Ganancias Integral 
	struct timeval inicio, fin;
	Sleep(2000);
	if (unaVez == 0)
	{
		for (int i = 0; i < n; i++) //Calculamos las rectas que nos llevarán al valor deseado
		{
			ordenadas[i] = qm[i];
			pendientes[i] = (qmDeseada[i] - qm[i]) / tFinal;

			bm0[i] = qm[i];
			bm3[i] = 10 * (qmDeseada[i] - qm[i]) / (tFinal * tFinal * tFinal);
			bm4[i] = -15 * (qmDeseada[i] - qm[i]) / (tFinal * tFinal * tFinal * tFinal);
			bm5[i] = 6 * (qmDeseada[i] - qm[i]) / (tFinal * tFinal * tFinal * tFinal * tFinal);
		}
		unaVez++;
	}
	while (t < tFinal)
	{
		auto inicio = std::chrono::high_resolution_clock::now();
		for (int i = 0; i < n; i++) //Calculamos el sig punto a donde debemos movernos (qActual)
			//qmActual[i] = pendientes[i] * t + ordenadas[i];
			qmActual[i] = bm5[i] * (t * t * t * t * t) + bm4[i] * (t * t * t * t) + bm3[i] * (t * t * t) + bm0[i];
		for (int i = 0; i < n; i++) //Calculamos el error actual
			error[i] = qm[i] - qmActual[i];
		//printf_s("Error %f     Tiempo = %f\n",error[0],t);
		auto fin = std::chrono::high_resolution_clock::now();
		auto diferencia = fin - inicio;
		double s = diferencia.count(); //nano segundos
		s = s / 1000000000;//segundos
		for (int i = 0; i < n; i++) //Integral del error
			integralError[i] += error[i] * s;
		t += s;
		for (int i = 0; i < n; i++) //Control PI
			taum[i] = -kpm[i] * error[i] - kim[i] * integralError[i];

		for (int i = 0; i < n; i++) //Velocity
			//qPunto[vAct][i] = (qm[i] - qLast[i]) / s;
			vAct++;

	}
	// Imprimir los valores de torque calculados en cada iteración. '\r' mueve el cursor al inicio de la línea.
	printf_s("Tau (Nm): T0=%+8.4f, T1=%+8.4f, T2=%+8.4f\r", taum[0], taum[1], taum[2]);
	printf_s("Listo. Fijado.\n");
}
void termina()
{
	printf_s("Apagando robot... \n");
	char resp;
	if (initialized && hdIsEnabled(HD_FORCE_OUTPUT))
		hdDisable(HD_FORCE_OUTPUT);
	hdUnschedule(servoLoopHandle);
	if (schedulerStarted)
		hdStopScheduler();
	if (initialized)
	{
		hdDisableDevice(hHDm);
		initialized = false;
		schedulerStarted = false;
	}
	printf_s("Robot apagado. Presione enter para terminar. \n");
	scanf_s(&resp);
}

HDCallbackCode HDCALLBACK ServoLoopCallback(void* pUserData)
{
	DeviceStateStruct* pState = static_cast<DeviceStateStruct*>(pUserData);
	HDdouble torque[3];
	hdBeginFrame(hHDm);

	hdGetDoublev(HD_CURRENT_POSITION, pState->position); //Leemos coordenadas del efector final
	pos[0] = state.position[0];
	pos[1] = state.position[1];
	pos[2] = state.position[2];

	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, pState->position); //Leemos ángulos de los joints
	qm[0] = -state.position[0];
	qm[1] = state.position[1];
	qm[2] = state.position[2] - 0.5 * pi - qm[1] - gamma_final_effector;

	hdGetDoublev(HD_LAST_JOINT_ANGLES, pState->position); //Leemos ángulos de los joints
	qLast[0] = -state.position[0];
	qLast[1] = state.position[1];
	qLast[2] = state.position[2] - 0.5 * pi - qLast[1] - gamma_final_effector;

	/*if (vIni == 1)
	{
		hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, pState->position); //Leemos velocidades de los joints
		qPunto[vAct][0] = state.position[0];
		qPunto[vAct][1] = state.position[1];
		qPunto[vAct][2] = state.position[2];
		vAct++;
	}*/

	torque[0] = -1000.0 * taum[0];
	torque[1] = 1000.0 * taum[1];
	torque[2] = 1000.0 * taum[2];

	hdSetDoublev(HD_CURRENT_JOINT_TORQUE, torque);

	hdEndFrame(hHDm);

	return HD_CALLBACK_CONTINUE;
}
void escribePos()
{
	printf_s("Posicion actual (en grados): \n");
	printf_s("Q0 = %.3f \nQ1 = %.3f \nQ2 = %.3f\n\n", qm[0] * 180.0 / pi, qm[1] * 180.0 / pi, qm[2] * 180.0 / pi);
	printf_s("Posicion actual (en mm): \n");
	printf_s("X = %.3f \nY = %.3f \nZ = %.3f\n\n", pos[0], pos[1], pos[2]);
	printf_s("Torque actual (mili Nm): \n");
	printf_s("T0 = %f \nT1 = %f \nT2= %f\n\n", taum[0], taum[1], taum[2]);
}

void escribeVact()
{
	for (int i = 0; i < vAct; i++)
	{
		printf_s("Velocidad actual (en rad/s): \n");
		printf_s("q1. = %.13f \nq2. = %.13f \nq3. = %.13f\n\n", qPunto[i][0], qPunto[i][1], qPunto[i][2]);
	}
}

void controlIbvs()
{
	// Chequeo de seguridad
	if (!initialized)
	{
		printf_s("Error: Robot no inicializado en controlIbvs().\n");
		return;
	}
	// Evitar iniciar si ya está corriendo
	if (keepRunningIbvs)
	{
		printf_s("Control IBVS ya esta corriendo.\n");
		return;
	}

	// Usar una ruta absoluta para asegurar que el archivo se encuentre sin importar desde dónde se ejecute el programa.
	const char* filename = "C:\\Users\\manuf\\OneDrive\\Escritorio\\SistemaNavegacionCirugiaRobotica\\img\\error_data.txt";
	std::ifstream infile(filename);

	if (!infile.is_open())
	{
		printf_s("Error: No se pudo abrir el archivo %s\n", filename);
		return;
	}

	// --- Omitir la primera línea (cabecera) ---
	std::string header_line;
	if (!std::getline(infile, header_line))
	{
		printf_s("Error: No se pudo leer la linea de cabecera de %s.\n", filename);
		infile.close();
		return;
	}
	printf_s("Archivo %s abierto, ignorando cabecera: %s\n", filename, header_line.c_str());

	printf_s("Iniciando control IBVS desde archivo %s (Presione 4 para detener)...\n", filename);
	keepRunningIbvs = true; // <<< INICIAR BUCLE

	std::string line;
	int line_count = 0;
	long long iteration_count = 0;
	HDdouble current_vel_raw[3]; // Para velocidades crudas

	// --- Bucle Principal IBVS ---
	while (keepRunningIbvs)
	{
		iteration_count++;

		// --- Lectura de Archivo (con rebobinado) ---
		if (!std::getline(infile, line))
		{
			if (infile.eof())
			{
				infile.clear();
				infile.seekg(0, std::ios::beg);
				if (!std::getline(infile, header_line)) { // Omitir cabecera de nuevo
					printf_s("Error: No se pudo leer cabecera de %s despues de reiniciar.\n", filename);
					keepRunningIbvs = false; break;
				}
				line_count = 0;
				if (!std::getline(infile, line)) { // Leer primera línea de datos
					printf_s("Error: No se pudo leer datos de %s despues de reiniciar.\n", filename);
					keepRunningIbvs = false; break;
				}
			}
			else {
				printf_s("Error leyendo linea %d del archivo.\n", line_count + 1);
				keepRunningIbvs = false; break;
			}
		}
		line_count++;

		// --- Parseo de Línea ---
		// La línea original tiene el formato "[pax, pay, pdx, pdy]".
		// Se deben eliminar los caracteres '[]' y reemplazar ',' por espacios para que stringstream pueda parsear los números.
		line.erase(std::remove(line.begin(), line.end(), '['), line.end());
		line.erase(std::remove(line.begin(), line.end(), ']'), line.end());
		std::replace(line.begin(), line.end(), ',', ' ');

		std::stringstream ss(line);
		double pax, pay, pdx, pdy;
		// Intentar leer los 4 valores de punto flotante
		if (!(ss >> pax >> pay >> pdx >> pdy))
		{
			printf_s("Error parseando linea %d (formato esperado: 4 numeros). Contenido: '%s'. Saltando.\n", line_count, line.c_str());
			continue;
		}

		// --- Obtener Estado Robot (de globales qm, pos) ---
		double q1 = qm[0]; double q2 = qm[1]; double q3 = qm[2]; // Rad
		double x = pos[0] / 1000.0; double y = pos[1] / 1000.0; double Z = pos[2] / 1000.0; // m

		// --- Obtener Velocidades (del dispositivo) ---
		hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, current_vel_raw); // rad/s
		// Aplicar transformaciones (¡VERIFICAR!)
		double q1punto = -current_vel_raw[0];
		double q2punto = current_vel_raw[1];
		double q3punto = current_vel_raw[2] - current_vel_raw[1];

		// --- Cálculo de Torque (Fórmulas IBVS Simplificadas) ---
		double cos_q2 = cos(q2); double cos_q3 = cos(q3);
		double sin_q2 = sin(q2); double sin_q3 = sin(q3);
		double sin_q2_m_q3 = sin(q2 - q3);
		double cos_q2_m_q3 = cos(q2 - q3);
		double error_x = pdx - pax; // Error visual en el eje X (pixeles) -> deseado - actual
		double error_y = pdy - pay; // Error visual en el eje Y (pixeles) -> deseado - actual

		const double kp11 = 0.00020; // Mantenemos KP bajo.
		const double kp22 = 0.00032; // Mantenemos KP bajo.
		const double kv11 = 2.2;     // Sin cambios por ahora.
		const double kv22 = 5.0;     // Aumentado para manejar la nueva fuerza de G2.
		const double kv33 = 3.5;     // Aumentado drásticamente para "domar" la articulación 3.

		// --- GANANCIAS DE COMPENSACIÓN DE GRAVEDAD ---
		// Aumentamos G2 para que la articulación 2 no se caiga.
		const double G2 = 13.0; // Aumento significativo para sostener la articulación 2.
		const double G3 = 2.3;  // Mantenemos G3 por ahora, el problema es de agresividad (damping).


		// Chequeo de seguridad para la división por Z
		if (abs(Z) < 1e-9)
		{
			if (iteration_count % 100 == 0) { // Evita spam en la consola
				printf_s("Advertencia: Profundidad Z es casi cero. Saltando cálculo de torque.\n");
			}
			taum[0] = 0; taum[1] = 0; taum[2] = 0;
		}
		else
		{
			// Cálculo de tau1
			double tau1_calc = kp11 * (error_x) * ((2614827 * cos_q3) / 1000000.0 + (1000000 * pow(x, 2) * cos_q3) / 2614827.0 - (1000000 * ((337312683 * x * cos_q2) / 1000000000.0 + (347771991 * x * cos_q3) / 1000000000.0)) / (2614827.0 * Z) - (1000000 * x * y * sin_q3) / 2614827.0) - kp22 * (error_y) * ((2614827 * sin_q3) / 1000000.0 + (1000000 * pow(y, 2) * sin_q3) / 2614827.0 + ((129.0 * y * cos_q2) / 1000.0 + (133.0 * y * cos_q3) / 1000.0) / Z - (1000000 * x * y * cos_q3) / 2614827.0) - kv11 * q1punto;

			// Cálculo de tau2
			// double tau2_calc = (509.0 * cos_q2) / 10000.0 - kv22 * q2punto - kp22 * (error_y) * (x - (2614827.0 * ((129.0 * cos_q2_m_q3) / 1000.0 + 133.0 / 1000.0)) / (1000000.0 * Z)) + kp11 * (y - (337312683.0 * sin_q2_m_q3) / (1000000000.0 * Z)) * (error_x);

			// Cálculo de tau3
			// double tau3_calc = (129.0 * cos_q3) / 2500.0 - kv33 * q3punto - kp22 * (x - 347771991.0 / (1000000000.0 * Z)) * (error_y)+kp11 * y * (error_x);

			// Cálculo de tau2
			double tau2_calc = (509.0 * cos_q2) / 10000.0 - kv22 * q2punto - kp22 * (error_y) * (x - (2614827.0 * ((129.0 * cos_q2_m_q3) / 1000.0 + 133.0 / 1000.0)) / (1000000.0 * Z)) + kp11 * (y - (337312683.0 * sin_q2_m_q3) / (1000000000.0 * Z)) * (error_x);

			// Cálculo de tau3
			double tau3_calc = G3 * ((129.0 * cos_q3) / 2500.0) - kv33 * q3punto - kp22 * (x - 347771991.0 / (1000000000.0 * Z)) * (error_y)+kp11 * y * (error_x);
			// Actualizar comando global de torque
			taum[0] = tau1_calc;
			taum[1] = tau2_calc;
			taum[2] = tau3_calc;

			// Imprimir el error y los torques en cada iteración. '\r' mueve el cursor al inicio de la línea.
			printf_s("Error (px): X=%+7.1f, Y=%+7.1f | Tau (Nm): T0=%+8.4f, T1=%+8.4f, T2=%+8.4f\n", error_x, error_y, taum[0], taum[1], taum[2]);
		}

		// --- Temporización ---
		Sleep(1); // Ceder CPU (1ms)

	} // Fin while(keepRunningIbvs)

	// --- Limpieza ---
	infile.close();
	printf_s("Control IBVS detenido.\n");
	taum[0] = 0.0; taum[1] = 0.0; taum[2] = 0.0; // Asegurar torques a cero

} // Fin controlIbvs
