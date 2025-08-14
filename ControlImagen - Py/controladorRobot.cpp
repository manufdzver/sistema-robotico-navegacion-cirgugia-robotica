// PruebaConsola.cpp : Defines the entry point for the console application.
//


#include <HD/hd.h>// Library of haptic device
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <Windows.h>
#include <math.h>
#include <chrono>
#include "mmsystem.h"


//Definiciones
#define pi 3.1415926535
#define MAX_GRAF_ROWS 40000

//Variables globales
HHD hHDm;
bool initialized = false, schedulerStarted = false; // user flags
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
	const double kpm[n] = { 1.2,1.2,1.2 };	//Ganancias Proporcional
	const double kim[n] = { .2,.2,.2 }; 	//Ganancias Integral 
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