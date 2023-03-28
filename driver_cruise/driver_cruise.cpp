/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Description: User Module for CyberCruise							█
█	作者: 杨辰兮 & ChatGPT												█
█	联系方式: yangchenxi@sjtu.edu.cn										█
█	日期: 2023.02.13							    						█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	贴士:	您可以折叠 #pragma region 和	#pragma endregion 之间的代码		█
█	这可以使您获得一次性折叠完成的程序块而不是一个函数的能力					█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	DLL接口部分，您可以跳过这部分不阅读									█
█	不要修改这个 #pragma region 中的任何代码!								█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< 不要修改这个 region 中的任何代码!
#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include "stdio.h"
#include <ostream>
#include <fstream>

#include "class_Visualization.h"
#define PI 3.141592653589793238462643383279

	static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void* pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo * modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_cruise";	// name of the module (short).
	modInfo[0].desc = "user module for CyberCruise";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void* pt)
{
	tUserItf* itf = (tUserItf*)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}

//Global variables for vehicle states
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm)
{
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;



}
#pragma endregion >>>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	上下确界约束函数									 					█
█	您需要理解它的功能，建议不要修改										█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< Boundaries of control	
double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}
#pragma endregion

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	赛道曲率半径计算函数													█
█	您需要理解它的功能，建议不要修改										█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< Radius of curvature
//		Given three points ahead, outputs a struct circle.				
//		{radius:[1,1000], sign{-1:left,1:right}							
typedef struct Circle
{
	double r;
	int sign;
}circle;

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
	x = (b * f - e * c) / (b * d - e * a);
	y = (d * c - a * f) / (b * d - e * a);
	r = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 500.0, r);
	int sign = (x > 0) ? 1 : -1;
	return { r,sign };
}
#pragma endregion >>>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	以下是核心控制程序													█
█	主要输入: _midline（道路中200米内中心线xy坐标）, _speed											█
█	次要输入: _yaw车辆运动方向Y与道路的夹角, _yawrate角速度, _acc加速度, _width道路宽度, _rpm发动机转速,	_gearbox发动机档位			█
█	主要输出: *cmdAcc, *cmdBrake, *cmdSteer  							█
█	次要输出: *cmdGear 【本样例中已实现】									█
█	详细信息请参见用户手册												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

//基于OpenCV的可视化工具，详情请见文档
cls_VISUAL cls_visual;

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	手动换挡程序															█
█	可以不用看懂，建议不要修改，除非您是学(Juan)霸(Wang) :P					█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< Manual Gear
const float fGearShift[2][7] = //0 for downshift, 1 for upshift
{
	0,92,128,167,208,233,255,
	0,105,145,182,224,249,272
};
void updateGear(int* cmdGear)
{

	if (_speed > fGearShift[1][_gearbox] && _gearbox < 7) //upshift
	{
		*cmdGear = _gearbox + 1;
	}
	else if (_speed < fGearShift[0][_gearbox - 1] && _gearbox > 1) //downshift
	{
		*cmdGear = _gearbox - 1;
	}
	else
	{
		*cmdGear = _gearbox;
	}
}
#pragma endregion >>>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	PID控制器，由ChatGPT生成												█
█	可选择性修改，需要完全理解												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
class PIDController//分开角度和速度系数
{
private:
	double kp, ki, kd;		// PID控制器的参数
	double targetValue;		// 目标值
	double lastError;		// 上一次误差值
	double errorIntegral;	// 误差积分值0

public:
	void initial(double p, double i, double d, double target)
	{
		kp = p;
		ki = i;
		kd = d;
		targetValue = target;
		lastError = 0;
		errorIntegral = 0;
	}

	double calculate(double input)//系数可调
	{
		double error = targetValue - input;
		double derivative = error - lastError;
		errorIntegral += error;
		lastError = error;
		return kp * error + ki * errorIntegral + kd * derivative;
	}

	double calculate1(double input)//系数可调
	{
		double error = targetValue - input;
		double derivative = error - lastError;
		errorIntegral += error;
		lastError = error;
		return (kp * error + ki * errorIntegral + kd * derivative) / input;//速度控制的归一化
	}

	double calculate2(double input)//负参数正向化
	{
		double error = input - targetValue;
		double derivative = error - lastError;
		errorIntegral += error;
		lastError = error;
		return (kp * error + ki * errorIntegral + kd * derivative) / input;//速度控制的归一化
	}


};

double max(double a, double b)
{
	if (a >= b) return a;
	else return b;
}


/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	车辆控制主程序，由ChatGPT自动生成助教完善								█
█	样例代码仅供参考，请在下方设计实现您的算法								█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
PIDController speedController;	//速度PID控制
PIDController angleController;	//舵角PID控制

double lastTargetSpeed = 999.0;	//上一帧目标速度
bool isFirstFrame = true;		//第一帧标志
int count = 0;

//检测道路部分的变量
int timeCounter = 0;
bool isDirt = false;
bool counterFinished = false;

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)

{
	/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
	█	舵角控制																█
	\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	double targetAngleError = 0.0; //目标误差
	//double currentAngleError = atan(_yaw); //当前误差!!!!!!!!!!!!

	double Distance = sqrt(pow(_midline[10][0], 2) + pow(_midline[10][1], 2));//原点距离计算
	//double currentAngleError = (0.25 * atan2(_midline[2][0], _midline[2][1]) + 0.5 * atan2(_midline[1][0], _midline[1][1]) + 0.25 * atan2(_midline[3][0], _midline[3][1])); //加权检测
	double currentAngleError = atan2(_midline[5][0], _midline[5][1]) / 50;



	circle myCurve1, myCurve2;
	float minCurve = 500.0, warnCurve = 500;

	float target = max(_speed * _speed * 0.00096, _speed * 0.48);

	for (int fStep = 5; fStep < target; fStep++)
	{
		myCurve1 = getR(_midline[fStep - 2][0], _midline[fStep - 2][1], _midline[fStep][0], _midline[fStep][1], _midline[fStep + 2][0], _midline[fStep + 2][1]);
		//myCurve1 = getR(_midline[fStep + 10][0], _midline[fStep][1], _midline[fStep + 10][0], _midline[fStep + 10][1], _midline[fStep + 20][0], _midline[fStep + 20][1]);
		//myCurve2 = getR(_midline[fStep + 30][0], _midline[fStep + 30][1], _midline[fStep + 30 + 10][0], _midline[fStep + 30 + 10][1], _midline[fStep + 30 + 20][0], _midline[fStep + 30 + 20][1]);
		if (myCurve1.r < minCurve)
		{
			minCurve = myCurve1.r;
		}
		//if (myCurve2.r < minCruve)
		//{
			//warnCruve = myCurve2.r;
		//}
	}

	double targetSpeed;  //目标车速
	double currentSpeed = _speed;

	//若为泥土赛道
	if (isDirt) {
		if (minCurve < 30) {
		targetSpeed = constrain(120, 140, 22 * sqrt(double(minCurve)));
		for (int i = 21; i <= 40; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 20;
			}
		}
		else if (minCurve < 50) {
		targetSpeed = constrain(140, 150, 16 * sqrt(double(minCurve)));
		for (int i = 21; i <= 50; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 30;
			}
		}
		else if (minCurve < 70) {
		targetSpeed = constrain(160, 190, 16 * sqrt(double(minCurve)));
		for (int i = 21; i <= 60; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 40;
			}
		}
		else if (minCurve < 95) {
		targetSpeed = constrain(190, 240, 16 * sqrt(double(minCurve)));
		for (int i = 21; i <= 70; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 50;
			}
		}
		else if (minCurve < 120) {
		targetSpeed = constrain(240, 320, 18 * sqrt(double(minCurve)));
		for (int i = 21; i <= 90; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 70;
			}
		}
		else {
		targetSpeed = constrain(320, 380, minCurve);
		for (int i = 21; i <= 120; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 100;
			}
	}
	//为水泥赛道
	else if (minCurve < 30) {
		targetSpeed = constrain(120, 140, 22 * sqrt(double(minCurve)));
		for (int i = 21; i <= 40; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 20;
			}
		}
		else if (minCurve < 50) {
		targetSpeed = constrain(140, 150, 16 * sqrt(double(minCurve)));
		for (int i = 21; i <= 50; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 30;
			}
		}
		else if (minCurve < 70) {
		targetSpeed = constrain(160, 190, 16 * sqrt(double(minCurve)));
		for (int i = 21; i <= 60; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 40;
			}
		}
		else if (minCurve < 95) {
		targetSpeed = constrain(190, 240, 16 * sqrt(double(minCurve)));
		for (int i = 21; i <= 70; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 50;
			}
		}
		else if (minCurve < 120) {
		targetSpeed = constrain(240, 320, 18 * sqrt(double(minCurve)));
		for (int i = 21; i <= 90; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 70;
			}
		}
		else {
		targetSpeed = constrain(320, 380, minCurve);
		for (int i = 21; i <= 120; ++i) {
			currentAngleError += atan2(_midline[i][0], _midline[i][1]) / 100;
			}
		}



	//第一帧初始化舵角控制参数，清空积分器和微分器，因为控制目标为恒零，所以只需要初始化一次
	if (isFirstFrame)
	{
		isFirstFrame = false;
		angleController.initial(12, 0.001, 7.0, targetAngleError);//只传递了一次
	}

	//舵角PID控制

	*cmdSteer = constrain(-1.0, 1.0, angleController.calculate(0.65 * currentAngleError));//修改的Stanly误差控制




	/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
	█	速度控制																█
	\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

	//计算前方是直道还是弯道


	/*
	//每当目标速度变化时初始化PID控制器，重设参数，清空积分器和微分器
	if (targetSpeed != lastTargetSpeed)
	{
		speedController.initial(7.0, 0.001, 0.01, targetSpeed);
		lastTargetSpeed = targetSpeed;
	}
	*/
	int tmp;

	if (abs(_midline[0][0]) > _width / 10)
	{
		tmp = count;
	}

	if (count < tmp + 100 || abs(Distance)>0.1)
	{
		speedController.initial(7.0, 0.001, 0.01, targetSpeed);
		lastTargetSpeed = targetSpeed;
	}

	//判定为泥土赛道
	if (isDirt) {
		if (currentSpeed < targetSpeed)  //当前速度低于目标，需要加速
		{	
			if (abs(*cmdSteer) < 0.1)//利用PID更广泛地控制速度
			{
			//速度较快且舵角较小时，使用PID控制，适用于平滑直道
			*cmdAcc = constrain(-2.0, 2.0, speedController.calculate1(currentSpeed)) * 800 / currentSpeed;

		}
		else if (abs(*cmdSteer) > 0.1)//起步油门
			{
			*cmdAcc = constrain(-2.0, 2.0, speedController.calculate1(currentSpeed)) / (abs(*cmdSteer) * 20);
			//*cmdAcc = 0.1;
			}
		//加速情况下，刹车为0
		*cmdBrake = 0;
		}
		else
		{
		*cmdAcc = 0;
		//不同减速情况下，刹车，间断轻踏过程
			
			//*cmdBrake  = constrain(0.0, 1.0, speedController.calculate2(currentSpeed))*sqrt( _speed / minCruve);
		*cmdBrake = 1.0;

		}
	}
	//水泥赛道
	//根据当前速度和目标速度关系，控制油门刹车以改变速度
	else if (currentSpeed < targetSpeed)  //当前速度低于目标，需要加速
		{	

		if (abs(*cmdSteer) < 0.1)//利用PID更广泛地控制速度
		{
			//速度较快且舵角较小时，使用PID控制，适用于平滑直道
			*cmdAcc = constrain(-2.0, 2.0, speedController.calculate1(currentSpeed)) * 800 / currentSpeed;

		}
		else if (abs(*cmdSteer) > 0.1)//起步油门
		{
			*cmdAcc = constrain(-2.0, 2.0, speedController.calculate1(currentSpeed)) / (abs(*cmdSteer) * 20);
			//*cmdAcc = 0.1;
		}
		//加速情况下，刹车为0
		*cmdBrake = 0;
		}
		else
		{
		*cmdAcc = 0;
		//不同减速情况下，刹车，间断轻踏过程



			//*cmdBrake  = constrain(0.0, 1.0, speedController.calculate2(currentSpeed))*sqrt( _speed / minCruve);
		*cmdBrake = 1.0;

		}
	
	if (_speed < 50 && minCurve>300)
		*cmdAcc = 1.0;
	//更新档位
	updateGear(cmdGear);

	//窗口可视化
	//cls_visual.Fig2Y(1, 0, 0.1, 0, 0.1, 10, "ferror", ferror, "cmdSteer", *cmdSteer, "cmdAcc", *cmdAcc);
	//cls_visual.Fig2Y(1, 0, 300, 0, 500, 10, "Target V", targetSpeed, "minCruve", minCurve, "Current V", _speed);
	//cls_visual.Fig2Y(2, -1, 1, 0, 500, 10, "Target Angel", targetAngleError, "minCurve", minCurve, "Current Angel", currentAngleError);

	count += 1;

	//帧率计数器
	if (timeCounter < 61) timeCounter++;//未检测出赛道前，帧率计数器增加

	if (!counterFinished && timeCounter == 60)//60帧时，对道路进行检测
	{
		counterFinished = true;
		printf("%d\n", timeCounter);
		if (_speed < 30) isDirt = true; //当60帧时，速度小于30km/h，则为土路
	}
}
