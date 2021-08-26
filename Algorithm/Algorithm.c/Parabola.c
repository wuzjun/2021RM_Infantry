/**
 * @file Parabola.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 弹道拟合计算
 * @version 0.1
 * @date 2021-04-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Parabola.h"

#define  angeTohudu(a)  radians(a)
#define  huduToangle(a) degrees(a)

bool Parabola_Matching(double angle_P, double d, double v0, double *result) {

	double  d2 = 0.118;//发射机构的半径
	double  h = 0;
	double g = 9.7833;//广东的重力加速度

	//角度，设向上抬为正方向。
	//double  angle_P = angeTohudu(90);//测量时方向和重力的夹角
	double  angle_A = 0;//向上的补偿角度

	double  h1 = d * cos(angle_P);


	double ans = 0;
	double last_ans = 0;
	double jindu = 0.8;//当前的测试精度

	//判断最远射程是否有解。
	//double L = cos(angle_P)*d;
	//double t = L / v0 * cos();


	double diffs = 0;
	double direction = 1;//默认增加角度。

	int error = 0; //计算过程中发生错误！
	do
	{
		angle_A += direction * PI / (180 * jindu);
		double angle_f = angle_A + angle_P;//最终与竖直方向的夹角

		double L = sin(angle_P)*d;

		double vx = v0 * sin(angle_f);
		double vy = v0 * cos(angle_f);
		double t = L / vx;//假设x 能到达，计算时间t
		double h2 = d2 * cos(angle_P) - d2 * cos(angle_f);//补偿后增加的高度
		h = h1 + h2;
		double Cal_h = vy * t + 0.5*g*t*t;

		//double zuo = -1 * v0*tan(angle_A - angle_P)*d*cos(angle_P) + (0.5*g*pow(d*cos(angle_P), 2) / (pow(v0*cos(angle_A - angle_P), 2)));
		//double you = d * sin(angle_P);

		last_ans = ans;
		ans = h - Cal_h;

		diffs = abs(ans) - abs(last_ans);
		if (diffs > 0 && last_ans != 0)//误差在减少
		{
			direction *= -1;
			//cout << "方向=" << direction << endl;
			jindu += 1;
			if (jindu > 20)
			{

				//cout << "精度超限！" << endl;
				printf("精度超限！\n");
				break;
			}
		}


		if (L < 0 || vx < 0 || angle_f>angeTohudu(180) || t < 0)
		{
			error = 1;//发生计算错误！
			/*cout << "计算错误！" << endl
				<< vx << endl
				<< L << endl
				<< h1 << endl
				<< angle_A << endl
				<< h << endl;*/

			printf("vx=%.2f\nL=%.2f\nh1=%.2f\naA=%.2f\nh=%.2f\naP=%.2f\n", vx, L, h1, angle_A, h, huduToangle(angle_P));

			return false;
		}
	} while (abs(ans) >= 0.01 && error == 0);//容忍的误差大小：单位米

	//cout << ans << "==度数=" << huduToangle(angle_A) << endl;
	*result = huduToangle(angle_A);
	return true;
}
