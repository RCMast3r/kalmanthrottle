#include <iostream>
#include <ostream>
#include <fstream>
#include <string>
#include "SimpleKalmanFilter.h"
#include <math.h>
#include <vector>
#include "matplotlibcpp.h"
using namespace std;

namespace plt = matplotlibcpp;

SimpleKalmanFilter::SimpleKalmanFilter(float mea_e, float est_e, float q)
{
  _err_measure=mea_e;
  _err_estimate=est_e;
  _q = q;
}

float SimpleKalmanFilter::updateEstimate(float mea)
{
  _kalman_gain = _err_estimate/(_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimate-_current_estimate)*_q;
  _last_estimate=_current_estimate;

  return _current_estimate;
}

void SimpleKalmanFilter::setMeasurementError(float mea_e)
{
  _err_measure=mea_e;
}

void SimpleKalmanFilter::setEstimateError(float est_e)
{
  _err_estimate=est_e;
}

void SimpleKalmanFilter::setProcessNoise(float q)
{
  _q=q;
}

float SimpleKalmanFilter::getKalmanGain() {
  return _kalman_gain;
}

float SimpleKalmanFilter::getEstimateError() {
  return _err_estimate;
}
double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
int main()
{
	ofstream out;
	out.open("filtered.txt");
	srand(time(0)); 
	//tune these 3 values
	SimpleKalmanFilter yyy = SimpleKalmanFilter(.04, .04, .085);
	
	ifstream in("test.txt");
	vector<double> yeet;
	vector<double> yeet2;
	vector<double> yeet3;
	for(int i=0;i<4500;i++){
		yeet2.push_back(i);
	}
	if (!in)
	{
		cout << "Cannot open input file!" << endl;
		return 1;
	}

	double test;
	int i =0;
	double estimated_value;
	while(in>>test){
		i++;
		yeet.push_back(test);
		double measured_value = test;
		//the estimated value is being updated every 5 measurements here


		if((i%5)==0){
			
			double test = yyy.updateEstimate(measured_value);
			if(test<=1 && test>.001){
				estimated_value=test;
			}
			else{
				estimated_value=estimated_value;
			}
			
		}



		else{
			estimated_value=estimated_value;
		}
		yeet3.push_back(estimated_value);
		//out<<estimated_value<<"\n";
		//cout<<test<<"\n";	
	}
	in.close();
	out.close();
	//system("pause");

	
	plt::figure_size(1920, 1080);
	plt::plot(yeet2, yeet);
	plt::plot(yeet2, yeet3);
	plt::xlim(500,1500);
	plt::title("Sample figure");
	plt::legend();
	plt::save("./basic.png");
}		