#ifndef __AODOMETER_H
#define __AODOMETER_H

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

using namespace std;
using namespace Eigen;

typedef struct vehicle_model{
    float TRACK_WIDTH;
    float WHEEL_BASE;
    float WHEEL_RADIUS_FORWARD_L;
    float WHEEL_RADIUS_FORWARD_R;
    float WHEEL_RADIUS_REAR_L;
    float WHEEL_RADIUS_REAR_R;
}Vehicle_Model;

class AOdometer{
public:
	AOdometer():fl_(0.0),fr_(0.0),rl_(0.0),rr_(0.0),ax_(0.0),ay_(0.0),awz_(0.0),velocity_(0.0),heading_(0.0),
				fl_linear_(0.0), fr_linear_(0.0), rl_linear_(0.0), rr_linear_(0.0){};
	AOdometer(Vehicle_Model v);
	void SetVehicleModel(Vehicle_Model v);
	void SetObservation(double fl, double fr, double rl, double rr);
	void GetWheelVelocity(double &fl, double &fr, double &rl, double &rr);
	void GetWheelVelocity(float &fl, float &fr, float &rl, float &rr);
	void GetVehicleVelocity(double &velocity, double &awz);
	void GetVehicleVelocity(float &velocity, float &awz);
	void ProcessUsingAVG();
	void ProcessUsingEKF();
	void ProcessAndUpdate();

	void init();
	void step(Ref<VectorXd> Z);
    bool f();
    bool h();
    void getF();
    void getH();
    void getX(Ref<VectorXd> X_);
    double getr(int i);
    void setR(const Ref<VectorXd> r_);
    void setr(int i, double val);
    void setq(int i, double val);
    void setQ(const Ref<VectorXd> q_);
    void setX(const Ref<VectorXd> X_init);
private:
	double fl_, fr_, rl_, rr_;
	double fl_linear_, fr_linear_, rl_linear_, rr_linear_;
	double z_fl_, z_fr_, z_rl_, z_rr_;
	double velocity_, heading_;
	double ax_, ay_, awz_;
	Vehicle_Model vehicle;

	MatrixXd P_pre, P_post;
    MatrixXd F, H;
    MatrixXd R, Q;
    MatrixXd I;
    VectorXd X;
    VectorXd X_prdct, hX, Z_post, X_post;
    VectorXd r, q;
    int n,m;
};

#endif