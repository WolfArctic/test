#include "aodom.h"

AOdometer::AOdometer(Vehicle_Model v)
:fl_(0.0),fr_(0.0),rl_(0.0),rr_(0.0),ax_(0.0),ay_(0.0),awz_(0.0),velocity_(0.0),heading_(0.0), 
fl_linear_(0.0), fr_linear_(0.0), rl_linear_(0.0), rr_linear_(0.0)
{
	vehicle.WHEEL_RADIUS_FORWARD_L = v.WHEEL_RADIUS_FORWARD_L;
	vehicle.WHEEL_RADIUS_FORWARD_R = v.WHEEL_RADIUS_FORWARD_R;
	vehicle.WHEEL_RADIUS_REAR_L = v.WHEEL_RADIUS_REAR_L;
	vehicle.WHEEL_RADIUS_REAR_R = v.WHEEL_RADIUS_REAR_R;
	vehicle.TRACK_WIDTH = v.TRACK_WIDTH;
	vehicle.WHEEL_BASE = v.WHEEL_BASE;

	init();
}

void AOdometer::SetVehicleModel(Vehicle_Model v)
{
	vehicle.WHEEL_RADIUS_FORWARD_L = v.WHEEL_RADIUS_FORWARD_L;
	vehicle.WHEEL_RADIUS_FORWARD_R = v.WHEEL_RADIUS_FORWARD_R;
	vehicle.WHEEL_RADIUS_REAR_L = v.WHEEL_RADIUS_REAR_L;
	vehicle.WHEEL_RADIUS_REAR_R = v.WHEEL_RADIUS_REAR_R;
	vehicle.TRACK_WIDTH = v.TRACK_WIDTH;
	vehicle.WHEEL_BASE = v.WHEEL_BASE;

	init();
}
void AOdometer::ProcessUsingAVG()
{
	fl_ = (fl_ + z_fl_)/2.0;
	fr_ = (fr_ + z_fr_)/2.0;
	rl_ = (rl_ + z_rl_)/2.0;
	rr_ = (rr_ + z_rr_)/2.0;
}
void AOdometer::ProcessUsingEKF()
{
	//fl_ = (fl_ + z_fl_)/2.0;
	//fr_ = (fr_ + z_fr_)/2.0;
	//rl_ = (rl_ + z_rl_)/2.0;
	//rr_ = (rr_ + z_rr_)/2.0;
	fl_ = z_rl_;
	fr_ = z_rr_;
	VectorXd my_z(2);
    my_z << z_rl_, z_rr_;
    step(my_z);
    VectorXd my_X(2);
    getX(my_X);
    rl_ = my_X(0);
    rr_ = my_X(1);
}
void AOdometer::ProcessAndUpdate()
{
	int choose = 0;
	if (choose == 1)
		ProcessUsingAVG();
	else
		ProcessUsingEKF();
	fl_linear_ = fl_ * vehicle.WHEEL_RADIUS_REAR_L;
	fr_linear_ = fr_ * vehicle.WHEEL_RADIUS_REAR_R;
	rl_linear_ = rl_ * vehicle.WHEEL_RADIUS_REAR_L;
	rr_linear_ = rr_ * vehicle.WHEEL_RADIUS_REAR_R;
	double left = rl_linear_;
	double right = rr_linear_;
	velocity_ = (right + left) / 2.0;
	awz_ = (right - left) / vehicle.TRACK_WIDTH;
}
void AOdometer::GetWheelVelocity(double &fl, double &fr, double &rl, double &rr)
{
	fl = fl_linear_;
	fr = fr_linear_;
	rl = rl_linear_;
	rr = rr_linear_;
}
void AOdometer::GetWheelVelocity(float &fl, float &fr, float &rl, float &rr)
{
	fl = (float)fl_linear_;
	fr = (float)fr_linear_;
	rl = (float)rl_linear_;
	rr = (float)rr_linear_;
}
void AOdometer::GetVehicleVelocity(double &velocity, double &awz)
{
	velocity = velocity_;
	awz = awz_;
}
void AOdometer::GetVehicleVelocity(float &velocity, float &awz)
{
	velocity = (float)velocity_;
	awz = (float)awz_;
}
void AOdometer::SetObservation(double fl, double fr, double rl, double rr)
{
	z_fl_ = fl;
	z_fr_ = fr;
	z_rl_ = rl;
	z_rr_ = rr;

	double fl_linear_temp = z_fl_ * vehicle.WHEEL_RADIUS_FORWARD_L;
	double fr_linear_temp = z_fr_ * vehicle.WHEEL_RADIUS_FORWARD_R;
	double rl_linear_temp = z_rl_ * vehicle.WHEEL_RADIUS_REAR_L;
	double rr_linear_temp = z_rr_ * vehicle.WHEEL_RADIUS_REAR_R;
	double threshold_A = 0.8;
	double threshold_B = 0.4;
}
void AOdometer::init(){
	n = 2, m = 2;
	VectorXd q_vector(2),r_vector(2);
    q_vector << 0.01,0.01;
    r_vector<< 0.1,0.1;
    X = VectorXd::Zero(n);
    I = MatrixXd::Identity(n,n);
    P_pre = MatrixXd::Zero(n, n);
    P_post = I * 0.1;
    F = MatrixXd::Zero(n, n);
    H = MatrixXd::Zero(m, n);
    R = MatrixXd::Zero(m, m);
    Q = MatrixXd::Zero(n, n);
    X_prdct = VectorXd::Zero(n);
    hX = VectorXd::Zero(m);
    Z_post = VectorXd::Zero(m);
    X_post = VectorXd::Zero(n);
    getF();
    getH();
    setR(r_vector);
    setQ(q_vector);
}
void AOdometer::step(Ref<VectorXd> Z){
    //cout<<"before f:"<<X.transpose()<<endl;
    f();
    //cout<<"after f:"<<X_prdct.transpose()<<endl;
    getF();
    getH();
    P_pre = F * P_post * F.transpose() + Q;
    MatrixXd G = MatrixXd::Zero(n, m);
    G = (P_pre * H.transpose()) * ((H * P_pre * H.transpose() + R).inverse());
    h();
    X_post = X;
    MatrixXd temp = G * (Z - hX);
    X = X_prdct + temp;
    P_post = (I - G * H) * P_pre;
}
bool AOdometer::f(){
    X_prdct(0) = X(0);
    X_prdct(1) = X(1);
    return true;
}
void AOdometer::getF(){
    F(0,0) = 1;    
    F(1,1) = 1;
}
bool AOdometer::h(){
    hX(0) = X_prdct(0);
    hX(1) = X_prdct(1);
    return true;
}
void AOdometer::getH(){
    H(0,0) = 1.0;
    H(1,1) = 1.0;
}
void AOdometer::setR(const Ref<VectorXd> r_){
    for(int i = 0; i < m; i++){
        R(i,i) = r_(i);
    }
}
void AOdometer::setr(int i, double val){
    R(i,i) = val;
}
double AOdometer::getr(int i){
    return R(i,i);
}
void AOdometer::setQ(const Ref<VectorXd> q_){
    for(int i = 0; i < n; i++){
        Q(i,i) = q_(i);
    }
}
void AOdometer::setX(const Ref<VectorXd> X_init){
    for(int i = 0; i < n; i++){
        X(i) = X_init(i);
    }
}
void AOdometer::getX(Ref<VectorXd> X_){
    X_ = X;
}
