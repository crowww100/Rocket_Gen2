//declaration of variables////////////////////////////////////////////////////
struct HWT906_out
{
        double w_x;
	double w_y;
	double w_z;
        double h_x;
	double h_y;
	double h_z;
};

struct compass_calibration_data
{
	double x_min=9000;
        double x_max=-9000;
        double z_min=9000;
        double z_max=-9000;
	double y_min=9000;
	double y_max=-9000;
        double delta_x=0;
        double delta_z=0;
	double delta_y=0;
};

//declaration of functions////////////////////////////////////////////////////
int uart_open(int , const char*);
int uart_set(int, int, int, char, int);
int uart_close(int);
int send_data(int, char*, int);
int recv_data(int, char*, int);
struct HWT906_out ParseData(char);
double calc_comp_heading(struct compass_calibration_data,double,double,double);
