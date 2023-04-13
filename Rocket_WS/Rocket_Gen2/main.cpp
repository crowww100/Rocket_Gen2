////////////////////////////////////     Rocket Control Gen 2    /////////////////////////////////
/* Project: VGHM (visual guided homing missile)													*/
/* develobed by: Elmar Kirchensteiner															*/
/* Project start: 12.2022																		*/
/*																								*/
/* Flight Procedure (variable stage)															*/
/* 0. Initialization of all systems																*/
/*																								*/
/* 1. Pre-launch phase (no pressure) --> stage = 1												*/
/*	- Activate data logging																		*/
/*	- Wing-Test																					*/
/*	- activate rescue system --> auto arm above 25m --> auto trigger below 20m					*/
/*	- Wait for GPS-Lock																			*/
/*	- Target check																				*/
/*		- Rotate Rocket untill "beep"															*/
/*		- crosscheck of orientation and calculated distance to target							*/
/*	- Bring wings to straight postition															*/
/*		- keep wing-ctrl on --> crosscheck for activated wing-ctrl								*/
/*																								*/
/* 1b. Fueling (automatic or manual) --> stage =1												*/
/*		- start clearance & activate datalogging via reedcontact --> 3x beep logging confirmed	*/
/*																								*/
/* 2. Rocket launch --> stage = 2																*/
/*	- send start clearance to rocket launcher													*/
/*	- keep wings activated and straight during rocket launch									*/
/*																								*/
/* 3. Thrust phase --> stage = 3																*/
/*	- keep wings activated and straight during thrust phase										*/
/*	- detect end of thrust phase (acceleration <0)												*/
/*																								*/
/* 4. climb phase --> stage =4																	*/
/*	- rotate rocket (roll) until bottom side is oriented to target								*/
/*	- wait for climb speed < x km/h																*/
/*																								*/
/* 5. Zenith-turn --> stage = 5																	*/
/*	- activate z-control (target corner = 100°)													*/
/*	- rotate until 100° (use integrated z-orientation since launche								*/
/*	- activate horizon scanner																	*/
/*																								*/
/* 6. Target approach (far) --> stage = 6														*/
/*	- set target for z-control to 135°															*/
/*		- if horizon = valid, set z-accumulation to 0											*/
/*		- if horizon != valid, use z-accumulation												*/
/*	- activate x-control (!!! separate compass-SW needed (135°))								*/
/*	- activate y-control (keep rocket parallel to horizon)										*/
/*																								*/
/* 7. Target approach (close)--> stage = 7														*/
/*	- set target for z-control to 180° (use z-accumulation from last valid horizon				*/
/*		if z-angle close to 180° set wings straight (z-control off)								*/
/*	- activate target-scanner																	*/
/*																								*/
/* 8. Target accquired --> stage = 8															*/
/*	- confirm target																			*/
/*	- activate x-ctrl (use target-info from visual guidance)									*/
/*	- activate z-ctrl (use target-info from visual guidance)									*/
/*																								*/
//////////////////////////////////////////////////////////////////////////////////////////////////


//includes________________________________________________________________________________________
#include <iostream>
#include <errno.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <strings.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include "PCA9685.h"
#include "HWT906.h"
#include "BN880.h"
#include <mqueue.h>
#include <curses.h>
#include <stdio.h>
#include <ncurses.h>

//defines_________________________________________________________________________________________
#define SERVER_QUEUE_NAME_1   "/queue_BN880"
#define SERVER_QUEUE_NAME_2   "/queue_horizon"
#define QUEUE_PERMISSIONS 0660
#define MAX_MESSAGES 1
#define MAX_MSG_SIZE 256
#define MSG_BUFFER_SIZE MAX_MSG_SIZE + 10
#define BAUD 9600//460800 //115200 for JY61 ,9600 for others
#define PI 3.14159265

using namespace std;

//function declaration____________________________________________________________________________
void initialization(void);
void compassCalibration(int);
void getHorizonData(mqd_t, char[]);
void getGPSsensorData(mqd_t, char[]);
void getMotionDataHWT906(void);
void writeLoggingData(void);
void calcNewServoPos(void);
void execute_stage_1(void);
void execute_stage_2(void);
void execute_stage_3(void);
void execute_stage_4(void);
void execute_stage_5(void);
void execute_stage_6(void);
void execute_stage_7(void);
void execute_stage_8(void);
//global virables_________________________________________________________________________________

//target data
double target_north_sec = 2194464;
double target_east_sec = 2635992;
double target_delta_north=0;
double target_delta_east=0;
double alpha_to_target=0;
double distance2target=0;
int onTargetTest=0;
int onTarget=0;
//variables for servos
int Servo_parachute_offset=0; //middleposition
int Servo_PL_offset=385;
int Servo_PR_offset=430;
int Servo_GU_offset=320;
int Servo_GD_offset=410;
int Servo_PL_pos;
int Servo_PR_pos;
int Servo_GU_pos;
int Servo_GD_pos;

//variables for control loops
double rot_prevent_p = -0.5;
double rot_y_p = 0.8;
int rot_prevent_out= 0;
int rot_y=0;

//variables for HWT906 (gyro, compass)
static int ret; static int fd;
char r_buf[1024];
struct HWT906_out HWT906_output;
struct compass_calibration_data C_C_D;
double e_y_heading_tmp;
double e_y_heading;
double Angle_x=0;
double Angle_y=0;
double compass_heading;
//double y_heading_target=45;

// variables for BN880(GPS)
struct BN880_out BN880_output;
struct timespec tm; //used for timedreceive
long posixReceive_ns = 10000000; //10ms = 10000000 at 50ms no timeout occure
int timeout_queue=0;
char north_min[2];
char north_sec[5];
char east_min[2];
char east_sec[5];
char GPS_height[3];

//variables for time
struct timeval start_time;
struct timeval end_time;
long long elapsedTime;

//create servo objects
Servo Servo_parachute;
Servo Servo_PL;
Servo Servo_PR;
Servo Servo_GU;
Servo Servo_GD;

//variables for horizon
char *data;
int horizon_data[4]; //roll, pitch, horizon_valid, last "/"

//variables for posix message queue
mqd_t qd_server_BN880, qd_server_horizon; // queue descriptor
char in_buffer [MSG_BUFFER_SIZE];
struct mq_attr attr;

//other variables
int stage = 0; //description in project description
char c_input;

//File for datarecorder
FILE *fp; //file for recording HWT906 data


//Main____________________________________________________________________________________________________________
int main (int argc, char **argv)
{
	//initialization and calibration______________________________________________________________
	initialization(); //initialization of all systems
	compassCalibration(12); //input value sets the calibration time in s

	gettimeofday(&start_time,NULL); //used as initial value for "getMotionDataHWT906"
	//main loop___________________________________________________________________________________
	while(1)
	{
		//Collecting all sensor data_________________________
		getHorizonData(qd_server_horizon,in_buffer);
		getGPSsensorData(qd_server_BN880,in_buffer);
		getMotionDataHWT906();

		//select & execute current stage
		switch (stage)
		{
			case 1: execute_stage_1(); break;
			case 2: execute_stage_2(); break;
			case 3: execute_stage_3(); break;
			case 4: execute_stage_4(); break;
			case 5: execute_stage_5(); break;
			case 6: execute_stage_6(); break;
			case 7: execute_stage_7(); break;
			case 8: execute_stage_8(); break;
			default: printf("invalid stage"); break;
		}
		//calc & set new servo positions

		//writing all data in the calculation loop to the logfile
		writeLoggingData();

		calcNewServoPos();

		printf("roll: %3d  pitch: %3d  valid: %1d | ", horizon_data[0], horizon_data[1], horizon_data[2]);
		printf("alpha2tar= %3f | ",alpha_to_target);
		printf("comp heading:  %3f ° | ",compass_heading);
		printf("y_rot_sum: %3f ° | ",Angle_y);
		printf("e_y: %f °   ",e_y_heading);
		printf("\n");
	}

	//clean up before exit/////////////////////////////////////////////////////////
/*
	ret = uart_close(fd);
	if(ret == -1)
	{
		fprintf(stderr,"uart_close error\n");
		exit(EXIT_FAILURE);
	}
	exit(EXIT_SUCCESS);
*/
}

//Rocket_functions________________________________________________________________________________________________
void initialization(void)
{
	//Initialisation wiringPi_________________________________________________________________________________
	wiringPiSetup(); //wiringPiSetup
	pinMode(5,OUTPUT); //beep sound
	pinMode(27,INPUT); //reedcontact
	digitalWrite(5,HIGH);
	usleep(10000);
	digitalWrite(5,LOW);

	//Initialization of message queue format
	attr.mq_flags = 0;
	attr.mq_maxmsg = MAX_MESSAGES;
	attr.mq_msgsize = MAX_MSG_SIZE;
	attr.mq_curmsgs = 0;

	//open message queues
	if ((qd_server_BN880 = mq_open (SERVER_QUEUE_NAME_1, O_RDONLY | O_CREAT, QUEUE_PERMISSIONS, &attr)) == -1)
	{
		perror ("Server: mq_open for BN880 (server)");
		exit (1);
	}
	else
	{
		printf("Start up procedure:   posix message queue for BN880 generated\n");
		usleep(1000000);
	}

	if ((qd_server_horizon = mq_open (SERVER_QUEUE_NAME_2, O_RDONLY | O_CREAT, QUEUE_PERMISSIONS, &attr)) == -1)
	{
		perror ("Server: mq_open for horizon (server)");
		exit (1);
	}
	else
	{
		printf("Start up procedure:   posix message queue for horiozon generated\n");
		usleep(1000000);
	}

	//Initialisation of servos_______________________________________________________________________________
	Servo_PL.init();
	Servo_PR.init();
	Servo_GU.init();
	Servo_GD.init();

	Servo_parachute.setRangeMin(170);
	Servo_parachute.setRangeMax(290);
	Servo_parachute.setChannel(0);

	Servo_PL.setRangeMin(Servo_PL_offset-120);
	Servo_PL.setRangeMax(Servo_PL_offset+120);
	Servo_PL.setChannel(8);

	Servo_PR.setRangeMin(Servo_PR_offset-120);
	Servo_PR.setRangeMax(Servo_PR_offset+120);
	Servo_PR.setChannel(10);

	Servo_GU.setRangeMin(Servo_GU_offset-120);
	Servo_GU.setRangeMax(Servo_GU_offset+120);
	Servo_GU.setChannel(11);

	Servo_GD.setRangeMin(Servo_GD_offset-120);
	Servo_GD.setRangeMax(Servo_GD_offset+120);
	Servo_GD.setChannel(9);
	
	Servo_parachute.switchOff();
	Servo_PL.switchOff();
	Servo_PR.switchOff();
	Servo_GU.switchOff();
	Servo_GD.switchOff();

	
	printf("Start up procedure:   servos initialised and switched off\n");
	usleep(1000000);


	//UART initialization for HTW906-Sensor______________________________________________________________
	fd = uart_open(fd,"/dev/ttyUSB0");
	bzero(r_buf,1024);
	if(fd == -1)
	{
		fprintf(stderr,"uart_open error\n");
		exit(EXIT_FAILURE);
	}
	if(uart_set(fd,BAUD,8,'N',1) == -1)
	{
		fprintf(stderr,"uart set failed!\n");
		exit(EXIT_FAILURE);
	}
	printf("Start up procedure:   UART initialised for HTW906-Gyro\n");
	usleep(1000000);

	//open file for data-logging_________________________________________________________________________
	//fp = fopen("Record.txt","w");
	printf("Start up procedure:   Initialization completed\n");
	printf("Start up procedure:   start GPS-Client manually, then press Enter\n");
	printf("press ENTER or reed contact________________________________________________________________________________\n");
	for(int i =0;i<5;i++)
		{
			digitalWrite(5,1);
			usleep(10000);
			digitalWrite(5,0);
			usleep(200000);
		}
	//c_input=getchar(); //wait for key press
	while(digitalRead(27))
	{
		//wait for reedcontact
	}
		for(int i =0;i<3;i++)
		{
			digitalWrite(5,1);
			usleep(10000);
			digitalWrite(5,0);
			usleep(300000);
		}
}

void compassCalibration(int caltime)
{
	usleep(3000000);
	elapsedTime=0;
	gettimeofday(&start_time,NULL);
	printf("Start up procedure:   start compass calibration --> rotate rocket around y-axes!\n");
	printf("Start up procedure:   Remaining calibration time:   %d s\n",caltime);

	while(elapsedTime<(caltime*1000000))
	{
		HWT906_output.h_x=0xffff;
		HWT906_output.h_y=0xffff;
		HWT906_output.h_z=0xffff;

		ret = recv_data(fd,r_buf,44); //4*11characters = 44
		if(ret == -1)
		{
			fprintf(stderr,"uart read failed!\n");
			exit(EXIT_FAILURE);
		}
		for (int i=0;i<ret;i++)
		{
			//fprintf(fp,"%2X ",r_buf[i]);  //write data to logfile Record.txt
			HWT906_output=ParseData(r_buf[i]);
		}
		if((HWT906_output.h_x!=0xffff)&& (HWT906_output.h_x!=0))
		{
			if(HWT906_output.h_x<C_C_D.x_min)
			{
				C_C_D.x_min = HWT906_output.h_x;
			}
			if(HWT906_output.h_x>C_C_D.x_max)
			{
				C_C_D.x_max = HWT906_output.h_x;
			}
		}
		if((HWT906_output.h_z!=0xffff) && (HWT906_output.h_z!=0))
		{
			if(HWT906_output.h_z<C_C_D.z_min)
			{
				C_C_D.z_min = HWT906_output.h_z;
			}
			if(HWT906_output.h_z>C_C_D.z_max)
			{
				C_C_D.z_max = HWT906_output.h_z;
			}
		}

		if((HWT906_output.h_y!=0xffff) && (HWT906_output.h_y!=0))
		{
			if(HWT906_output.h_y<C_C_D.y_min)
			{
				C_C_D.y_min = HWT906_output.h_y;
			}
			if(HWT906_output.h_y>C_C_D.y_max)
			{
				C_C_D.y_max = HWT906_output.h_y;
			}
		}

		gettimeofday(&end_time,NULL);
		elapsedTime = (end_time.tv_sec - start_time.tv_sec) * 1000000 + (end_time.tv_usec - start_time.tv_usec);
	}
	C_C_D.delta_x= C_C_D.x_max-C_C_D.x_min;
	C_C_D.delta_z= C_C_D.z_max-C_C_D.z_min;
	C_C_D.delta_y= C_C_D.y_max-C_C_D.y_min;
	printf("Start up procedure:   compass_x_min = %f   compass_x_max = %f \n",C_C_D.x_min, C_C_D.x_max);
	printf("Start up procedure:   compass_z_min = %f   compass_z_max = %f \n",C_C_D.z_min, C_C_D.z_max);
	printf("Start up procedure:   compass_y_min = %f   compass_y_max = %f \n",C_C_D.y_min, C_C_D.y_max);
	printf("Start up procedure:   compass_x_delta = %f   compass_z_delta = %f   compass_y_delta = %f \n",C_C_D.delta_x, C_C_D.delta_z, C_C_D.delta_y);
	printf("Start up procedure:   compass calibration completed\n");
	printf("press ENTER or reed contact________________________________________________________________________________\n");
	//c_input=getchar(); //wait for key press
	for(int i =0;i<3;i++)
		{
			digitalWrite(5,1);
			usleep(10000);
			digitalWrite(5,0);
			usleep(300000);
		}
	usleep(5000000);
	stage=1;
}

void getHorizonData(mqd_t qd_server_horizon_, char in_buffer_[]) //collect data from horizon detection module via IPC
{
	//set time for receivetimeout
	clock_gettime(CLOCK_REALTIME, &tm);
	timeout_queue=0;
	tm.tv_nsec +=  posixReceive_ns;

	// get the oldest message with highest priority
	//if (mq_receive (qd_server_horizon, in_buffer, MSG_BUFFER_SIZE, NULL) == -1) //alternative wo timeout
	if (mq_timedreceive(qd_server_horizon_, in_buffer_, MSG_BUFFER_SIZE, NULL, &tm) == -1)
	{
		//wait for message or timeout
		//perror ("Server: mq_receive");
		timeout_queue=1;
		//exit (1);
	}
	if(timeout_queue==0)
	{
		//printf("horizondata: %s \n",in_buffer);
		//decoding of message string received from mq_timedreceive
		data = strtok(in_buffer_, "/");
		int i=0;
		while(data != NULL)
		{
			horizon_data[i]=atoi(data);
			data = strtok(NULL, "/");
			i++;
		}
	}
	//printf("roll: %3d  pitch: %3d  valid: %1d | ", horizon_data[0], horizon_data[1], horizon_data[2]);
}

void getGPSsensorData(mqd_t qd_server_BN880_,char in_buffer_[]) //collect data from GPS-sensor via IPC
{
	//set time for receivetimeout
	clock_gettime(CLOCK_REALTIME, &tm);
	timeout_queue=0;
	tm.tv_nsec +=  posixReceive_ns;
	//tm.tv_sec += 1;

	// get the oldest message with highest priority
	//if (mq_receive (qd_server_BN880, in_buffer, MSG_BUFFER_SIZE, NULL) == -1) //alternative wo timeout
	if (mq_timedreceive(qd_server_BN880_, in_buffer_, MSG_BUFFER_SIZE, NULL, &tm) == -1)
	{
		//wait for message or timeout
		//perror ("Server: mq_receive");
		timeout_queue=1;
		//exit (1);
	}
	if(timeout_queue==0)
	{
		//printf("buffer: %s\n",in_buffer);
		for(int i=0;i<2;i++)
		{
			north_min[i]=in_buffer_[i];
		}
		for(int i=2;i<7;i++)
		{
			north_sec[i-2]=in_buffer_[i];
		}
		for(int i=7;i<9;i++)
		{
			east_min[i-7]=in_buffer_[i];
		}
		for(int i=9;i<14;i++)
		{
			east_sec[i-9]=in_buffer_[i];
		}
		for(int i=14;i<17;i++)
		{
			GPS_height[i-14]=in_buffer_[i];
		}
		BN880_output.north_sec = (atoi(north_min)*100000)+atoi(north_sec);
		BN880_output.east_sec = (atoi(east_min)*100000)+atoi(east_sec);
		BN880_output.GPS_height = atoi(GPS_height);
//		printf("north_sec = %d  east_sec = %d GPS_height = %d\n", BN880_output.north_sec, BN880_output.east_sec, BN880_output.GPS_height);
//		printf("\n");

		//calc alpha_to_target (ccw = positiv, staight north = 0)
		target_delta_north = target_north_sec - BN880_output.north_sec;
		target_delta_east = target_east_sec - BN880_output.east_sec;

		if(target_delta_north>=0 && target_delta_east>=0)
		{
			alpha_to_target= 2*PI-atan(target_delta_east/target_delta_north);
		}
		if(target_delta_north>=0 && target_delta_east<0)
		{
			alpha_to_target= atan(-target_delta_east/target_delta_north);
		}
		if(target_delta_north<0 && target_delta_east<0)
		{
			alpha_to_target= PI-atan(-target_delta_east/-target_delta_north);
		}
		if(target_delta_north<0 && target_delta_east>=0)
		{
			alpha_to_target= PI+atan(target_delta_east/target_delta_north);
		}
		alpha_to_target=alpha_to_target * 360/(2*PI);
		
		//calc distance2Target
		//Info: 1 sec_north = 0,0186m; 1sec_east = 0,0122m
		distance2target= sqrtf((target_delta_north*target_delta_north*0.00034596)+(target_delta_east*target_delta_east*0.00014884));

//		printf("delta_north_sec = %f delta_east_sec= %f alpha_to_target= %f\n", target_delta_north, target_delta_east,alpha_to_target);

	}

	//printf("delta_north_sec = %f delta_east_sec= %f alpha_to_target= %f\n", target_delta_north, target_delta_east,alpha_to_target);
	//printf("alpha2tar= %3f | ",alpha_to_target);
}

void getMotionDataHWT906(void)
{
	//fetch and parse motion data from HWT906____________________________________________________
	HWT906_output.h_x=0xffff;
	HWT906_output.h_y=0xffff;
	HWT906_output.h_z=0xffff;
	HWT906_output.w_x=0xffff;
	HWT906_output.w_y=0xffff;
	HWT906_output.w_z=0xffff;

	ret=0;
	while(ret==0)
	{
		ret = recv_data(fd,r_buf,44); //4*11characters = 44
	}

	if(ret == -1)
	{
		fprintf(stderr,"uart read failed!\n");
		exit(EXIT_FAILURE);
	}
	for (int i=0;i<ret;i++)
	{
		//fprintf(fp,"%2X ",r_buf[i]);  //write data to logfile Record.txt
		HWT906_output=ParseData(r_buf[i]);
	}
	gettimeofday(&end_time,NULL);
	elapsedTime = (end_time.tv_sec - start_time.tv_sec) * 1000000 + (end_time.tv_usec - start_time.tv_usec);
	//printf("w:%7.3f         ",wert);
	//printf("time: %lld us   ",elapsedTime);
	gettimeofday(&start_time,NULL);

	//printf("h_x =%f   ",HWT906_output.h_x);


	//calc compass heading______________________________________________________________________
	compass_heading= calc_comp_heading(C_C_D,HWT906_output.h_x, HWT906_output.h_z, HWT906_output.h_y);
	//printf("comp heading:  %3f ° | ",compass_heading);

	// calc Angle_y_____________________________________________________________________________
	Angle_y=Angle_y+((double)(elapsedTime)/1000000)*HWT906_output.w_y;
	//printf("winkelgeschwindigkeit %f         °/s",HWT906_output.w_y);
	if(Angle_y>=360)
	{
		Angle_y=Angle_y-(double)360;
	}
	if(Angle_y<0)
	{
		Angle_y=(double)360+Angle_y;
	}
	//printf("y_rot_sum: %3f ° | ",Angle_y);
}

void calcNewServoPos(void)
{
	//calc new servo position
	Servo_PL_pos=(Servo_PL_offset-rot_y+rot_prevent_out);
	Servo_PR_pos=(Servo_PR_offset-rot_y+rot_prevent_out);
	Servo_GU_pos=(Servo_GU_offset-rot_y+rot_prevent_out);
	Servo_GD_pos=(Servo_GD_offset-rot_y+rot_prevent_out);

	//set new servo pos
	Servo_PL.setServoPos(Servo_PL_pos); // ca 560 bis 150 --> 204 bis 408 (1ms bis 2 ms)
	Servo_PR.setServoPos(Servo_PR_pos);
	Servo_GU.setServoPos(Servo_GU_pos);
	Servo_GD.setServoPos(Servo_GD_pos);
}

void writeLoggingData(void)
{
	
}

void execute_stage_1(void)
{
	/* 1. Pre-launch phase (no pressure) --> stage = 1												*/
	/*	- Wing-Test																					*/
	/*	- activate rescue system --> auto arm above 25m --> auto trigger below 20m					*/
	/*	- Wait for GPS-Lock																			*/
	/*	- Target check																				*/
	/*		- Rotate Rocket untill "beep"															*/
	/*		- crosscheck of orientation and calculated distance to target							*/
	/*	- Bring wings to straight position															*/
	/*		- keep wing-ctrl on --> crosscheck for activated wing-ctrl								*/
	/*	- start clearance & activattion of data logging via reedcontact								*/
	/*																								*/

	//Set Servo to initial position and switch off
	Servo_PL.setServoPos(Servo_PL_offset); // ca 560 bis 150 --> 204 bis 408 (1ms bis 2 ms
	usleep(200000);
	Servo_PR.setServoPos(Servo_PR_offset);
	usleep(200000);
	Servo_GU.setServoPos(Servo_GU_offset);
	usleep(200000);
	Servo_GD.setServoPos(Servo_GD_offset);
	usleep(200000);
	Servo_parachute.switchOff();
	Servo_PL.switchOff();
	Servo_PR.switchOff();
	Servo_GU.switchOff();
	Servo_GD.switchOff();
	usleep(1000000); //wait for servo move to initial position
	
	//activate rescue system
	Servo_parachute.setServoPos(180); //bring parachute servo to lock position (180= lock; 280 = release)
	printf("Prelaunch phase Stage 1:   rescue system activated but not armed\n");
	usleep(1000000); //wait for servo move to initial position

	printf("Prelaunch phase Stage 1:   waiting for GPS-lock\n");
	usleep(5000000);
	while(BN880_output.north_sec==0)
	{
		getGPSsensorData(qd_server_BN880,in_buffer);
	}
	
	
	printf("Prelaunch phase Stage 1:   GPS-lock completed\n");
	printf("press ENTER________________________________________________________________________________________________\n");
	//c_input=getchar(); //wait for key press
	for(int i =0;i<3;i++)
	{
		digitalWrite(5,1);
		usleep(10000);
		digitalWrite(5,0);
		usleep(300000);
	}
	
	printf("Prelaunch phase Stage 1:   Rotate rocket around y until beep\n");
	printf("Prelaunch phase Stage 1:   Crosscheck if back faces to target and wings are straight ahead \n");


	while(!onTarget) // as long as back of rocket not facing to target
	{
		//Collecting all sensor data_________________________
		//getHorizonData(qd_server_horizon,in_buffer);
		//getGPSsensorData(qd_server_BN880,in_buffer);
		getMotionDataHWT906();
		//printf("test");
		// calc y_rotation (spin prevention)______________________________________________________________________________
		rot_prevent_out= (int)(HWT906_output.w_y*rot_prevent_p);

		// calc y_rotation (heading)________________________________________________________________

		e_y_heading_tmp=alpha_to_target -compass_heading;
		if((e_y_heading_tmp>=0 && e_y_heading_tmp<180)||(e_y_heading_tmp<0 && e_y_heading_tmp>-180))
		{
			e_y_heading = e_y_heading_tmp;
		}
		if(e_y_heading_tmp>180)
		{
			e_y_heading = -(360-e_y_heading_tmp);
		}
		if(e_y_heading_tmp<-180)
		{
			e_y_heading=360+e_y_heading_tmp;
		}
		printf("e_heading %f \n",e_y_heading);
		rot_y=(int)((e_y_heading)*rot_y_p);
		
		calcNewServoPos();

		//beep when e_y_heading absolut <2°
		if((e_y_heading < 2) && (e_y_heading > -2))
		{
				for(int i=0;i<400;i++)
				{
					digitalWrite(5,HIGH);
					usleep(2000);
					digitalWrite(5,LOW);
					usleep(3000);
				}
				onTarget=1;
		}
	}
	usleep(3000000);
	
	//calc distance to target
	printf("Prelaunch phase Stage 1:   Distance to target = %d m\n", (int)distance2target);
	printf("Prelaunch phase Stage 1:   Crosscheck if distance to target is valid --> confirm with ENTER\n");
	//c_input=getchar(); //wait for key press
	for(int i=0;i<((int)(distance2target/10));i++) //one beep for each 10m
	{
		digitalWrite(5,HIGH);
		usleep(2000);
		digitalWrite(5,LOW);
		usleep(500000);
	}
	
	//Set Servo to initial position and keep on
	Servo_PL.setServoPos(Servo_PL_offset); // ca 560 bis 150 --> 204 bis 408 (1ms bis 2 ms
	usleep(200000);
	Servo_PR.setServoPos(Servo_PR_offset);
	usleep(200000);
	Servo_GU.setServoPos(Servo_GU_offset);
	usleep(200000);
	Servo_GD.setServoPos(Servo_GD_offset);
	usleep(200000);
	printf("Prelaunch phase Stage 1:   Rocket is now initialized and calibrated, target data are confirmed\n");
	printf("Prelaunch phase Stage 1:   bring rocket into launch position and start fueling\n");
	printf("start clearance & acitvation of data logging via reedcontact___________________________________________________\n");
	usleep(3000000);
	for(int i =0;i<5;i++)
		{
			digitalWrite(5,1);
			usleep(10000);
			digitalWrite(5,0);
			usleep(200000);
		}
	while(digitalRead(27))
	{
		//wait for reedcontact
	}
		for(int i =0;i<3;i++)
		{
			digitalWrite(5,1);
			usleep(10000);
			digitalWrite(5,0);
			usleep(300000);
		}
	stage=2;

}

void execute_stage_2(void)
{
	while(1)
	{
		
	}
}

void execute_stage_3(void)
{
	
}

void execute_stage_4(void)
{
	
}

void execute_stage_5(void)
{
	
}

void execute_stage_6(void)
{
	
}

void execute_stage_7(void)
{
	
}

void execute_stage_8(void)
{
	
}