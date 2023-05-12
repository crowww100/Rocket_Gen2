#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include "HWT906.h"
#include <math.h>

#define PI 3.14159265


// variables_________________________________________________________________________
float a[3],w[3],Angle[3],h[3];
static struct HWT906_out ret;


// Functions_________________________________________________________________________

int uart_open(int fd,const char *pathname)
{
	fd = open(pathname, O_RDWR|O_NOCTTY);
	if (-1 == fd)
	{
		perror("Can't Open Serial Port");
		return(-1);
	}
	else
	{
		//printf("open %s success!\n",pathname);
	}
	if(isatty(STDIN_FILENO)==0)
	{
		printf("standard input is not a terminal device\n");
	}
	else
	{
		//printf("isatty success!\n");
	}
	return fd;
}

int uart_set(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if ( tcgetattr( fd,&oldtio) != 0)
	{
		perror("SetupSerial 1");
		printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio));
		return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	switch( nBits )
	{
		case 7:
		newtio.c_cflag |= CS7;
		break;
		case 8:
		newtio.c_cflag |= CS8;
		break;
	} 
	switch( nEvent )
	{
		case 'o':
		case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
		case 'e':
		case 'E':
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
		case 'n':
		case 'N':
		newtio.c_cflag &= ~PARENB;
		break;
		default:
		break;
	}

	switch( nSpeed )
	{
		case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
		case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
		case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
		case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
		case 460800:
		cfsetispeed(&newtio, B460800);
		cfsetospeed(&newtio, B460800);
		break;
		case 921600://eki
		cfsetispeed(&newtio, B921600);//eki
		cfsetospeed(&newtio, B921600);//eki
		break;

		default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}
	if( nStop == 1 )
      newtio.c_cflag &= ~CSTOPB;
     else if ( nStop == 2 )
      newtio.c_cflag |= CSTOPB;
     newtio.c_cc[VTIME] = 0;
     newtio.c_cc[VMIN] = 0;
     tcflush(fd,TCIFLUSH); if((tcsetattr(fd,TCSANOW,&newtio))!=0)
     {
      perror("com set error");
      return -1;
     } 
     //printf("set done!\n");
     return 0;
}
int uart_close(int fd) {
    assert(fd);
    close(fd);
    return 0;
}
int send_data(int fd, char *send_buffer,int length) {
	length=write(fd,send_buffer,length*sizeof(unsigned char));
	return length;
}
int recv_data(int fd, char* recv_buffer,int length) {
	length=read(fd,recv_buffer,length);
	return length;
}

struct HWT906_out ParseData(char chr)
{
		static char chrBuf[100];
		static unsigned char chrCnt=0;
		signed short sData[4];
		unsigned char i;
		char cTemp=0;

		//time_t now;
		chrBuf[chrCnt++]=chr;
		if (chrCnt<11) return(ret);
		for (i=0;i<10;i++)
		{
			cTemp+=chrBuf[i];
		}
		if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)||(cTemp!=chrBuf[10]))
		{
			printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);
			memcpy(&chrBuf[0],&chrBuf[1],10);
			chrCnt--;
			return(ret);
		}

		memcpy(&sData[0],&chrBuf[2],8);
		switch(chrBuf[1])
		{
				case 0x51:
					for (i=0;i<3;i++) 
					{
						a[i] = (float)sData[i]/32768.0*16.0;
					}
					//time(&now);
					//printf("\r\nT:%s a:%6.3f %6.3f %6.3f ",asctime(localtime(&now)),a[0],a[1],a[2]);
					ret.a_y=a[1];
					break;
				case 0x52:
					for (i=0;i<3;i++)
					{
						 w[i] = (float)sData[i]/32768.0*2000.0;
					}
					//printf("w:%7.3f %7.3f %7.3f ",w[0],w[1],w[2]);
					//printf("wv_roh %f     ", w[1]);
					ret.w_x=w[0]; 
					ret.w_y=w[1]; 
					ret.w_z=w[2];
					break;
				case 0x53:
					for (i=0;i<3;i++) 
					{
						Angle[i] = (float)sData[i]/32768.0*180.0;
					}
					//printf("A:%7.3f %7.3f %7.3f ",Angle[0],Angle[1],Angle[2]);
					break;
				case 0x54:
					for (i=0;i<3;i++) 
					{
						h[i] = (float)sData[i];
					}
					ret.h_x = h[0]; 
					ret.h_y = h[1]; 
					ret.h_z = h[2];
					//printf("Kompass_z: %4.0f     ",ret.h_z);
					//printf("h:%4.0f %4.0f %4.0f ",h[0],h[1],h[2]);
					break;
		}
		chrCnt=0;
		return(ret);
}

double calc_comp_heading(struct compass_calibration_data calibration_data, double h_x, double h_z, double h_y)
{

	double compass_x_norm;
	double compass_z_norm;
	double compass_y_norm;
	double alpha_north=0;
	//double y_tilt=0;
	//double reference=100;

	compass_x_norm=(h_x- calibration_data.x_min-calibration_data.delta_x/2)*200/ calibration_data.delta_x; //norm to +-100
	compass_z_norm=(h_z- calibration_data.z_min- calibration_data.delta_z/2)*200/ calibration_data.delta_z; //norm to +-100
	compass_y_norm=(h_y- calibration_data.y_min- calibration_data.delta_y/2)*200/ calibration_data.delta_y; //norm to +-100
	if (compass_x_norm>100) compass_x_norm=100;
	if (compass_x_norm<-100) compass_x_norm=-100;
	if (compass_z_norm>100) compass_z_norm=100;
	if (compass_z_norm<-100) compass_z_norm=-100;
	if (compass_y_norm>100) compass_y_norm=100;
	if (compass_y_norm<-100) compass_y_norm=-100;


	//correction of x and z "norm-values" due to tilt in y direction --> faulty due to non horizontal  magnetic field
	/*y_tilt=PI/2-acos(abs(compass_y_norm)/100); // absolut in rad
	y_tilt= y_tilt*360/(2*PI);
	printf("y_tilt= %f   ",y_tilt);
	reference= cos(y_tilt)*100; 
	*/

	//printf("compass_x_norm = %f   compass_z_norm = %f      \n ",compass_x_norm, compass_z_norm);

	if((compass_x_norm>=0)&&(compass_z_norm>=0))
	{
		alpha_north=atan(compass_x_norm/compass_z_norm);
		//alpha_north=atan(compass_z_norm/compass_x_norm); //neu
		/*if(compass_z_norm>=compass_x_norm) //old solution
		{
			alpha_north=asin(compass_x_norm/reference);
		}
		else
		{
			alpha_north=acos(compass_z_norm/reference);
		}*/
	}

	if((compass_x_norm>=0)&&(compass_z_norm<0))
	{
		alpha_north=atan(-compass_z_norm/compass_x_norm)+PI/2;
		//alpha_north=atan(-compass_z_norm/compass_x_norm)+3*PI/2;
		/*if(compass_x_norm>=-compass_z_norm) //old solution
		{
			alpha_north=PI/2+asin(-compass_z_norm/reference);
		}
		else
		{
			alpha_north=PI/2+acos(compass_x_norm/reference);
		}*/
	}

	if((compass_x_norm<0)&&(compass_z_norm<0))
	{
		alpha_north=atan(-compass_x_norm/-compass_z_norm)+PI;
		//alpha_north=atan(-compass_z_norm/-compass_x_norm)+PI; //neu
		/*if(-compass_z_norm>=-compass_x_norm) //old solution
		{
			alpha_north=PI+asin(-compass_x_norm/reference);
		}
		else
		{
			alpha_north=3*PI/2-asin(-compass_z_norm/reference);
		}*/
	}

	if((compass_x_norm<0)&&(compass_z_norm>=0))
	{
		alpha_north=atan(compass_z_norm/-compass_x_norm)+3*PI/2;
		//alpha_north=atan(compass_z_norm/-compass_x_norm)+PI/2; //neu
		/*if(compass_z_norm<-compass_x_norm) //old solution
		{
			alpha_north=3*PI/2+asin(compass_z_norm/reference);
		}
		else
		{
			alpha_north=2*PI-asin(-compass_x_norm/reference);
		}*/
	}
	
	alpha_north=alpha_north*360/(2*PI);
	alpha_north=360-alpha_north; //due to change of orientation of sensor
	//printf("alpha_north = %f    ",alpha_north);
	return alpha_north;
}
