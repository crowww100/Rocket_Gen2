/******************************************************************************
	--> read GPS sensor BN880
	--> decode data
	--> and send data via posix message queue

	developer: 	Elmar Kirchensteiner
	date:		04.03.2023

******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <mqueue.h>
#include <unistd.h>

#define SERVER_QUEUE_NAME   "/queue_BN880"
#define QUEUE_PERMISSIONS 0660
#define MAX_MESSAGES 1
#define MAX_MSG_SIZE 256
#define MSG_BUFFER_SIZE MAX_MSG_SIZE + 10


int main ()
{
	int serial_port;
  	char dat,buff[100],buff_tx[17],GGA_code[3];
  	char buff_tmp[4];
  	int val=0;
  	unsigned char IsitGGAstring=0;
  	unsigned char GGA_index=0;
  	unsigned char is_GGA_received_completely = 0;

  	mqd_t qd_server; // queue descriptor

  	struct mq_attr attr;
	attr.mq_flags = 0;
  	attr.mq_maxmsg = MAX_MESSAGES;
  	attr.mq_msgsize = MAX_MSG_SIZE;
  	attr.mq_curmsgs = 0;

	//link qd_server to message_queue --> must be opened by server
  	if ((qd_server = mq_open (SERVER_QUEUE_NAME, O_WRONLY)) == -1)
  	{
  		perror ("Client: mq_open (server)");
        	exit (1);
  	}
  	else
  	{
		//printf("message queue open\n");
  	}

  	//open serial port for BN880 communication (set baudrate via "tool u-box"
	if ((serial_port = serialOpen ("/dev/ttyS0", 38400)) < 0)
  	{
    		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    	return 1 ;
  	}

	//initializes wiringPi setup
  	if (wiringPiSetup () == -1)
  	{
    		fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    	return 1 ;
  	}

	// main loop///////////////////////////////////////////////////////////////////////
 	while(1)
	{

		if(serialDataAvail (serial_port) )		//check for any data available on serial port
		{
			dat = serialGetchar(serial_port);	//receive character serially
			if(dat == '$')
			{
				IsitGGAstring = 0;
				GGA_index = 0;
			}
			else if(IsitGGAstring ==1)
			{
				buff[GGA_index++] = dat;
				if(dat=='\r')
				{
					is_GGA_received_completely = 1;
				}
			}
			else if(GGA_code[0]=='G' && GGA_code[1]=='G' && GGA_code[2]=='A')
			{
				IsitGGAstring = 1;
				GGA_code[0]= 0;
				GGA_code[0]= 0;
				GGA_code[0]= 0;
			}
			else
			{
				GGA_code[0] = GGA_code[1];
				GGA_code[1] = GGA_code[2];
				GGA_code[2] = dat;
			}
		}
		if(is_GGA_received_completely==1)
		{
			//usleep(5000);
			for(int i=12;i<14;i++)//format north_min,north_sek,east_min,east_sec,heigth withour spaces
			{
				buff_tx[i-12]=buff[i];
			}

			for(int i=15;i<20;i++)
			{
				buff_tx[i-13] = buff[i];
			}
			for(int i=26;i<28;i++)
                        {
                                buff_tx[i-19]=buff[i];
                        }

                        for(int i=29;i<34;i++)
                        {
                                buff_tx[i-20] = buff[i];
                        }
			for(int i=47;i<50;i++) //heigth
                        {
                                buff_tx[i-33] = buff[i];
                        }


			//printf("original = %s\n",buff);
                        printf(" tx buffer = %s \n",buff_tx);


			// send message to server
                        if (mq_send (qd_server, buff_tx, strlen(buff_tx), 0) == -1)
                        {
                                perror ("Client: Not able to send message to server");
                                continue;
                        }

			is_GGA_received_completely = 0;
		}
	}
	return 0;
}
