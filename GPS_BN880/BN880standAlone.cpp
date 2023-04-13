/*
	GPS Interfacing with Raspberry Pi using C (WiringPi Library)
	http://www.electronicwings.com
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>

int main ()
{
  int serial_port;
  char dat,buff[100],GGA_code[3];
  char buff_tmp[4];
  int val=0;
  unsigned char IsitGGAstring=0;
  unsigned char GGA_index=0;
  unsigned char is_GGA_received_completely = 0;

  if ((serial_port = serialOpen ("/dev/ttyS0", 38400)) < 0)		/* open serial port */
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  if (wiringPiSetup () == -1)							/* initializes wiringPi setup */
  {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    return 1 ;
  }

  while(1){

		if(serialDataAvail (serial_port) )		/* check for any data available on serial port */
		{
			dat = serialGetchar(serial_port);		/* receive character serially */
//			printf("character = %c \n",dat);
			if(dat == '$')
			{
//				printf("new\n");
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
				//printf("last else");
				GGA_code[0] = GGA_code[1];
				GGA_code[1] = GGA_code[2];
				GGA_code[2] = dat;
			}
		}
		if(is_GGA_received_completely==1)
		{
			
			for(int i=0;i<50;i++)
			{
				printf("%c ",buff[i]);
			}
			printf("   \n");

			for(int i=0;i<3;i++)
			{
				buff_tmp[i] = buff[i];
			}
			val=atoi(buff_tmp);




			printf(" Val=%d 2xVal= %d \n",val,(2*val));
			is_GGA_received_completely = 0;
		}
	}
	return 0;
}
