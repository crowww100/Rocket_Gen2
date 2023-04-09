#include <iostream>
#include <errno.h>
extern "C" {
	#include <wiringPiI2C.h>
}
#include <time.h>
#include <unistd.h>
#include "PCA9685.h"
using namespace std;


Servo::Servo()
{
        //cout << "object Servo generated" << endl;

}

Servo::~Servo()
{
        wiringPiI2CWriteReg8(fd, (channel*4+6), 0x00);
        wiringPiI2CWriteReg8(fd, (channel*4+7), 0x00);
        wiringPiI2CWriteReg8(fd, (channel*4+8), 0x00);
        wiringPiI2CWriteReg8(fd, (channel*4+9), 0x00);
        cout << "position set to 0" << endl;

        cout << "object Servo destroyed" << endl;
}

void Servo::init()
{
        // Initialize the interface by giving it an external device ID.
        // The PCA9685 defaults to address 0x40.
        // It returns a standard file descriptor.
        fd = wiringPiI2CSetup(0x40);

        //configuration of PCA9685
        result = wiringPiI2CWriteReg8(fd,0xFE, 121); //Set PRE-SCALE register to 121 for 50Hz
        if(result == -1)
        {
                cout << "Error.  Errno is: " << errno << endl;
        }
}

void Servo::switchOff(void)
{
        wiringPiI2CWriteReg8(fd, (channel*4+6), 0x00);
        wiringPiI2CWriteReg8(fd, (channel*4+7), 0x00);
        wiringPiI2CWriteReg8(fd, (channel*4+8), 0x00);
        wiringPiI2CWriteReg8(fd, (channel*4+9), 0x00);
        //cout << "position set" << endl;
}

void Servo::setRangeMin(int rangeMin_)
{
        rangeMin = rangeMin_;
        //cout << "rangeMin set" << endl;
}

void Servo::setRangeMax(int rangeMax_)
{
        rangeMax = rangeMax_;
        //cout << "rangeMax set" << endl;
}

void Servo::setChannel(int channel_)
{
        channel = channel_;
        //cout << "channel set" << endl;
}

void Servo::setServoPos(int pos)
{
        //set pos into range
        if(pos<rangeMin) pos = rangeMin;
        if(pos>rangeMax) pos = rangeMax;


        //Set PWM
        wiringPiI2CWriteReg8(fd, (channel*4+6), 0x00);
        wiringPiI2CWriteReg8(fd, (channel*4+7), 0x00);
        wiringPiI2CWriteReg8(fd, (channel*4+8), (pos & 0xFF));
        wiringPiI2CWriteReg8(fd, (channel*4+9), ((pos >> 0x08) & 0xFF));

//      cout << "position set" << endl;
}

void Servo::readReg(int reg)
{
        cout << "Registervalue" << wiringPiI2CReadReg8(fd, reg) << endl;
}

