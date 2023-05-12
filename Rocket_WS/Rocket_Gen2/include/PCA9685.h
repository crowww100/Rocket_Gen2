//Servo control
//PCA9685.h

#include <iostream>
#include <errno.h>
#include <wiringPiI2C.h>
#include <unistd.h>
using namespace std;


class Servo
{
        public:
                Servo();  //constructor
                ~Servo(); //destructor
                void init(void);
                void setRangeMin(int rangeMin = 0xCCC); // 
                void setRangeMax(int rangeMax = 0x1998); // 
                void setChannel(int channel = 0);        
                void setServoPos(int pos = 0);
                void readReg(int reg= 0);
                void switchOff(void);
        private:
                int fd; //filedescriptor
                int channel;
                int rangeMin = 0xCCC;
                int rangeMax = 0x1998;
                int result;
};

