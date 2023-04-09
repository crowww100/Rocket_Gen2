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
                void setRangeMin(int rangeMin = 200); // default value =200 middle = 350
                void setRangeMax(int rangeMax = 400); // default value = 500
                void setChannel(int channel = 0);        // default channel = 0
                void setServoPos(int pos = 0);
                void readReg(int reg= 0);
                void switchOff(void);
        private:
                int fd; //filedescriptor
                int channel;
                int rangeMin = 200;
                int rangeMax = 400;
                int result;
};

