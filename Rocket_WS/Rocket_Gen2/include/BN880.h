//declaration of variables////////////////////////////////////////////////////
struct BN880_out
{
        int north_sec;
	int east_sec;
	int GPS_height;
};


//declaration of functions////////////////////////////////////////////////////
struct BN880_out getGPSdata(void);
int openUART(void);
