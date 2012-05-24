#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B115200
#define MODEMDEVICE "/dev/ttyS0"
#define _POSIX_SOURCE 1

#define FALSE 0
#define TRUE 1

#define HEADER1 0xFF
#define HEADER2 0xEA

//include header, total 16bytes.
// header 2byte
// data 3*float(12) + 1byte = 13;
// chksum.

union
{
	float ft;
	unsigned char fti[4];
} p;

unsigned char toHeli[16] = {0x00};

void packData(float x,float y,float z,unsigned char good)
{
	toHeli[0] = HEADER1;
	toHeli[1] = HEADER2;
	
	p.ft = x;
	toHeli[2] = p.fti[3] & 0xff;
	toHeli[3] = p.fti[2] & 0xff;
	toHeli[4] = p.fti[1] & 0xff;
	toHeli[5] = p.fti[0] & 0xff;

	p.ft = y;
	toHeli[6] = p.fti[3] & 0xff;
	toHeli[7] = p.fti[2] & 0xff;
	toHeli[8] = p.fti[1] & 0xff;
	toHeli[9] = p.fti[0] & 0xff;

	p.ft = z;
	toHeli[10] = p.fti[3] & 0xff;
	toHeli[11] = p.fti[2] & 0xff;
	toHeli[12] = p.fti[1] & 0xff;
	toHeli[13] = p.fti[0] & 0xff;

	toHeli[14] = good;

	toHeli[15] = 0;

	for(int i=2;i<=14;i++)
	{
		toHeli[15] ^= toHeli[i];
	}
}

int main(int argc, char** argv)
{
	if(argc == 1)
	{
		ROS_ERROR("Specify serial port. e.g., /dev/ttyUSB0");
		return -1;
	}

	ros::init(argc, argv, "ar_listener");
	ros::NodeHandle node;
	tf::TransformListener listener;
	
	// Serial comm related
	int fd;
	struct termios newtio;
	
	// open, we trust a user should enter /dev/ttyS0 or so.
	fd = open(argv[1], O_RDWR | O_NOCTTY ); 
	if (fd <0) 
	{
		ROS_ERROR("Serial port open error %s",argv[1]);
		perror(MODEMDEVICE); 
		exit(-1); 
	}
	else
	{
		ROS_INFO("Connected to %s successfully.",argv[1]);
	}

	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

        /* 
          BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
          CS8     : 8n1 (8bit,no parity,1 stopbit)
          CLOCAL  : local connection, no modem contol
          CREAD   : enable receiving characters
        */
	newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;

        /*
          IGNPAR  : ignore bytes with parity errors
          otherwise make device raw (no other input processing)
        */
         newtio.c_iflag = IGNPAR;
         
        /*
         Raw output.
        */
         newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
	newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */

	tcflush(fd,TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);

	ros::Rate rate(10.0);

	while (node.ok())
	{
		tf::StampedTransform transform;

		try
		{
			ros::Time now = ros::Time::now();
			listener.waitForTransform("/ar_marker","/pgr_camera_frame",now,ros::Duration(0.033));
			listener.lookupTransform("/ar_marker","/pgr_camera_frame",now,transform);
		}
		catch(tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			packData(0.0,0.0,0.0,0);
			write(fd,toHeli,16);
			ros::spinOnce();
			continue;
		}

		ROS_INFO("(%f,%f,%f)",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
		packData(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z(),1);
		write(fd,toHeli,16);
		ros::spinOnce();
		rate.sleep();
	}

	close(fd);

	return 0;
}
