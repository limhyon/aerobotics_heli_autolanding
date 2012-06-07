#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <sys/signal.h>
#include <sys/poll.h>

#define BAUDRATE B115200
#define MODEMDEVICE "/dev/ttyS0"
#define _POSIX_SOURCE 1

#define FALSE 0
#define TRUE 1

#define HEADER1 0xFF
#define HEADER2 0xEA

#ifndef RAD2DEG
	#define RAD2DEG (180.0/M_PI)
#endif

#define PKT_LENGTH	28	

//include header, total 16bytes.
// header 2byte
// data 3*float(12) + 1byte = 13;
// chksum.

union
{
	float ft;
	unsigned char fti[4];
} p;

unsigned char toHeli[PKT_LENGTH] = {0x00};
double roll,pitch,yaw;
bool wait_flag = true;
char buf[0xff]= {0};

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

	p.ft = (float)roll;
	toHeli[14] = p.fti[3] & 0xff;
	toHeli[15] = p.fti[2] & 0xff;
	toHeli[16] = p.fti[1] & 0xff;
	toHeli[17] = p.fti[0] & 0xff;

	p.ft = (float)pitch;
	toHeli[18] = p.fti[3] & 0xff;
	toHeli[19] = p.fti[2] & 0xff;
	toHeli[20] = p.fti[1] & 0xff;
	toHeli[21] = p.fti[0] & 0xff;

	p.ft = (float)yaw;
	toHeli[22] = p.fti[3] & 0xff;
	toHeli[23] = p.fti[2] & 0xff;
	toHeli[24] = p.fti[1] & 0xff;
	toHeli[25] = p.fti[0] & 0xff;

	toHeli[26] = good;

	toHeli[27] = 0;

	for(int i=2;i<=26;i++)
	{
		toHeli[27] ^= toHeli[i];
	}
}

int main(int argc, char** argv)
{
	unsigned int count = 0;
	roll = pitch = yaw = 0.0;

	if(argc == 1)
	{
		ROS_ERROR("Specify serial port. e.g., /dev/ttyUSB0");
		return -1;
	}

	ros::init(argc, argv, "ar_listener");
	ros::NodeHandle node;
	tf::TransformListener listener;

	// Autopilot control status topic creation
	ros::Publisher control_status = node.advertise<std_msgs::Bool>("isauto",1000);
	 
	// Serial comm related
	int fd;
	struct termios newtio;

	struct pollfd poll_events;
	int poll_state;
	
	// open, we trust a user should enter /dev/ttyS0 or so.
	fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK); 

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

	// Poll for data receiving.
	poll_events.fd = fd;
	poll_events.events = POLLIN | POLLERR;
	poll_events.revents = 0;

	ros::Rate rate(100.0);
	bool success = true;

	while (node.ok())
	{

		poll_state = poll((struct pollfd*)&poll_events,1,0); // immediately return

		std_msgs::Bool isauto;
		tf::StampedTransform transform;
		tf::Quaternion attitude;
		success = true;
		ros::Time now = ros::Time::now();
		
		try
		{
			//listener.waitForTransform("/ar_marker","/pgr_camera_frame",now,ros::Duration(0.033));
			listener.lookupTransform("/ar_marker","/pgr_camera_frame",ros::Time(0),transform);
		}
		catch(tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			//write(fd,toHeli,16);
			//ros::spinOnce();
			//rate.sleep();
			//continue;
			success = false;
		}

		ros::Duration d = ros::Time::now() - transform.stamp_;

		if(d.toSec() > 0.5)
		{
			success = false;
		}

		if(success)
		{
			packData(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z(),1);
			attitude = transform.getRotation();
			btMatrix3x3(attitude).getRPY(roll,pitch,yaw);
			roll = roll*RAD2DEG;
			pitch = pitch*RAD2DEG;
			yaw = yaw*RAD2DEG;
			ROS_DEBUG("[%06d]:LOC(%f,%f,%f),RPY:(%f,%f,%f)",count++,transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z(),roll,pitch,yaw);
			
			isauto.data = true;
			control_status.publish(isauto);
		}
		else
		{
			packData(0.0,0.0,0.0,0);
		}

		if(poll_state > 0)
		{
			if(poll_events.revents & POLLIN)
			{
				int cnt = read(fd,buf,1);
				//ROS_INFO("Serial received! - %d %x",cnt,buf[0]);
			}

			if(poll_events.revents & POLLERR)
			{
				//ROS_INFO("Poll error!");
			}
			// res = read(fd,buf,255);
			// See http://www.faqs.org/docs/Linux-HOWTO/Serial-Programming-HOWTO.html
		}


		write(fd,toHeli,PKT_LENGTH);
		ros::spinOnce();
		rate.sleep();
	}

	close(fd);

	return 0;
}
