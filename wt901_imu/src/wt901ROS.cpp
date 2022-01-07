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
#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/Imu.h>

static int ret;
static int fd;

#define BAUD 115200 //115200 for JY61 ,9600 for others

int uart_open(int fd,const char *pathname)
{
    fd = open(pathname, O_RDWR|O_NOCTTY);
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
    return(-1);
  }
    else
    printf("open %s success!\n",pathname);
    if(isatty(STDIN_FILENO)==0)
    printf("standard input is not a terminal device\n");
    else
    printf("isatty success!\n");
    return fd;
}

int uart_set(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
     struct termios newtio,oldtio;
     if  ( tcgetattr( fd,&oldtio)  !=  0) {
      perror("SetupSerial 1");
    printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio));
      return -1;
     }
     bzero( &newtio, sizeof( newtio ) );
     newtio.c_cflag  |=  CLOCAL | CREAD;
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

     /*设置波特率*/

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
     default:
      cfsetispeed(&newtio, B9600);
      cfsetospeed(&newtio, B9600);
     break;
     }
     if( nStop == 1 )
      newtio.c_cflag &=  ~CSTOPB;
     else if ( nStop == 2 )
      newtio.c_cflag |=  CSTOPB;
     newtio.c_cc[VTIME]  = 0;
     newtio.c_cc[VMIN] = 0;
     tcflush(fd,TCIFLUSH);

if((tcsetattr(fd,TCSANOW,&newtio))!=0)
     {
      perror("com set error");
      return -1;
     }
     printf("set done!\n");
     return 0;
}

int uart_close(int fd)
{
    assert(fd);
    close(fd);

    return 0;
}
int send_data(int  fd, char *send_buffer,int length)
{
  length=write(fd,send_buffer,length*sizeof(unsigned char));
  return length;
}
int recv_data(int fd, char* recv_buffer,int length)
{
  length=read(fd,recv_buffer,length);
  return length;
}

float a[3],w[3],Angle[3],h[3], q[4];
sensor_msgs::Imu imu_msg;
void ParseData(char chr)
{
    static char chrBuf[100];
    static unsigned char chrCnt=0;
    signed short sData[4];
    unsigned char i;
    char cTemp=0;
    time_t now;
    chrBuf[chrCnt++]=chr;
    if (chrCnt<11) return;
    for (i=0;i<10;i++) cTemp+=chrBuf[i];
    if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)||(cTemp!=chrBuf[10]))
    {
      printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);
      memcpy(&chrBuf[0],&chrBuf[1],10);
      chrCnt--;
      return;
    }

    memcpy(&sData[0],&chrBuf[2],8);
    switch(chrBuf[1])
    {
        case 0x51: //acceleration output
          for (i=0;i<3;i++) a[i] = (float)sData[i]/32768.0*16.0;
          time(&now);
          imu_msg.linear_acceleration.y = boost::lexical_cast<double>(a[0]);
          imu_msg.linear_acceleration.x = boost::lexical_cast<double>(a[1]);
          imu_msg.linear_acceleration.z = -boost::lexical_cast<double>(a[2]);
          break;
        case 0x52: //angular velocity output
          for (i=0;i<3;i++) w[i] = (float)sData[i]/32768.0*2000.0;
          imu_msg.angular_velocity.y = boost::lexical_cast<double>(w[0]);
          imu_msg.angular_velocity.x = boost::lexical_cast<double>(w[1]);
          imu_msg.angular_velocity.z = -boost::lexical_cast<double>(w[2]);
          break;
        case 0x53: //euler angle output
          for (i=0;i<3;i++) Angle[i] = (float)sData[i]/32768.0*180.0;
          //printf("A:%7.3f %7.3f %7.3f ",Angle[0],Angle[1],Angle[2]);
          break;
        case 0x59: //quaternion output
          for (i=0;i<4;i++) q[i] = (float)sData[i]/32768.0;
          imu_msg.orientation.y = boost::lexical_cast<double>(q[0]);
          imu_msg.orientation.x = boost::lexical_cast<double>(q[1]);
          imu_msg.orientation.z = -boost::lexical_cast<double>(q[2]);
          imu_msg.orientation.w = boost::lexical_cast<double>(q[3]);
          break;
    }
    chrCnt=0;
}

int main(int argc, char **argv)
{
    char r_buf[1024];
    bzero(r_buf,1024);

    fd = uart_open(fd,"/dev/wt901IMU");/*串口号/dev/ttySn,USB口号/dev/ttyUSBn */
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
    ros::init(argc, argv, "wt901_imu_NODE");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 1);
    ros::Rate r(50);
    imu_msg.header.frame_id = "wt901_imu";

    while(ros::ok() && !ros::isShuttingDown())
    {
        ret = recv_data(fd,r_buf,44);
        if(ret == -1)
        {
            fprintf(stderr,"uart read failed!\n");
            exit(EXIT_FAILURE);
        }
        for (int i=0;i<ret;i++) {ParseData(r_buf[i]);}
        imu_msg.header.stamp = ros::Time::now();
        imu_pub.publish(imu_msg);
        ros::spinOnce();
        r.sleep();
    }

    ret = uart_close(fd);
    if(ret == -1)
    {
        fprintf(stderr,"uart_close error\n");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}
