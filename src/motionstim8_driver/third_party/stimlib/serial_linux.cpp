/***************************************************************************
                          serial_linux.cpp  -  description
                             -------------------
    
    Author               : Henrik Gollee,
                           Nils-Otto Negård

    email                : negaard@mpi-magdburg.mpg.de
 ***************************************************************************/
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
/*!
\class SerialPort
\brief Serial interface class
\version 1.0
\author Nils-Otto Negård, Henrik Gollee

\warning This code is published under the GNU general public
licence GPL, which means that the code distributed here comes with no
warranty. For more details read the hole licence in the file COPYING.

*/

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include "serial_linux.h"

/*!
\fn SerialPort::SerialPort()
Constructor 
*/
serialport::serialport()
{
}
/*!
\fn SerialPort::~SerialPort()
Destructor

*/

serialport::~serialport()
{
}

/*!
\fn int SerialPort::serial_openport(char *port)
\param *port Name of port

Opens port and saves the file descriptor in the object.

Error code:
\verbatim
Error                  Explanation
----------------------------------------------------
0                      No error has occurred
1                      Error opening serial port
\endverbatim

*/


//int SerialPort::serial_openport(char *port)
bool serialport::serial_openport(char *port)
{
    printf("setting up serial port\n");
    this->fd = open(port, O_RDWR | O_NOCTTY);

    if (fd < 0) {    /* arithmetic test because the fd is regarded
		      * as a number */
	perror("Error: Invalid int while opening port");
    return 1;
    //return false;
    }
    return 0;
    //return true;


}


int serialport::serial_fileid()
{

    return this->fd;
}

/*!
\fn int SerialPort::serial_setupport(long Baud_Rate)

\param Baud_Rate The baud rate for the communication.
    
Setting up the serial port with the baud-rate given as parameter.

Error code:
\verbatim
Error                  Explanation
----------------------------------------------------
0                      No error has occurred
1                      Error setting up serial port
\endverbatim
*/

int serialport::serial_setupport(long Baud_Rate)
{
    struct termios tio;
    int fSuccess;
    long BAUD;
    //this->fd=3;
    /* Get the current configuration. */
    fSuccess = tcgetattr(this->fd, &tio);
    if (fSuccess > 0) {
	perror("Error getting COM state: ");
	return fSuccess;
    }
    //  printf("testing");
    /*
     * Fill in the default values
     */
    memset(&tio, 0, sizeof(tio));

    switch (Baud_Rate)
      {
      case 115200:
      default:
	BAUD = B115200;
	break;
      case 19200:
	BAUD  = B19200;
	break;
      case 9600:
	BAUD  = B9600;
	break;
      }  //end of switch baud_rate


    //    tio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    tio.c_cflag = BAUD | CS8 | CLOCAL | CREAD;


    //tio.c_iflag = IGNPAR ;
     tio.c_iflag = IGNPAR | IGNBRK;
    
    tio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    tio.c_lflag = 0;

    tio.c_cc[VTIME] = 10/100;
    tio.c_cc[VMIN] = 1;

    //fcntl(fd,F_SETFL,FNDELAY);
 
   
    /* Set baud-rate. 
     * If the input baud rate is set to  zero,  the 
     * input baud rate will be equal to the output baud rate. 
     */
    /* cfsetospeed(&tio, B9600);
    cfsetispeed(&tio, B0); */

    /* Set the new configuration */
    tcflush(this->fd, TCIFLUSH);
    fSuccess = tcsetattr(this->fd, TCSANOW, &tio);
    if (fSuccess > 0) {
	perror("Error setting serial port state: ");
	return fSuccess;
    }
    return fSuccess;
}


/*!
\fn int SerialPort::serial_closeport()

Closing the serial port.

Error code:
\verbatim
Error                  Explanation
--------------------------------------------------
0                      No error has occurred
1                      Error closing serial port
\endverbatim
*/

int serialport::serial_closeport()
{
    int fSuccess;

    fSuccess = close(this->fd);

    if (fSuccess > 0) {
	perror("Error closing serial port: ");
    }
    return fSuccess;
}


/*!
\fn int SerialPort::serial_sendstring(char *buffer, size_t nb)

\param *buffer String of chars to be sent to the serial port.
\param nb Number of bytes to be sent to the serial port.

This function is sending a bytes to the serial port. 

Error code:
\verbatim
Error                  Explanation
------------------------------------------------------
0                      No error has occurred
1                      Error writing to the serial port
\endverbatim
*/


int serialport::serial_sendstring(char *buffer, size_t nb)
{
    int fSuccess;

    fSuccess = write(this->fd, (const void *) buffer, nb);

    if (fSuccess < 0) {
	perror("Error writing to serial port [sendstring]: ");
    }
    return fSuccess;
}


/*!
\fn  int SerialPort::serial_readstring(char *buffer, size_t nb)

\param *buffer String read from serial port
\param nb Number of bytes to be read from serial port.

This function is reading a string of the length nb from the serial port.

Error code:
\verbatim
Error                  Explanation
----------------------------------------------------
0                      No error has occurred
1                      Error reading serial port
\endverbatim
*/
int serialport::serial_readstring(char *buffer, size_t nb)
{
    fd_set rfds;
    struct timeval tv;
    int input_available;

    size_t chrs_read;
    
    FD_ZERO(&rfds);
    FD_SET(this->fd, &rfds);

    /* Wait up to five seconds. */
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    input_available = select(this->fd + 1, &rfds, NULL, NULL, &tv);
    //input_available=1;
    /* Don't rely on the value of tv now! */
    if (input_available)
      /*
      {
	buffer[0]=0xC1;
	return 1;
      }
      */ 
      {
        fcntl(this->fd,F_SETFL,FNDELAY);
	//    fcntl(this->fd,F_SETFL,0);  
       chrs_read = read(this->fd, (void *) buffer, nb);
       sleep(2);
          //printf("reading\n");
       if (chrs_read < 0) {
      	    perror("Error reading from serial port: ");
       }
       return chrs_read;
    } else {
	return 0;
    }
    
}

/*!
\fn  int SerialPort::serial_flush()

This function is flushing all buffers.

Error code:
\verbatim
Error                  Explanation
----------------------------------------------------
0                      No error has occurred
1                      Error reading serial port
\endverbatim
*/

int serialport::serial_flush()
{
  tcflush(this->fd, TCIOFLUSH);
  return 0;
}


/*!
\fn  int SerialPort::serial_readstring(char *buffer, size_t nb)

\param *buffer String read from serial port
\param nb Number of bytes to be read from serial port.

This function is reading a string of the length nb from the serial port.

Error code:
\verbatim
Error                  Explanation
----------------------------------------------------
0                      No error has occurred
1                      Error reading serial port
\endverbatim
*/
int serialport::serial_read_timeout(char *buffer, size_t nb)
{
    fd_set rfds;
    struct timeval tv;
    int input_available;
    int flags;
    size_t chrs_read;
    int still_to_read,total_read,time1,time2;

    if( (flags = fcntl(this->fd, F_GETFL, 0)) < 0)
      printf("ERROR");


   /*
    * O_NONBLOCK flag loeschen und file-status-flags neu setzen
    */

   if(fcntl(this->fd, F_SETFL, flags & ~O_NONBLOCK) < 0)
     printf("ERROR");

   //usleep(10);

   FD_ZERO(&rfds);
   FD_SET(this->fd, &rfds);

   /* Wait up to one seconds. */
   tv.tv_sec = 1;
   tv.tv_usec = 0;

   chrs_read=0;
   total_read=0;
   still_to_read=nb;

   //time1=mu_time();

   while ((total_read<nb)&&(select(this->fd + 1, &rfds, NULL, NULL, &tv) == 1))
     {
       chrs_read = read(this->fd, (void *) &buffer[total_read], still_to_read);
       total_read=total_read+chrs_read;
       //printf("nb %d total_read %d chrs_read %d\n",nb,total_read,chrs_read);
       still_to_read=still_to_read-chrs_read;
       if (chrs_read < 0)
     {
       perror("Error reading from serial port: ");
       total_read=-1;
     }
     }

   //time2=mu_time();
   //printf("TimeDiff %d\n",time2-time1);
   return total_read;
}

