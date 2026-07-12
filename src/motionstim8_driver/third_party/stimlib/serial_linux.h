/***************************************************************************
                          serial_linux.h  -  description
                             -------------------
    begin                :
    copyright            : (C) 2003 Max Planck Institute for Dynamics
                                    of Complex Dynamical Systems
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

#ifndef SERIALLINUX_H
#define SERIALLINUX_H

//static const  char* SerialPortList[]={"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2","/dev/ttyS3",0};
//static const  char* serialportList[]={"/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3",0};

static const  int NrofPortsInTheList=4;

#include <stdio.h>
class serialport
{
 public:
  serialport();
  ~serialport();
  
  //int serial_openport(char *port);
  bool serial_openport(char *port);
  int serial_fileid();
  
  int serial_setupport(long Baud_Rate);
  int serial_closeport();
  
  int serial_sendstring(char *buffer, size_t nb);
  
  int serial_readstring(char *buffer, size_t nb);

  int serial_read_timeout(char *buffer, size_t nb);
  int serial_flush();


  
 private:
  int fd;

  
};
#endif
