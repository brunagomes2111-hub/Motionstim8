/***************************************************************************
                          serialport.h  -  description
                             -------------------
    begin                :
    copyright            : (C) 2003 Max Planck Institute for Dynamics
                                    of Complex Dynamical Systems
    Author               :  Nils-Otto Negård

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

#ifndef _QEXTSERIALPORT_H_
#define _QEXTSERIALPORT_H_

/*POSIX CODE*/
#ifdef _TTY_POSIX_
#include "serial_linux.h"
#define SerialPort serialport

#endif

#endif
