/******************************************************************************
*******************************************************************************
**
**
**					rovim.h - Description of the ROVIM sytem.
**
**		This module describes the whole ROVIM system, from the Dalf-1F 
**		motor control board firmware point of view.
**
**		It comprises, for the system and each of its subsystems, its
**		describing structures and definitions.
**		Different non-runtime system configurations are also referenced and
**		chosen here.
**
**		This code was designed originally for the Dalf-1F motor control
**		board, the brain of the T2D module.
**		Original Dalf-1F firmware revision was 1.73.
**		See Dalf-1F owner's manual and the ROVIM T2D documentation for more 
**		details.
**
**			The ROVIM Project
*******************************************************************************
******************************************************************************/

#ifndef __ROVIM_H
#define __ROVIM_H

//System description and default configuration
//#include "rovim_description.h"				// Description of the project
#include 	"rovim_t2d.h"						// Description of the T2D subsystem

#define INIT_VERBOSITY_LEVEL 0x07   //disable call info verbosity for now, due to the issue with the #line directive

//Project ID
#define ROVIM_T2D_SW_MAJOR_ID 1
#define ROVIM_T2D_SW_MINOR_ID 0
#define ROVIM_T2D_RELEASE_DATE "18-12-2015"
#define ROVIM_T2D_CONTACTS "Goncalo Andre (programmer): goncalofr87@gmail.\
com\r\nAntonio Serralheiro (superviser): ajserralheiro@gmail.com"

#endif /*__ROVIM_H*/
