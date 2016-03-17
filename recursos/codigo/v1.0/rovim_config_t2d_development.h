/******************************************************************************
*******************************************************************************
**
**
**                  rovim_config_v0.1.h - Configuration of the ROVIM
**                          system for the version 0.1 of the ROVIM
**                          T2D software.
**
**      This file holds the non-runtime software configuration profile of
**      the ROVIM system for the defined software version. It may be used
**      for other versions too, as long as it is unchanged.
**
**      This code was designed originally for the Dalf-1F motor control
**      board, the brain of the T2D module.
**      Original Dalf-1F firmware revision was 1.73.
**      See Dalf-1F owner's manual and the ROVIM T2D documentation for more 
**      details.
**
**          The ROVIM Project
*******************************************************************************
******************************************************************************/

#ifndef __ROVIM_CONFIG_V0_1_H
#define __ROVIM_CONFIG_V0_1_H

//use maximum verbosity
#define INIT_VERBOSITY_LEVEL 0x0F   //disable call info verbosity for now, due to the issue with the #line directive

#endif /*__ROVIM_CONFIG_V0_1_H*/
