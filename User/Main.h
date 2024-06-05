
/******************************************************************************
 * @file 		Main.h
 * @brief		main source file for Main Controller project - STM32F4 chip
 * @details		This class is used to configure and use the Main project:
 * @author		Ronchi - Perletti \n
 *  General Medical Merate - GMM.spa - Seriate - ITALY
 * @version		1.0
 * @date		February 4th, 2014
 * @pre			Initialize and Enable the Serial class for the communication
 * @post		Nope
 * @bug			Not all memory is freed when deleting an object of this class.
 * @warning		Improper use can crash your application
 * @copyright 	GMM.spa - All Rights Reserved
 *
 ******************************************************************************/

#ifdef _USB_DEBUG_
#endif
#define _FW_VER_MAJOR_ 	     ( 93 )
#define _FW_VER_MINOR_	     ( 91 )
#define MMM_DD_YYYY_DATE     ("Jul 20 2023") // "May 30 2018" "May  8 2018"
// FW name MC_XX_YY_ZZ_OD4000-PA (Ats Like)�
// FW name MC_XX_YY_ZZ_OD4000-PD (Dll )
long getFwVersion(void);
long monthBuild( const char * u8aDate);
void MainAutoma_PID_Init(void);
void MainAutoma_GEN_Init(void);
void MainAutoma_CALYPSO_Init(void);

void MainInizialization(void);

