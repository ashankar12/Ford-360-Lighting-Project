/****************************************************************************
        Module:
        Master_Main_Service.c
     
        Notes:
   
        External Functions Required:

        Public Functions:
				
        
****************************************************************************/

// ######################################################################################################################################################################
// ---------------------------- Includes
// ######################################################################################################################################################################

#include "ES_Configure.h"
#include "ES_Framework.h"

// the common headers for C99 types
#include <stdint.h>
#include <stdbool.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

// the headers to access the TivaWare Library
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "ES_Port.h"

// bitdefs
#include "BITDEFS.H"

// CAN top layer
#include "MS_CAN_top_layer.h"

// ######################################################################################################################################################################
// ---------------------------- Module Definitions
// ######################################################################################################################################################################



// ######################################################################################################################################################################
// ---------------------------- Module Level Variables
// ######################################################################################################################################################################

// Priority Var
static uint8_t MyPriority;

// Use pointers so these can only exist in one place
static uint32_t My_Node_ID;          		// This node's ID
static uint8_t My_RX_Data[2];          	// This node's data store for incoming data
static uint8_t My_Remote_Data[2];      	// This node's data store for incoming data that was requested (master), or data that we will send on request (slave)
static uint8_t My_Current_Command[2];		// This node's current command

static uint8_t * p_My_RX_Data = &(My_RX_Data[0]);
static uint8_t * p_My_Remote_Data = &(My_Remote_Data[0]);
static uint8_t * p_My_Current_Command = &(My_Current_Command[0]);

// ######################################################################################################################################################################
// ---------------------------- Private Function Prototypes
// ######################################################################################################################################################################



// ######################################################################################################################################################################
// ---------------------------- Public Functions
// ######################################################################################################################################################################

/****************************************************************************
     Public Function
          Init_Master_Main_Service

     Description
          Initializes the master node

****************************************************************************/
bool Init_Master_Main_Service ( uint8_t Priority ) {
    ES_Event ThisEvent;

    // Initialize the MyPriority variable with the passed in parameter.
    MyPriority = Priority;

//    // Initialize the port line to receive start button signals (Port A, bit 7)
//    // Enable peripheral clock to Port A
//    HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R0;
//    while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R0) != SYSCTL_PRGPIO_R0)
//        ; // (wait for port to be ready)
//    // Enable bit 7 on Port A as digital I/O line
//    HWREG(GPIO_PORTA_BASE+GPIO_O_DEN) |= BIT7HI;
//    // Set data direction for bit 7 on Port A to be an input
//    HWREG(GPIO_PORTA_BASE+GPIO_O_DIR) &= BIT7LO;

//    // Sample the button port pin (bit 7) and use it to initialize LastButtonState
//    LastButtonState = HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + ALL_BITS)) & BIT7HI;
	
		// Set up our ID and stuff
		My_Node_ID = 0x01;
		My_Current_Command[0] = 0xf2;
		My_Current_Command[1] = 0x31;
	
		// Initialize CAN bus
		Initialize_CAN_Internal_Bus(&My_Node_ID, p_My_RX_Data, p_My_Remote_Data);

    // Start debounce timer (timer posts to ButtonDebounceSM)
    ES_Timer_InitTimer(MASTER_NODE_TIMER, 1000);

    // post the initial transition event
    ThisEvent.EventType = ES_INIT;
    if (ES_PostToService( MyPriority, ThisEvent) == true) {
        // End of initialization (return True)
        return true;
    } else {
        return false;
    }
}

/****************************************************************************
     Public Function
          Post_Master_Main_Service

     Description
          Post event to the master node

****************************************************************************/
bool Post_Master_Main_Service( ES_Event ThisEvent ) {
    return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
     Public Function
          Run_Master_Main_Service

     Description
          Post event to the master node

****************************************************************************/
ES_Event Run_Master_Main_Service( ES_Event ThisEvent ) {
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
	
		if (ThisEvent.EventType == ES_TIMEOUT)
		{
			printf("\r\nMaster Sending Data: %d", My_Remote_Data[0]);
			CAN_Master_Command_Slave(0x02, p_My_Current_Command);
			ES_Timer_InitTimer(MASTER_NODE_TIMER, 1000);
		}

    return ReturnEvent;
}

// ######################################################################################################################################################################
// ---------------------------- Private Functions
// ######################################################################################################################################################################
