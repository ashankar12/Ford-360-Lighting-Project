/****************************************************************************
        Module:
        MS_CAN_top_layer.c
     
        Notes:
        This module implements the extended CAN 2.0b protocol with the 29-bit identifiers.

        Between the 360 lighting master and slave nodes, we will encode all of our data within the 29-bit identifiers

   
        External Functions Required:

        Public Functions:
        	void Initialize_CAN_Internal_Bus(uint8_t * p_this_node_id, uint8_t * p_rx_data, uint8_t * p_remote_data)
        	void CAN_Master_Command_Slave(uint32_t slave_id, uint8_t * p_cmd_data)
        	void CAN_Master_Request_Slave(uint32_t slave_id)
        	void CAN_Slave_Send_Master(uint8_t * p_slave_data)
        	void CAN_Internal_Bus_ISR(void)
        
****************************************************************************/

// ######################################################################################################################################################################
// ---------------------------- Includes
// ######################################################################################################################################################################

/* Standard ANSI  99 C types for exact integer sizes*/
#ifdef COMPILER_IS_C99
#include <stdint.h>
#else  /* use the open source version included with ES framework */
#include "stdint.h"
#endif

/* Standard ANSI  99 C types for booleans, true & false*/
/* must come after stdint.h because the emulation uses uint8_t */
#ifdef COMPILER_IS_C99
#include <stdbool.h>
#else   // provide C99 compliant definitions
#include "stdbool.h"
#endif

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"  // Define PART_TM4C123GH6PM in project
#include "driverlib/gpio.h"

#include "MS_CAN_top_layer.h"
#include "driverlib/can.h"

// ######################################################################################################################################################################
// ---------------------------- Module Definitions
// ######################################################################################################################################################################

// The internal bus is within the 360 lighting system (between master and slave nodes)
// The external bus is between the 360 lighting system and the vehicle's ecu
#define CAN_INTERNAL_BUS_BASE		CAN0_BASE
#define CAN_EXTERNAL_BUS_BASE		CAN1_BASE

// Node Types
#define MASTER_NODE				0
#define SLAVE_NODE				1

// Node ID's
//	The lowest binary value ID wins arbitration
#define MASTER_NODE_ID			((uint32_t) 1<<0)
#define SLAVE_NODE_01_ID			((uint32_t) 1<<1)
#define SLAVE_NODE_02_ID			((uint32_t) 1<<2)

// Mask to allow master to receive from all slave nodes
#define ALL_SLAVES_ID_MASK		(0x00000001)			// Allow only the master to pass through
#define ALL_SLAVES_ID			(0x00000000)			// As long as the data isn't from the master, we accept it

// Message Object Numbers
#define MASTER_RX_OBJ_ID			32
#define MASTER_REQUEST_OBJ_ID		31
#define SLAVE_RX_OBJ_ID			32
#define SLAVE_RESPONSE_OBJ_ID		31

// This Node Info
#define THIS_NODE_TYPE			MASTER_NODE

// ######################################################################################################################################################################
// ---------------------------- Module Level Variables
// ######################################################################################################################################################################

// Use pointers so these can only exist in one place
static uint8_t * p_My_Node_ID;		// This node's ID
static uint8_t * p_My_RX_Data;		// This node's data store for incoming data
static uint8_t * p_My_Remote_Data;		// This node's data store for incoming data that was requested (master), or data that we will send on request (slave)

// ######################################################################################################################################################################
// ---------------------------- Private Function Prototypes
// ######################################################################################################################################################################

static void can_master_receive_slave(void);
static void can_slave_respond_master(void);
static void can_slave_receive_master(void);
static uint32_t find_avail_tx_object(uint32_t ui32Base);

// ######################################################################################################################################################################
// ---------------------------- Public Functions
// ######################################################################################################################################################################

/****************************************************************************
	Public Function
     	Initialize_CAN_Internal_Bus

     Parameters
     	uint32_t this_node_id:		the 29-bit id for this node
     	uint8_t * p_rx_data:		a pointer to where this module place new data received
     	uint8_t * p_remote_data:		a pointer to where this module will place data that was requested (if we are the master)
     								or where this module will pull data from (if we are the slave)

     Description
     	Initializes the CAN network's internal bus (required for master and all slaves)

****************************************************************************/
void Initialize_CAN_Internal_Bus(uint8_t * p_this_node_id, uint8_t * p_rx_data, uint8_t * p_remote_data)
{
	// ~~~~~~~~~~~~~~~~~~~~~
	// CAN INTERNAL BUS INIT
	// ~~~~~~~~~~~~~~~~~~~~~
	
	// X. Enable the CAN 0 module peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

	// X. Wait for the CAN0 module to be ready
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0))
	{
	}

	// 1. Initialize the CAN base layer module by resetting the module
	CANInit(CAN_INTERNAL_BUS_BASE);

	// 2. Set the CAN bit timing, there are 4 members (all uint32_t) of this struct type
	// The equation to determine the actual bit rate is as follows:
	// CAN Clock / ((ui32SyncPropPhase1Seg + ui32Phase2Seg + 1) * (ui32QuantumPrescaler))
	// Thus with ui32SyncPropPhase1Seg = 4, ui32Phase2Seg = 1, ui32QuantumPrescaler = 2 and
	// an 8 MHz CAN clock, the bit rate is (8 MHz) / ((5 + 2 + 1) * 2) or 500 Kbit/sec.
	tCANBitClkParms clk_params = {0};
	clk_params.ui32SyncPropPhase1Seg = 4;
	clk_params.ui32Phase2Seg = 1;
	clk_params.ui32SJW = 4;
	clk_params.ui32QuantumPrescaler = 2;
	CANBitTimingSet(CAN_INTERNAL_BUS_BASE, &clk_params);
	// ***Note: Could possibly also use CANBitRateSet (less specified, but simpler)

	// 3. Enable the CAN controller
	CANEnable(CAN_INTERNAL_BUS_BASE);
	
	// 5. Set up the CAN retry behavior to automatic
	CANRetrySet(CAN_INTERNAL_BUS_BASE, true)

	// X. Register the ISR for the CAN bus, we could do this manually BTW, probably the better idea...
	CANIntRegister(CAN_INTERNAL_BUS_BASE, ***NEED FUNCTION POINTER***);

	// 5. Enable CAN Interrupts
	CANIntEnable(CAN_INTERNAL_BUS_BASE, CAN_INT_MASTER);

	// X. Save our node's ID pointer into this module for future use
	p_My_Node_ID = p_this_node_id;

	// X. Save pointers to this node's data stores
	p_My_RX_Data = p_rx_data;
	p_My_Remote_Data = p_remote_data;

	// X. Based on our node type, we set up appropriate message objects here
	if (MASTER_NODE_ID == *p_My_Node_ID)
	{
		can_master_receive_slave();
	}
	else
	{
		can_slave_respond_master();
		can_slave_receive_master();
	}
}

/****************************************************************************
	Public Function
     	CAN_Master_Command_Slave

     Description
     	Sets up a message object to send a command to a specified slave node
	
	Parameters
		ui32 slave_id: id of slave
 		ui8 p_cmd_data: pointer to the command data to be sent

 	Notes
 		We might have to clear this message object after the TX complete interrupt. not sure if this would continue to send data...

****************************************************************************/
void CAN_Master_Command_Slave(uint32_t slave_id, uint8_t * p_cmd_data)
{
	//
	// Definitions
	//
	#define NUM_DATA_BYTES_MASTER_COMMAND_SLAVE 2					// Number of bytes in data we are sending

	//
	// Configure message (follows from page 85 of peripheral manual)
	//
	tCANMsgObject message_object = {0};							// Declare struct
	message_object.ui32MsgID = slave_id;							// Message ID (The 11 or 29 bit identifier)
	message_object.ui32MsgIDMask = 0;								// Used to ensure incoming message is a specific data frame, unused for TX, set to 0
	message_object.ui32Flags = MSG_OBJ_TX_INT_ENABLE; 				// Generate interrupt on TX complete
	message_object.ui32MsgLen = NUMBER_OF_MASTER_COMMAND_DATA_BYTES;		// Number of data bytes to send
	message_object.pui8MsgData = p_cmd_data;						// Pointer to 1st data byte

	//
	// Message Object ID: find lowest object available for transmit, we use the highest 2 registers for the RX data
	//
	uint32_t object_id = find_avail_tx_object(CAN_INTERNAL_BUS_BASE);

	//
	// Set up message object for transmitting commmands to slaves
	//
	CANMessageSet(CAN_INTERNAL_BUS_BASE, object_id, &message_object, (tMsgObjType) MSG_OBJ_TYPE_TX);
}

/****************************************************************************
	Public Function
     	CAN_Master_Request_Slave

     Description
     	Sets up a message object to request data from a slave. This object will automatically convert to a receive node
     	when the request is successfully sent.
	
	Parameters
		ui32 slave_id: id of slave

	Notes
		We might want to set up a function that would allow requesting from all n-slaves. To do this, we would set up n objects for TX_REMOTE.
		This way we wouldn't have to individually request then wait for response, then repeat n times.

****************************************************************************/
void CAN_Master_Request_Slave(uint32_t slave_id)
{
	//
	// Definitions
	//
	#define NUM_DATA_BYTES_MASTER_REQUEST_SLAVE 2					// Number of bytes in data we are expecting from request

	//
	// Configure message (follows from page 85 of peripheral manual)
	//
	tCANMsgObject message_object = {0};							// Declare struct
	message_object.ui32MsgID = slave_id;							// Message ID (The 11 or 29 bit identifier)
	message_object.ui32MsgIDMask = 0;								// Used to ensure incoming message is a specific data frame, unused, set to 0
	message_object.ui32Flags = MSG_OBJ_TX_INT_ENABLE; 				// Which interrupt flag do we use for this?? !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	message_object.ui32MsgLen = NUM_DATA_BYTES_MASTER_REQUEST_SLAVE;		// Number of data bytes we are expecting to receive
	message_object.pui8MsgData = 0;								// Unused pointer value since requests use no data

	//
	// Message Object ID: use second highest ID for incoming requested data, if we want to request from all slaves, we might need to dynamically change this
	//
	uint32_t object_id = MASTER_REQUEST_OBJ_ID;						// 1 of 32 object buffers

	//
	// Set up message object for requesting data from slaves
	//
	CANMessageSet(CAN_INTERNAL_BUS_BASE, object_id, &message_object, (tMsgObjType) MSG_OBJ_TYPE_TX_REMOTE);
}

/****************************************************************************
	Public Function
     	CAN_Slave_Send_Master

     Description
     	Sets up a message object to send data to the master
	
	Parameters
 		ui8 p_cmd_data: pointer to the data to be sent

****************************************************************************/
void CAN_Slave_Send_Master(uint8_t * p_slave_data)
{
	//
	// Definitions
	//
	#define NUM_DATA_BYTES_SLAVE_SEND_MASTER 2						// Number of bytes in data we are sending

	//
	// Configure message (follows from page 85 of peripheral manual)
	//
	tCANMsgObject message_object = {0};							// Declare struct
	message_object.ui32MsgID = *p_My_Node_ID;						// Message ID (The 11 or 29 bit identifier)
	message_object.ui32MsgIDMask = 0;								// Used to ensure incoming message is a specific data frame, unused for TX, set to 0
	message_object.ui32Flags = MSG_OBJ_TX_INT_ENABLE; 				// Generate interrupt on TX complete
	message_object.ui32MsgLen = NUM_DATA_BYTES_SLAVE_SEND_MASTER;		// Number of data bytes to send
	message_object.pui8MsgData = p_slave_data;						// Pointer to 1st data byte

	//
	// Message Object ID: find lowest object available for transmit, we use the highest registers for the RX data
	//
	uint32_t object_id = find_avail_tx_object(CAN_INTERNAL_BUS_BASE);

	//
	// Set up message object for transmitting data to master
	//
	CANMessageSet(CAN_INTERNAL_BUS_BASE, object_id, &message_object, (tMsgObjType) MSG_OBJ_TYPE_TX);
}

/****************************************************************************
	Public Function
     	CAN_Internal_Bus_ISR

     Description
     	This function handles CAN interrupts on the internal bus
	
	Parameters
		None

	Notes
		This method of servicing interrupts requires that we don't get multiple interrupts at a time.

****************************************************************************/
void CAN_Internal_Bus_ISR(void)
{
	//
	// See the cause of the pending interrupts
	//
	uint32_t int_source = CANIntStatus(CAN_INTERNAL_BUS_BASE, (tCANIntStsReg) CAN_INT_STS_CAUSE);
	// If the interrupt is a contoller status interrupt
	if (CAN_INT_INTID_STATUS == int_source)
	{
		//
		// Handle the status accordingly. For now get the status interrupt, in order to clear the int, save it, then do nothing.
		//
		uint32_t controller_status = CANStatusGet(CAN_INTERNAL_BUS_BASE, (tCANStsReg) CAN_STS_CONTROL);
	}
	// Else if the interrupt is from a specific message object
	else if ((0 < int_source) && (32 >= int_source))
	{
		//
		// Only process interrupts from RX's (the highest two objects 31-32)
		//
		if ((32 == int_source) || (31 == int_source))
		{	
			CANMessageGet(CAN_INTERNAL_BUS_BASE, int_source, 0, true)
		}
		else
		{
			CANIntClear(CAN_INTERNAL_BUS_BASE, int_source);
		}

		//
		// Otherwise, the interrupt is from a transmission complete from another message object.
		//
		// Do nothing.

		//
		// Note: could use CANIntStatus(base,CAN_INT_STS_OBJECT) to see all objects that have pending interrupts
		//
	}
	// Else, the interrupt is unspecified in the documentation.
	else
	{
		//
		// Put break point here, but this should never happen.
		//
	}

	//
	// Check interrupt status again to make sure everything is cleared (Page 82)?
	//
}

// ######################################################################################################################################################################
// ---------------------------- Private Functions
// ######################################################################################################################################################################

/****************************************************************************
	Private Function
     	can_master_receive_slave

     Description
     	* This function should only be called once as part of the initialization for the internal CAN bus. *
     	Sets up a message object to receive data from any slave node.
     	This will only work IF the object is not cleared when we read the data;
     	IF the object is cleared then we will need to call this as part of every interrupt response.
     	Update: It looks like only the interrupt is cleared. (page 82 of API doc)
	
	Parameters
		None

	Notes
		We will most likely want to use a mask so that we can use a single object to receive data from multiple message id's

****************************************************************************/
static void can_master_receive_slave(void)
{
	//
	// Definitions
	//
	#define NUM_DATA_BYTES_MASTER_RECEIVE_SLAVE 2					// Number of bytes in data we are expecting to receive

	//
	// Configure message (follows from page 85 of peripheral manual)
	//
	tCANMsgObject message_object = {0};							// Declare struct
	message_object.ui32MsgID = ALL_SLAVES_ID;						// The message id for all slaves after the incoming ID is AND'ed with the mask below
	message_object.ui32MsgIDMask = ALL_SLAVES_ID_MASK;				// This mask is AND'ed with the incoming ID, if matches ID above, the message is received
	message_object.ui32Flags = MSG_OBJ_RX_INT_ENABLE \
		| MSG_OBJ_USE_ID_FILTER; 								// Enable RX interrupts and masking of incoming IDs
	message_object.ui32MsgLen = NUM_DATA_BYTES_MASTER_RECEIVE_SLAVE;		// Number of data bytes we are expecting to receive
	message_object.pui8MsgData = 0;								// Unused pointer value since receives send no data

	//
	// Message Object ID: use highest ID for incoming data from slaves
	//
	uint32_t object_id = MASTER_RX_OBJ_ID;							// Use highest ID'ed object for incoming data from slaves

	//
	// Set up message object for receiving data from slaves
	//
	CANMessageSet(CAN_INTERNAL_BUS_BASE, object_id, &message_object, (tMsgObjType) MSG_OBJ_TYPE_RX);
}

/****************************************************************************
	Private Function
     	can_slave_respond_master

     Description
     	* This function should only be called once in the the initialization of a slave *
     	Sets up a message object to automatically respond to data requests from master
	
	Parameters
 		ui8 p_cmd_data: pointer to the requested data to be sent

****************************************************************************/
static void can_slave_respond_master(void)
{
	//
	// Definitions
	//
	#define NUM_DATA_BYTES_SLAVE_RESPOND_MASTER 2					// Number of bytes in data we are sending

	//
	// Configure message (follows from page 85 of peripheral manual)
	//
	tCANMsgObject message_object = {0};							// Declare struct
	message_object.ui32MsgID = *p_My_Node_ID;						// Message ID (The 11 or 29 bit identifier)
	message_object.ui32MsgIDMask = 0;								// Used to ensure incoming message is a specific data frame, unused for TX, set to 0
	message_object.ui32Flags = MSG_OBJ_TX_INT_ENABLE; 				// Which interrupt flag do we use for this?? !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	message_object.ui32MsgLen = NUM_DATA_BYTES_SLAVE_RESPOND_MASTER;		// Number of data bytes to send
	message_object.pui8MsgData = p_My_Remote_Data;					// Pointer to 1st data byte

	//
	// Message Object ID: use second highest ID for fulfilling data requests
	//
	uint32_t object_id = SLAVE_RESPONSE_OBJ_ID;						// 1 of 32 object buffers

	//
	// Set up message object for responding to requests from master
	//
	CANMessageSet(CAN_INTERNAL_BUS_BASE, object_id, &message_object, (tMsgObjType) MSG_OBJ_TYPE_RXTX_REMOTE);
}

/****************************************************************************
	Private Function
     	can_slave_receive_master

     Description
     	* This function should only be called once as part of the initialization for the internal CAN bus. *
     	Sets up a message object on a slave to receive data from the master that corresponds to a particular slave.
     	This will only work IF the object is not cleared when we read the data;
     	IF the object is cleared then we will need to call this as part of every interrupt response.
     	Update: It looks like only the interrupt is cleared. (page 82 of API doc)
	
	Parameters
		None

	Notes
		None

****************************************************************************/
static void can_slave_receive_master(void)
{
	//
	// Definitions
	//
	#define NUM_DATA_BYTES_SLAVE_RECEIVE_MASTER 2					// Number of bytes in data we are expecting to receive

	//
	// Configure message (follows from page 85 of peripheral manual)
	//
	tCANMsgObject message_object = {0};							// Declare struct
	message_object.ui32MsgID = *p_My_Node_ID;						// The message id for this particular slave
	message_object.ui32MsgIDMask = 0;								// This mask is AND'ed with the incoming ID, if matches ID above, the message is received
	message_object.ui32Flags = MSG_OBJ_RX_INT_ENABLE;					// Enable RX interrupts and masking of incoming IDs
	message_object.ui32MsgLen = NUM_DATA_BYTES_SLAVE_RECEIVE_MASTER;		// Number of data bytes we are expecting to receive
	message_object.pui8MsgData = 0;								// Unused pointer value since receives send no data

	//
	// Message Object ID: use highest ID for incoming data from master
	//
	uint32_t object_id = SLAVE_RX_OBJ_ID;							// Use highest ID'ed object for incoming data from master

	//
	// Set up message object for receiving commands from master
	//
	CANMessageSet(CAN_INTERNAL_BUS_BASE, object_id, &message_object, (tMsgObjType) MSG_OBJ_TYPE_RX);
}

/****************************************************************************
	Private Function
     	find_avail_tx_object

     Description
     	Finds highest priority available message object for transmission
	
	Parameters
		CAN bus base

	Returns
		uint32_t: integer object (1-32)

	Notes
		None

****************************************************************************/
static uint32_t find_avail_tx_object(uint32_t ui32Base)
{
	//
	// Definitions
	//
	#define LSB_SET 		0x01
	#define MAX_TX_OBJECTS	30

	//
	// Get bit map of object pending transmission
	//
	uint32_t tx_bit_map = CANStatusGet(ui32Base, (tCANStsReg) CAN_STS_TXREQUEST);	// Returns bit map of objects pending transmission

	//
	// Find lowest object available for transmission
	//
	uint32_t object_id = 0; 												// Return invalid object id if none are available
	for (int i = 0; i < MAX_TX_OBJECTS; i++)
	{
		if ( ((tx_bit_map >> i) & LSB_SET) == LSB_SET )
		{
			object_id = i+1;
			break
		}
	}

	//
	// Return object id
	//
	return object_id;
}

