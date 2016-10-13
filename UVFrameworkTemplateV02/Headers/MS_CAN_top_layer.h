#ifndef MS_CAN_top_layer_H
#define MS_CAN_top_layer_H

// typedefs for the states in the state machine
// State definitions for use with the query function

//Public function prototypes

void Initialize_CAN_Internal_Bus(uint32_t * p_this_node_id, uint8_t * p_rx_data, uint8_t * p_remote_data);
void CAN_Master_Command_Slave(uint32_t slave_id, uint8_t * p_cmd_data);
void CAN_Master_Request_Slave(uint32_t slave_id);
void CAN_Slave_Send_Master(uint8_t * p_slave_data);
void CAN_Internal_Bus_ISR(void);

#endif // MS_CAN_top_layer_H
