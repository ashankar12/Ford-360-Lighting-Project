#ifndef Master_Main_Service_H
#define Master_Main_Service_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Framework.h"

// Public Function Prototypes
bool Init_Master_Main_Service ( uint8_t Priority );
bool Post_Master_Main_Service( ES_Event ThisEvent );
ES_Event Run_Master_Main_Service( ES_Event ThisEvent );

#endif /* Master_Main_Service_H */
