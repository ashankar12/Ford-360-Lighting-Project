/****************************************************************************
 Module
   EventCheckers.c

 Revision
   1.0.1 

 Description
   This is the sample for writing event checkers along with the event 
   checkers used in the basic framework test harness.

 Notes
   Note the use of static variables in sample event checker to detect
   ONLY transitions.
   
 History
 When           Who     What/Why
 -------------- ---     --------
 08/06/13 13:36 jec     initial version
****************************************************************************/

// this will pull in the symbolic definitions for events, which we will want
// to post in response to detecting events
#include "ES_Configure.h"
// this will get us the structure definition for events, which we will need
// in order to post events in response to detecting events
#include "ES_Events.h"
// if you want to use distribution lists then you need those function 
// definitions too.
#include "ES_PostList.h"
// This include will pull in all of the headers from the service modules
// providing the prototypes for all of the post functions
#include "ES_ServiceHeaders.h"
// this test harness for the framework references the serial routines that
// are defined in ES_Port.c
#include "ES_Port.h"
// include our own prototypes to insure consistency between header & 
// actual functionsdefinition
#include "EventCheckers.h"

#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_ShortTimer.h"
#include "ES_Types.h"     /* gets bool type for returns */

#define BRAKE_BIT BIT0HI
#define PAIR_BIT BIT1HI

/****************************************************************************
 Function
   Check4Keystroke
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   checks to see if a new key from the keyboard is detected and, if so, 
   retrieves the key and posts an ES_NewKey event to TestHarnessService0
 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c
   Since we always retrieve the keystroke when we detect it, thus clearing the
   hardware flag that indicates that a new key is ready this event checker 
   will only generate events on the arrival of new characters, even though we
   do not internally keep track of the last keystroke that we retrieved.
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Keystroke(void) {
    
//  if ( IsNewKeyReady() ) // new key waiting?
//  {
//        
//    ES_Event ThisEvent;
//    ThisEvent.EventType = ES_BEGIN_TRANSMIT;
//    
//        switch(GetNewKey()) {
//            
//            case 'y':
//                ThisEvent.EventType = ES_BEGIN_TRANSMIT;
//                ThisEvent.EventParam = 0x00;
//                PostPACTransmit( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_BEGIN_TRANSMIT: REQ_PAIR event.");
//            break;
//            
//            case 'e':
//                ThisEvent.EventType = ES_BEGIN_TRANSMIT;
//                ThisEvent.EventParam = 0x01;
//                PostPACTransmit( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_BEGIN_TRANSMIT: ENCR_KEY event.");
//            break;
//            
//            case 'c':
//                ThisEvent.EventType = ES_BEGIN_TRANSMIT;
//                ThisEvent.EventParam = 0x02;
//                PostPACTransmit( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_BEGIN_TRANSMIT: CTRL event.");
//            break;
//            
//            case 't':
//                ThisEvent.EventType = ES_BEGIN_TRANSMIT;
//                ThisEvent.EventParam = 0x03;
//                PostPACTransmit( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_BEGIN_TRANSMIT: STATUS event.");
//            break;
//            
//            case 'z':
//                ThisEvent.EventType = ES_BEGIN_TRANSMIT;
//                ThisEvent.EventParam = 0x04;
//                PostPACTransmit( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_BEGIN_TRANSMIT: RESEND event.");
//            break;
//            
//            case 'p':
//                ThisEvent.EventType = ES_USER_PAIR_PRESS;
//                ThisEvent.EventParam = 0x00; // Param doesn't matter    
//                PostPACService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_USER_PAIR_REQUEST event.");
//            break;
//            
//            case 'l':
//                ThisEvent.EventType = ES_STATUS_RECEIVED;
//                ThisEvent.EventParam = (BIT0HI); // Paired
//                PostPACService( ThisEvent );
//            printf("\r\n\n\n\n\n\nSent ES_STATUS_RECEIVED: Paired event.");
//            break;
//            
//            case '0':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x00; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x00 event.");
//            break;
//            
//            case '1':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x01; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x01 event.");
//            break;
//            
//            case '2':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x02; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x02 event.");
//            break;
//            
//            case '3':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x03; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x03 event.");
//            break;
//            
//            case '4':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x04; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x04 event.");
//            break;
//            
//            case '5':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x05; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x05 event.");
//            break;
//            
//            case '6':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x06; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x06 event.");
//            break;
//            
//            case 'w':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x01; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x01 event.");
//            break;
//            
//            case 'a':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x03; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x03 event.");
//            break;
//            
//            case 's':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x02; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x02 event.");
//            break;
//            
//            case 'd':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x04; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x04 event.");
//            break;
//            
//            case 'f':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x00; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x00 event.");
//            break;
//            
//            case 'g':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x06; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x06 event.");
//            break;
//            
//            case 'r':
//                ThisEvent.EventType = ES_TEST;
//                ThisEvent.EventParam = 0x05; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_TEST: 0x05 event.");
//            break;
//            
//            case ' ':
//                ThisEvent.EventType = ES_SPACEBAR;
//                ThisEvent.EventParam = 0x00; // Doesn't matter
//                PostUIService( ThisEvent );
//                //printf("\r\n\n\n\n\n\nSent ES_SPACEBAR event.");
//            break;
//            
//            case 'b':
//                ThisEvent.EventType = ES_SPECIAL_ACTION;
//                ThisEvent.EventParam = BRAKE_BIT; // Doesn't matter
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_SPECIAL_ACTION: Brake event.");
//            break;
//            
//            case 'u':
//                ThisEvent.EventType = ES_SPECIAL_ACTION;
//                ThisEvent.EventParam = PAIR_BIT; // Unpaired
//                PostUIService( ThisEvent );
//                printf("\r\n\n\n\n\n\nSent ES_SPECIAL_ACTION: Unpair event.");
//            break;
//            
//        }
//        
//    return true;
//  }
  return false;
}
