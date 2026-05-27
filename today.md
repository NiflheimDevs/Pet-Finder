## 2/27/2026

Program Framework.   
The proc_main_task function is the entrance of Embedded Application, just like the main() in C 
application. 

Ql_OS_GetMessage() blocks until a message arrives in this task's queue.   
The message may come from:
- another task
- the operating system
- a timer callback
- UART/event handlers
- network events
- SIM events
- etc.

So the queue belongs to this task, but the messages are usually produced elsewhere.

```c
//the entrance of this application
void proc_main_task(s32 taskId){
    ST_MSG msg;
    //start message loop of this task
    while(1)
    {
        Ql-OS_GetMessag(&msg);
        switch(msg.message)
        {
            case MSG_ID_RIL_READY: 
            { 
                Ql_Debug_Trace("<-- RIL is ready -->\r\n"); 

                //Before using the RIL feature, developers must initialize it by calling the following APIs.  
                //After receiving the MSG_ID_RIL_READY message. 
                Ql_RIL_Initialize(); 

                //Now developers can start to send AT commands. 
                Demo_SendATCmd(); 
                break; 
            } 
            case MSG_ID_URC_INDICATION: 
            { 
                //Ql_Debug_Trace("<-- Received URC: type: %d, -->\r\n", msg.param1); 
                switch (msg.param1) 
                { 
                case URC_SYS_INIT_STATE_IND:
                    Ql_Debug_Trace("<-- Sys Init Status %d -->\r\n", msg.param2); 
                    break; 
                case URC_SIM_CARD_STATE_IND: 
                    Ql_Debug_Trace("<-- SIM Card Status:%d -->\r\n", msg.param2); 
                    break; 
                case URC_GSM_NW_STATE_IND: 
                    Ql_Debug_Trace("<-- GSM Network Status:%d -->\r\n", msg.param2); 
                    break; 
                case URC_GPRS_NW_STATE_IND: 
                    Ql_Debug_Trace("<-- GPRS Network Status:%d -->\r\n", msg.param2); 
                    break; 
                case URC_CFUN_STATE_IND: 
                    Ql_Debug_Trace("<-- CFUN Status:%d -->\r\n", msg.param2); 
                    break; 
                }
                default: 
                    Ql_Debug_Trace("<-- Other URC: type=%d\r\n", msg.param1); 
                    break;  
            }
            // 
            //Other Message ID of users… 
            // 
        default: 
            break;
        }
    }
}
```
\
Data types :
>
```c
bool = typedef unsigned char bool; 
s8 = typedef signed char s8; 
u8 = typedef unsigned char u8; 
s16 = typedef signed short s16; 
u16 = typedef unsigned short u16; 
s32 = typedef int s32; 
u32 = typedef unsigned int u32; 
u64 = typedef unsigned long lone u64; 
float = Floating-point variable. This variable is declared in math.h. 
```
>



Developers should avoid calling these functions: ***Ql_Sleep()***, ***Ql_OS_TakeSemaphore()*** and 
***Ql_OS_TakeMutex()***. These functions will block the task, thus will make the task cannot fetch message 
from the message queue. If the message queue is filled up, the system will automatically reboot 
unexpectedly.