#include "app_cfg.h"
#include "os.h"
#include "stm32f10x_rcc.h"

static OS_TCB       AppTaskStartTCB;
static OS_TCB       AppTask1_TCB;
static OS_TCB		AppTask2_TCB;

static OS_MUTEX     AppMutex;
static OS_Q         AppQ;

static CPU_STK      AppTaskStartStk[128]; // 32-bit per unit
static CPU_STK      AppTask1_Stk[128];
static CPU_STK 		AppTask2_Stk[128];

static void AppTaskStart(void *p_arg);
static void AppTask1    (void *p_arg);
static void AppTask2 	(void *p_arg);

void main(void)
{
    OS_ERR err;

    // Stm_IntDisAll(); // custom
    OSInit(&err);
    /**
    * Check for 'err'
    */

    OSMutexCreate((OS_MUTEX *)&AppMutex,
    			  (CPU_CHAR *)"My App Mutex",
    			  (OS_ERR	*)&err);
    /**
    * Check for 'err'
    */

    OSQCreate((OS_Q  	*)&AppQ,
    		  (CPU_CHAR *)"My App Queue",
    		  (OS_MSG_QTY)10,
    		  (OS_ERR	*)&err);
    /**
    * Check for 'err'
    */

    OSTaskCreate((OS_TCB 	 *)&AppTaskStartTCB, 
				 (CPU_CHAR 	 *)"App Task Start",
				 (OS_TASK_PTR )AppTaskStart,
				 (void 		 *)0,
				 (OS_PRIO 	  )APP_TASK_START_PRIO,
				 (CPU_STK 	 *)&AppTaskStartStk[0],
				 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
				 (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
				 (OS_MSG_QTY  )0,
				 (OS_TICK 	  )0,
				 (void 		 *)0,
				 (OS_OPT 	  )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR 	 *)&err);    
    /**
    * Check for 'err'
    */			  

    OSStart(&err);
    /**
    * Check for 'err'
    */	   
}


static void AppTaskStart(void *p_arg)
{
	OS_ERR err;

	// Stm_Init(); // custom
	CPU_Init();

	// Stm_Cfg_Tick(); // ?? custom

	OSTaskCreate((OS_TCB 	 *)&AppTask1_TCB, 
				 (CPU_CHAR 	 *)"App Task 1",
				 (OS_TASK_PTR )AppTask1,
				 (void 		 *)0,
				 (OS_PRIO 	  )5,
				 (CPU_STK 	 *)&AppTask1_Stk[0],
				 (CPU_STK_SIZE)0,
				 (CPU_STK_SIZE)128,
				 (OS_MSG_QTY  )0,
				 (OS_TICK 	  )0,
				 (void 		 *)0,
				 (OS_OPT 	  )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR 	 *)&err);

	OSTaskCreate((OS_TCB 	 *)&AppTask2_TCB, 
				 (CPU_CHAR 	 *)"App Task 2",
				 (OS_TASK_PTR )AppTask2,
				 (void 		 *)0,
				 (OS_PRIO 	  )6,
				 (CPU_STK 	 *)&AppTask2_Stk[0],
				 (CPU_STK_SIZE)0,
				 (CPU_STK_SIZE)128,
				 (OS_MSG_QTY  )0,
				 (OS_TICK 	  )0,
				 (void 		 *)0,
				 (OS_OPT 	  )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR 	 *)&err);	

	// BSP_LED_Off(0);
	while(1)
	{
		// BSP_LED_Toggle(0);
		OSTimeDlyHMSM((CPU_INT16U) 0,
					  (CPU_INT16U) 0,
					  (CPU_INT16U) 0,
					  (CPU_INT32U)100,
					  (OS_OPT 	 )OS_OPT_TIME_HMSM_STRICT,
					  (OS_ERR	*)&err);
	}
	/**
	* Error Handle
	*/
}


static void AppTask1(void *p_arg)
{
	OS_ERR err;
	CPU_TS ts;

	p_arg = p_arg;
	while(1)
	{
		OSTimeDly((OS_TICK)1, // delay 1 tick, the actual time is depend on the ucos tick rate
				  (OS_OPT )OS_OPT_TIME_DLY,
				  (OS_ERR*)&err);

		OSQPost((OS_Q 		*)&AppQ,
				(void  		*)1, // also can send the address of a buffer, address of function, or more.
				(OS_MSG_SIZE )sizeof(void *),
				(OS_OPT 	 )OS_OPT_POST_FIFO,
				(OS_ERR	    *)&err);

		// wait mutex release and require a mutex
		OSMutexPend((OS_MUTEX *)&AppMutex, 
					(OS_TICK   )0, // time out set, 0 means forever.
					(OS_OPT    )OS_OPT_PEND_BLOCKING,
					(CPU_TS   *)&ts,
					(OS_ERR   *)&err);

		/* Access shared resource */ 

		// release the mutex
		OSMutexPost((OS_MUTEX *)&AppMutex, 
					(OS_OPT    )OS_OPT_POST_NONE,
					(OS_ERR   *)&err);
	}
}


static void AppTask2 (void *p_arg)
{
	OS_ERR err;
	void *p_msg;
	OS_MSG_SIZE msg_size;
	CPU_TS ts;
	CPU_TS ts_delta;

	p_arg = p_arg;
	while (1) 
	{
		p_msg = OSQPend((OS_Q 		 *)&AppQ, 
						(OS_TICK 	  )0, // time out set, 0 means forever.
						(OS_OPT       )OS_OPT_PEND_BLOCKING,
                        (OS_MSG_SIZE *)&msg_size,
						(CPU_TS 	 *)&ts,
						(OS_ERR 	 *)&err); 
		/**
		* OSQPend() return a pointer to "something" which is the message.
		* "p_msg" point to a buffer and "msg_size" indicate the size of this buffer.
		* "ts" contains the timestamp of when the message was sent.
		*/

		ts_delta = OS_TS_GET() - ts; 
		/* Process message received */ 
	}
}


#define  DWT_CR      *(CPU_REG32 *)0xE0001000
#define  DWT_CYCCNT  *(CPU_REG32 *)0xE0001004
#define  DEM_CR      *(CPU_REG32 *)0xE000EDFC
#define  DBGMCU_CR   *(CPU_REG32 *)0xE0042004

#define  DEM_CR_TRCENA                   (1 << 24)

#define  DWT_CR_CYCCNTENA                (1 <<  0)


/*
*********************************************************************************************************
*                                            BSP_CPU_ClkFreq()
*
* Description : Read CPU registers to determine the CPU clock frequency of the chip.
*
* Argument(s) : none.
*
* Return(s)   : The CPU clock frequency, in Hz.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT32U  BSP_CPU_ClkFreq (void)
{
    RCC_ClocksTypeDef  rcc_clocks;


    RCC_GetClocksFreq(&rcc_clocks);

    return ((CPU_INT32U)rcc_clocks.HCLK_Frequency);
}


/*$PAGE*/
/*
*********************************************************************************************************
*                                          CPU_TS_TmrInit()
*
* Description : Initialize & start CPU timestamp timer.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : CPU_TS_Init().
*
*               This function is an INTERNAL CPU module function & MUST be implemented by application/
*               BSP function(s) [see Note #1] but MUST NOT be called by application function(s).
*
* Note(s)     : (1) CPU_TS_TmrInit() is an application/BSP function that MUST be defined by the developer
*                   if either of the following CPU features is enabled :
*
*                   (a) CPU timestamps
*                   (b) CPU interrupts disabled time measurements
*
*                   See 'cpu_cfg.h  CPU TIMESTAMP CONFIGURATION  Note #1'
*                     & 'cpu_cfg.h  CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION  Note #1a'.
*
*               (2) (a) Timer count values MUST be returned via word-size-configurable 'CPU_TS_TMR'
*                       data type.
*
*                       (1) If timer has more bits, truncate timer values' higher-order bits greater
*                           than the configured 'CPU_TS_TMR' timestamp timer data type word size.
*
*                       (2) Since the timer MUST NOT have less bits than the configured 'CPU_TS_TMR'
*                           timestamp timer data type word size; 'CPU_CFG_TS_TMR_SIZE' MUST be
*                           configured so that ALL bits in 'CPU_TS_TMR' data type are significant.
*
*                           In other words, if timer size is not a binary-multiple of 8-bit octets
*                           (e.g. 20-bits or even 24-bits), then the next lower, binary-multiple
*                           octet word size SHOULD be configured (e.g. to 16-bits).  However, the
*                           minimum supported word size for CPU timestamp timers is 8-bits.
*
*                       See also 'cpu_cfg.h   CPU TIMESTAMP CONFIGURATION  Note #2'
*                              & 'cpu_core.h  CPU TIMESTAMP DATA TYPES     Note #1'.
*
*                   (b) Timer SHOULD be an 'up'  counter whose values increase with each time count.
*
*                   (c) When applicable, timer period SHOULD be less than the typical measured time
*                       but MUST be less than the maximum measured time; otherwise, timer resolution
*                       inadequate to measure desired times.
*
*                   See also 'CPU_TS_TmrRd()  Note #2'.
*********************************************************************************************************
*/

#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED)
void  CPU_TS_TmrInit (void)
{
    CPU_INT32U  cpu_clk_freq_hz;
      
      
    DEM_CR         |= (CPU_INT32U)DEM_CR_TRCENA;                /* Enable Cortex-M3's DWT CYCCNT reg.                   */
    DWT_CYCCNT      = (CPU_INT32U)0u;
    DWT_CR         |= (CPU_INT32U)DWT_CR_CYCCNTENA;

    cpu_clk_freq_hz = BSP_CPU_ClkFreq();    
    CPU_TS_TmrFreqSet(cpu_clk_freq_hz);
}
#endif


/*$PAGE*/
/*
*********************************************************************************************************
*                                           CPU_TS_TmrRd()
*
* Description : Get current CPU timestamp timer count value.
*
* Argument(s) : none.
*
* Return(s)   : Timestamp timer count (see Notes #2a & #2b).
*
* Caller(s)   : CPU_TS_Init(),
*               CPU_TS_Get32(),
*               CPU_TS_Get64(),
*               CPU_IntDisMeasStart(),
*               CPU_IntDisMeasStop().
*
*               This function is an INTERNAL CPU module function & MUST be implemented by application/
*               BSP function(s) [see Note #1] but SHOULD NOT be called by application function(s).
*
* Note(s)     : (1) CPU_TS_TmrRd() is an application/BSP function that MUST be defined by the developer
*                   if either of the following CPU features is enabled :
*
*                   (a) CPU timestamps
*                   (b) CPU interrupts disabled time measurements
*
*                   See 'cpu_cfg.h  CPU TIMESTAMP CONFIGURATION  Note #1'
*                     & 'cpu_cfg.h  CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION  Note #1a'.
*
*               (2) (a) Timer count values MUST be returned via word-size-configurable 'CPU_TS_TMR'
*                       data type.
*
*                       (1) If timer has more bits, truncate timer values' higher-order bits greater
*                           than the configured 'CPU_TS_TMR' timestamp timer data type word size.
*
*                       (2) Since the timer MUST NOT have less bits than the configured 'CPU_TS_TMR'
*                           timestamp timer data type word size; 'CPU_CFG_TS_TMR_SIZE' MUST be
*                           configured so that ALL bits in 'CPU_TS_TMR' data type are significant.
*
*                           In other words, if timer size is not a binary-multiple of 8-bit octets
*                           (e.g. 20-bits or even 24-bits), then the next lower, binary-multiple
*                           octet word size SHOULD be configured (e.g. to 16-bits).  However, the
*                           minimum supported word size for CPU timestamp timers is 8-bits.
*
*                       See also 'cpu_cfg.h   CPU TIMESTAMP CONFIGURATION  Note #2'
*                              & 'cpu_core.h  CPU TIMESTAMP DATA TYPES     Note #1'.
*
*                   (b) Timer SHOULD be an 'up'  counter whose values increase with each time count.
*
*                       (1) If timer is a 'down' counter whose values decrease with each time count,
*                           then the returned timer value MUST be ones-complemented.
*
*                   (c) (1) When applicable, the amount of time measured by CPU timestamps is
*                           calculated by either of the following equations :
*
*                           (A) Time measured  =  Number timer counts  *  Timer period
*
*                                   where
*
*                                       Number timer counts     Number of timer counts measured
*                                       Timer period            Timer's period in some units of
*                                                                   (fractional) seconds
*                                       Time measured           Amount of time measured, in same
*                                                                   units of (fractional) seconds
*                                                                   as the Timer period
*
*                                                  Number timer counts
*                           (B) Time measured  =  ---------------------
*                                                    Timer frequency
*
*                                   where
*
*                                       Number timer counts     Number of timer counts measured
*                                       Timer frequency         Timer's frequency in some units
*                                                                   of counts per second
*                                       Time measured           Amount of time measured, in seconds
*
*                       (2) Timer period SHOULD be less than the typical measured time but MUST be less
*                           than the maximum measured time; otherwise, timer resolution inadequate to
*                           measure desired times.
*********************************************************************************************************
*/

#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED)
CPU_TS_TMR  CPU_TS_TmrRd (void)
{
    return ((CPU_TS_TMR)DWT_CYCCNT);
}
#endif