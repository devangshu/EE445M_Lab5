// *************os.c**************
// EE445M/EE380L.6 Labs 1, 2, 3, and 4 
// High-level OS functions
// Students will implement these functions as part of Lab
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 
// Jan 12, 2020, valvano@mail.utexas.edu


#include <stdint.h>
#include <stdio.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/Timer4A.h"
#include "../inc/WTimer0A.h"
#include "../inc/Timer1A.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../inc/ADCT0ATrigger.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../src/globals.h"


// Performance Measurements 
int32_t MaxJitter = 0;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
uint32_t const JitterSize=JITTERSIZE;
uint32_t JitterHistogram[JITTERSIZE]={0,};

// Lab  2 stuff
TCB_t * RunPt;
uint32_t * current_sp;
int32_t * current_block_pt;

#define STACKDEPTH 128
TCB_t TCBs[MAXTHREADS];
uint32_t Stacks[MAXTHREADS][STACKDEPTH];
uint32_t NumThreads_Global = 0;
#define TIME1MS 80000
uint32_t MsTime = 0;
TCB_t *lastTCB;


#define MAX_FIFO 64


void StartOS(void);
void ContextSwitch(void);
//void SysTick_Handler(void);

// make static arrays of tcbs and stack, don't use malloc
// make size 7-10 static array of tcbs
// stack is 2d array, one array for each thread, size 128 bytes works


static void Timer3A_Init(void){
	unsigned long sr = StartCritical();
	SYSCTL_RCGCTIMER_R |= 0x08;   // 0) activate TIMER3
	TIMER3_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
	TIMER3_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
	TIMER3_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
	TIMER3_TAILR_R = TIME1MS-1;    // 4) reload value
	TIMER3_TAPR_R = 0;            // 5) bus clock resolution
	TIMER3_ICR_R = 0x00000001;    // 6) clear TIMER3A timeout flag
	TIMER3_IMR_R = 0x00000001;    // 7) arm timeout interrupt
	NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x20000000; // 8) priority 1
	// vector number 51, interrupt number 35
	NVIC_EN1_R = 1<<(35-32);      // 9) enable IRQ 35 in NVIC
	TIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
  MsTime = 0;
	EndCritical(sr);
}

/*
Timer 3 is used to update all of the sleeping threads
*/
void Timer3A_Handler(void){
	TIMER3_ICR_R = TIMER_ICR_TATOCINT;
	MsTime++;
	for(int i = 0; i < NumThreads_Global; i++){
		if(TCBs[i].current_state == SLEEP){
			TCBs[i].sleep_ms--;
			if(TCBs[i].sleep_ms == 0){
				TCBs[i].current_state = ACTIVE;
			}
		}
	}
}

void OS_GetNextThread(void){
	
	TCB_t *tempPt = RunPt; // temporary copy so we can iterate through TCBs
	TCB_t *last = RunPt; 
	TCB_t *highestPriority;
	int maxPriority = 255; // this value will change based on the number of priority levels
  
	if (last->current_state != ACTIVE){
		last = last->prev;
	}
	
  do
  {
    tempPt = tempPt->next;
    if(((tempPt->priority) < maxPriority) && ((tempPt->current_state) == ACTIVE)){
      maxPriority = tempPt->priority;
      highestPriority = tempPt;
    }
  } while (last != tempPt);
  RunPt = highestPriority;  
	
	/*
	RunPt = RunPt->next;
  while(RunPt->current_state != ACTIVE){
    RunPt = RunPt->next;
  }
	*/
}


/*------------------------------------------------------------------------------
  Systick Interrupt Handler
  SysTick interrupt happens every 10 ms
  used for preemptive thread switch
 *------------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  
  //PD3^=0x08;
	OS_Suspend();
	//PD3^=0x08;

} // end SysTick_Handler

unsigned long OS_LockScheduler(void){
  // lab 4 might need this for disk formating
  return 0;// replace with solution
}
void OS_UnLockScheduler(unsigned long previous){
  // lab 4 might need this for disk formating
}


void SysTick_Init(unsigned long period){
  NVIC_ST_CTRL_R = 0;
  NVIC_ST_CURRENT_R = 0;
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000;
}

/**
 * @details  Initialize operating system, disable interrupts until OS_Launch.
 * Initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers.
 * Interrupts not yet enabled.
 * @param  none
 * @return none
 * @brief  Initialize OS
 */
void OS_Init(void){
  // put Lab 2 (and beyond) solution here
  DisableInterrupts();
  PLL_Init(Bus80MHz);
	Timer3A_Init();
  UART_Init();
  ST7735_InitR(INITR_REDTAB);

  // initialize all TCBs
  uint32_t i;
  for(i = 0; i < MAXTHREADS; i++){
      TCBs[i].current_state = DEAD; // initialize all TCBs as free
  }

  SysTick_Init(10000); //not using period, gets reset later anyway
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00E00000; // initialize PendSV
	NumThreads_Global = 0;

}; 


// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, int32_t value){
  semaPt-> Value = value;
  semaPt->head = 0;
  semaPt->tail = 0;
}; 

static void Mutex_Block(Sema4Type *semaPt){
  RunPt->block_pt = semaPt;
  RunPt->current_state = BLOCKED;
  
  uint32_t tail = semaPt->tail;
  semaPt->blocked_threads[tail] = RunPt;
  semaPt->tail = (tail+1) % MAXTHREADS;
}

static void Mutex_Release(Sema4Type *semaPt){
  uint32_t head = semaPt->head;
  semaPt->blocked_threads[head]->current_state = ACTIVE;
  semaPt->head = (head + 1) % MAXTHREADS;
}

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
  DisableInterrupts(); // disable interrupts to make sure current thread is the only thread trying to access the semaphore at any given time 
  semaPt->Value -= 1;
  if(semaPt->Value < 0){
    RunPt->block_pt = semaPt;
		RunPt->current_state = BLOCKED;
  
		uint32_t tail = semaPt->tail;
		semaPt->blocked_threads[tail] = RunPt;
		semaPt->tail = (tail+1) % MAXTHREADS;
    EnableInterrupts();
    OS_Suspend();
  }
  EnableInterrupts(); // allow other threads to request semaphore after we have acquired it
}; 

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
  long sr = StartCritical();
  semaPt->Value += 1; // increment semaphore value atomically. If value was at 0, this allows a waiting thread to acquire the semaphore.
  if(semaPt->Value <= 0){
		uint32_t head = semaPt->head;
		semaPt->blocked_threads[head]->current_state = ACTIVE;
		semaPt->head = (head + 1) % MAXTHREADS;  
  }
  EndCritical(sr);
}; 

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
  DisableInterrupts();
  while(semaPt->Value == 0){ // same as OS_Wait, allow other threads to access cpu time
    RunPt->block_pt = semaPt;
		RunPt->current_state = BLOCKED;
  
		uint32_t tail = semaPt->tail;
		semaPt->blocked_threads[tail] = RunPt;
		semaPt->tail = (tail+1) % MAXTHREADS;
    EnableInterrupts();
    OS_Suspend();
  }
  semaPt->Value = 0;
  EnableInterrupts();
}; 

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
  long sr;
  sr = StartCritical();
  if(semaPt->Value == 0){
		uint32_t head = semaPt->head;
		semaPt->blocked_threads[head]->current_state = ACTIVE;
		semaPt->head = (head + 1) % MAXTHREADS;
  }
  semaPt->Value = 1;
  EndCritical(sr);
}; 

void SetInitialStack(int i){
  TCBs[i].sp = &Stacks[i][STACKDEPTH-16]; // thread stack pointer
  Stacks[i][STACKDEPTH-1] = 0x01000000;   // thumb bit
  // Stacks[i][STACKDEPTH-2] is PC
  Stacks[i][STACKDEPTH-3] = 0x14141414;   // R14
  Stacks[i][STACKDEPTH-4] = 0x12121212;   // R12
  Stacks[i][STACKDEPTH-5] = 0x03030303;   // R3
  Stacks[i][STACKDEPTH-6] = 0x02020202;   // R2
  Stacks[i][STACKDEPTH-7] = 0x01010101;   // R1
  Stacks[i][STACKDEPTH-8] = 0x00000000;   // R0
  Stacks[i][STACKDEPTH-9] = 0x11111111;   // R11
  Stacks[i][STACKDEPTH-10] = 0x10101010;  // R10
  Stacks[i][STACKDEPTH-11] = 0x09090909;  // R9
  Stacks[i][STACKDEPTH-12] = 0x08080808;  // R8
  Stacks[i][STACKDEPTH-13] = 0x07070707;  // R7
  Stacks[i][STACKDEPTH-14] = 0x06060606;  // R6
  Stacks[i][STACKDEPTH-15] = 0x05050505;  // R5
  Stacks[i][STACKDEPTH-16] = 0x04040404;  // R4
}

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), 
  uint32_t stackSize, uint32_t priority){

  long sr = StartCritical(); // ensure this is atomic

  if(NumThreads_Global == 0){
    TCBs[0].current_state = ACTIVE;
    TCBs[0].id = NumThreads_Global;
    NumThreads_Global++;
    TCBs[0].priority = priority;
    TCBs[0].next = &TCBs[0];
    TCBs[0].prev = &TCBs[0];
    lastTCB = &TCBs[0];
    RunPt = &TCBs[0];
    SetInitialStack(0);
    Stacks[0][STACKDEPTH - 2] = (int32_t)task;
  } else {
    uint32_t numTCBs;
    // find first free TCB
    for(numTCBs = 0; numTCBs < MAXTHREADS; numTCBs++){
      if(TCBs[numTCBs].current_state == DEAD){
        break;
      }
    }

    // check to make sure we have room to add another thread
    if(numTCBs == MAXTHREADS){
      EndCritical(sr);
      return 0;
    }

    TCBs[numTCBs].next = lastTCB->next;
    TCBs[numTCBs].prev = lastTCB;

    (lastTCB->next)->prev = &TCBs[numTCBs];
    lastTCB->next = &TCBs[numTCBs];
    lastTCB = &TCBs[numTCBs];

    TCBs[numTCBs].current_state = ACTIVE;
    TCBs[numTCBs].id = NumThreads_Global;
    NumThreads_Global++;
    TCBs[numTCBs].priority = priority;
    SetInitialStack(numTCBs);
    Stacks[numTCBs][STACKDEPTH - 2] = (int32_t)task;
  }

  EndCritical(sr); 
  return 1;
};

//******** OS_AddProcess *************** 
// add a process with foregound thread to the scheduler
// Inputs: pointer to a void/void entry point
//         pointer to process text (code) segment
//         pointer to process data segment
//         number of bytes allocated for its stack
//         priority (0 is highest)
// Outputs: 1 if successful, 0 if this process can not be added
// This function will be needed for Lab 5
// In Labs 2-4, this function can be ignored
int OS_AddProcess(void(*entry)(void), void *text, void *data, 
  unsigned long stackSize, unsigned long priority){
  // put Lab 5 solution here

     
  return 0; // replace this line with Lab 5 solution
}


//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
uint32_t OS_Id(void){
  // put Lab 2 (and beyond) solution here
  
  return 0; // replace this line with solution
};

#define MAXPERIODIC 2
uint32_t periodic_thread_cnt = 0;
//void(*PeriodicTask1)(void);
void(*PeriodicTask2)(void);
uint32_t periodic_counter = 0; //will need an array for multiple jitter tasks
uint32_t maxJitter1;
uint32_t LastTime;

/*
Timer 4 is used for one periodic thread
*/
//void Timer4A_Handler(void){
  //TIMER4_ICR_R = TIMER_ICR_TATOCINT;
  //(*PeriodicTask1)();
	/*
	static unsigned long lastTime;
	unsigned long jitter;
	unsigned long thisTime;


if(NumSamples < RUNLENGTH){   // finite time run
    thisTime = OS_Time();       // current time, 12.5 ns
		PeriodicTask();
		periodic_counter++;
    if(periodic_counter>1){    // ignore timing of first interrupt
      uint32_t diff = OS_TimeDifference(LastTime,thisTime);
      if(diff>PERIOD){
        jitter = (diff-PERIOD+4)/8;  // in 0.1 usec
      }else{
        jitter = ( PERIOD-diff+4)/8;  // in 0.1 usec
      }
      if(jitter > MaxJitter){
        MaxJitter = jitter; // in usec
      }       // jitter should be 0
      if(jitter >= JitterSize){
        jitter = JitterSize-1;
      }
      JitterHistogram[jitter]++; 
    }
    LastTime = thisTime;
  }
	*/
//}

//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In lab 1, this command will be called 1 time
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), 
   uint32_t period, uint32_t priority){
  // put Lab 2 (and beyond) solution here
  long sr; 
  sr = StartCritical(); // make this function atomic
  if(periodic_thread_cnt < MAXPERIODIC){
    switch (periodic_thread_cnt)
    {
    case 0:
			Timer4A_Init(task, period, priority);
      break;
    case 1:
			Timer1A_Init(task, period, priority);
      break; 
    default:
      break;
    }
    periodic_thread_cnt++;
    EndCritical(sr);
    return 1;
  } else {
    EndCritical(sr);
    return 0;
  }
}


void (*SW1_Task) (void);
void (*SW2_Task) (void);


void SW1_Debounce(void){
  OS_Sleep(10);
  GPIO_PORTF_ICR_R = 0x10; // clear flag
  GPIO_PORTF_IM_R |= 0x10; // arm interrupt again
	OS_Kill();
}

void SW2_Debounce(void){
  OS_Sleep(10);
  GPIO_PORTF_ICR_R = 0x01; // clear flag
  GPIO_PORTF_IM_R |= 0x01; // arm interrupt again
	OS_Kill();
}
/*----------------------------------------------------------------------------
  PF1 Interrupt Handler
 *----------------------------------------------------------------------------*/
void GPIOPortF_Handler(void){
 
  if(GPIO_PORTF_RIS_R & 0x10){ // if SW5 is pressed
    GPIO_PORTF_IM_R &= ~0x10; // disarm interrupt
    (*SW1_Task)();
    int addThreadSuccess = OS_AddThread(&SW1_Debounce, 128, 1); // temporary hardcoded priority
    if(addThreadSuccess == 0){
       GPIO_PORTF_ICR_R = 0x10; // clear flag
       GPIO_PORTF_IM_R |= 0x10; // arm interrupt again 
    }
  }
	
	  if(GPIO_PORTF_RIS_R & 0x01){ // if SW1 is pressed
    GPIO_PORTF_IM_R &= ~0x01; // disarm interrupt
    (*SW1_Task)();
    int addThreadSuccess = OS_AddThread(&SW2_Debounce, 128, 1); // temporary hardcoded priority
    if(addThreadSuccess == 0){
       GPIO_PORTF_ICR_R = 0x01; // clear flag
       GPIO_PORTF_IM_R |= 0x01; // arm interrupt again 
    }
  }

}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), uint32_t priority){
  long volatile delay;
	SYSCTL_RCGCGPIO_R |= 0x00000020; 	// (a) activate clock for port F
  delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTF_CR_R = 0x10;           // allow changes to PF4
  GPIO_PORTF_DIR_R &= ~0x10;    		// (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  		//     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     		//     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       		//     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     		//     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     		// (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;     		//     PF4 is not edges
	GPIO_PORTF_IEV_R &= ~0x10;     		//     PF4 falling edges

	GPIO_PORTF_ICR_R = 0x10;      		// (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      		// (f) arm interrupt on PF4
	
	priority = (priority & 0x07) << 21;	// NVIC priority bit (21-23)
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF); // clear priority
	NVIC_PRI7_R = (NVIC_PRI7_R | priority); 
  NVIC_EN0_R = 0x40000000;      		// (h) enable interrupt 30 in NVIC  

  SW1_Task = task;

  return 1; // replace this line with solution
};

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here
    long volatile delay;
	SYSCTL_RCGCGPIO_R |= 0x00000020; 	// (a) activate clock for port F
  delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTF_CR_R = 0x01;           // allow changes to PF4
  GPIO_PORTF_DIR_R &= ~0x01;    		// (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x01;  		//     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x01;     		//     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x0000000F; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       		//     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x01;     		//     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x01;     		// (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x01;     		//     PF4 is not edges
	GPIO_PORTF_IEV_R &= ~0x01;     		//     PF4 falling edges

	GPIO_PORTF_ICR_R = 0x01;      		// (e) clear flag4
  GPIO_PORTF_IM_R |= 0x01;      		// (f) arm interrupt on PF4
	
	priority = (priority & 0x07) << 21;	// NVIC priority bit (21-23)
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF); // clear priority
	NVIC_PRI7_R = (NVIC_PRI7_R | priority); 
  NVIC_EN0_R = 0x40000000;      		// (h) enable interrupt 30 in NVIC  

  SW2_Task = task;

  return 1; // replace this line with solution
};


// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  DisableInterrupts();
  RunPt->sleep_ms = sleepTime;
	RunPt->current_state = SLEEP;
  EnableInterrupts();
	OS_Suspend();

};  

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
  // put Lab 2 (and beyond) solution here
  DisableInterrupts();
  (RunPt->prev)->next = RunPt->next;
	(RunPt->next)->prev = RunPt->prev;
  RunPt->current_state = DEAD;
  RunPt->sleep_ms = 0;
  if(RunPt == lastTCB){
    lastTCB = RunPt->prev;
  }
	NumThreads_Global--;
  EnableInterrupts();   // end of atomic section 
	OS_Suspend();

  for(;;){};        // can not return
}; 

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
  // put Lab 2 (and beyond) solution here
  // call context switch for part 1
		//NVIC_ST_CURRENT_R = 0;
    ContextSwitch();
};
  
// ******** Fifo Globals ************
Sema4Type Fifo_CurrentSize;
uint32_t *Fifo_PutPt;
uint32_t *Fifo_GetPt;
uint32_t Fifo[MAX_FIFO];
#define FIFOSUCCESS 1 
#define FIFOFAIL 0

// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(uint32_t size){
  // initializes fifo array, initializes semaphore to control access
  long sr;
	sr = StartCritical();
  Fifo_PutPt = &Fifo[0];
  Fifo_GetPt = &Fifo[0];
  OS_InitSemaphore(&Fifo_CurrentSize, 0);
  EndCritical(sr);
};

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(uint32_t data){
  // put Lab 2 (and beyond) solution here
  // Check to make sure there is space in the fifo
  if (Fifo_CurrentSize.Value == MAX_FIFO){
    return FIFOFAIL;
  } else {
    OS_Signal(&Fifo_CurrentSize);
    // insert data and update put pointer
    *Fifo_PutPt = data; 
    Fifo_PutPt += 1;

    // if the fifo is full make sure to loop back around
    if(Fifo_PutPt == &Fifo[MAX_FIFO]){
      Fifo_PutPt = &Fifo[0];
    }
    return FIFOSUCCESS;
  }
}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
uint32_t OS_Fifo_Get(void){
  // wait until there's elements on the fifo
  OS_Wait(&Fifo_CurrentSize);
  if(Fifo_GetPt == Fifo_PutPt){
    return FIFOFAIL;
  } else {
    uint32_t data = *Fifo_GetPt;
    Fifo_GetPt += 1;
    if(Fifo_GetPt == &Fifo[MAX_FIFO]){
      Fifo_GetPt = &Fifo[0];
      return data;
    }
  } 
	return FIFOFAIL;
};

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
int32_t OS_Fifo_Size(void){
  // put Lab 2 (and beyond) solution here 
  return Fifo_CurrentSize.Value;
};

// ******** Mailbox Globals ************
MailBox_t mailbox;
// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
  // put Lab 2 (and beyond) solution here
  OS_InitSemaphore(&mailbox.Send, 0);
  OS_InitSemaphore(&mailbox.Ack, 1);
};

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(uint32_t data){
  // put Lab 2 (and beyond) solution here
  OS_bWait(&mailbox.Send);
  mailbox.mail = data;
  OS_bSignal(&mailbox.Ack);
};

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
uint32_t OS_MailBox_Recv(void){
  // put Lab 2 (and beyond) solution here
  uint32_t data;
  OS_bWait(&mailbox.Ack);
  data = mailbox.mail;
  OS_bSignal(&mailbox.Send);
  return data;
};

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
uint32_t OS_Time(void){
  // put Lab 2 (and beyond) solution here
  return MsTime;
};

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
uint32_t OS_TimeDifference(uint32_t start, uint32_t stop){
  // put Lab 2 (and beyond) solution here
  // if (start > stop) {
  //   return start - stop;
  // } else {
  //   stop - start;
  // }

  //return start > stop ? (stop + UINT32_MAX - start) : stop - start;
	int32_t time_difference = stop - start;
	if(time_difference < 0){
		time_difference += UINT32_MAX;
	}
	return time_difference;
};

// ******** OS_ClearMsTime ************
// sets the system time to zero (solve for Lab 1), and start a periodic interrupt
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
  // put Lab 1 solution here
    MsTime = 0;
		TIMER3_TAR_R = TIME1MS - 1;
};

// ******** OS_MsTime ************
// reads the current time in msec (solve for Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// For Labs 2 and beyond, it is ok to make the resolution to match the first call to OS_AddPeriodicThread
uint32_t OS_MsTime(void){
  // put Lab 1 solution here
  return MsTime; // replace this line with solution
};


//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(uint32_t theTimeSlice){
  NVIC_ST_RELOAD_R = theTimeSlice - 1;
  NVIC_ST_CTRL_R = 0x00000007;
  StartOS();
    
};

//************** I/O Redirection *************** 
// redirect terminal I/O to UART or file (Lab 4)

int StreamToDevice=0;                // 0=UART, 1=stream to file (Lab 4)

int fputc (int ch, FILE *f) { 
  if(StreamToDevice==1){  // Lab 4
    if(eFile_Write(ch)){          // close file on error
       OS_EndRedirectToFile(); // cannot write to file
       return 1;                  // failure
    }
    return 0; // success writing
  }
  
  // default UART output
  UART_OutChar(ch);
  return ch; 
}

int fgetc (FILE *f){
  char ch = UART_InChar();  // receive from keyboard
  UART_OutChar(ch);         // echo
  return ch;
}

int OS_RedirectToFile(const char *name){  // Lab 4
  eFile_Create(name);              // ignore error if file already exists
  if(eFile_WOpen(name)) return 1;  // cannot open file
  StreamToDevice = 1;
  return 0;
}

int OS_EndRedirectToFile(void){  // Lab 4
  StreamToDevice = 0;
  if(eFile_WClose()) return 1;    // cannot close file
  return 0;
}

int OS_RedirectToUART(void){
  StreamToDevice = 0;
  return 0;
}

int OS_RedirectToST7735(void){
  
  return 1;
}
