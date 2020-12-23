// os.c
// Runs on LM4F120/TM4C123/MSP432
// A priority/blocking real-time operating system 
// Lab 4 starter file.
// Daniel Valvano
// March 25, 2016
// Hint: Copy solutions from Lab 3 into Lab 4
#include <stdint.h>
#include "os.h"
#include "CortexM.h"
#include "BSP.h"
#include "../inc/tm4c123gh6pm.h"

// function definitions in osasm.s
void StartOS(void);

#define NUMTHREADS  8        // maximum number of threads
#define NUMPERIODIC 2        // maximum number of periodic threads
#define STACKSIZE   100      // number of 32-bit words in stack per thread
#define LOWESTPRIORITY 255 // 255
struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
	int32_t *blocked;// nonzero if blocked on this semaphore
	uint32_t sleepTime; // nonzero if this thread is sleeping
	uint8_t priority;  // 0 is highest, 254 lowest
	
//*FILL THIS IN****
};
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];
void static runperiodicevents(void);

// ******** OS_Init ************
// Initialize operating system, disable interrupts
// Initialize OS controlled I/O: periodic interrupt, bus clock as fast as possible
// Initialize OS global variables
// Inputs:  none
// Outputs: none
void OS_Init(void){
  DisableInterrupts();
  BSP_Clock_InitFastest();// set processor clock to fastest speed
// perform any initializations needed, 
// set up periodic timer to run runperiodicevents to implement sleeping
	
  
}

void SetInitialStack(int i){
  // ****IMPLEMENT THIS**** 
  // **Same as Lab 2 and Lab 3****
	tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000; // Thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414; // R14
  Stacks[i][STACKSIZE-4] = 0x12121212; // R12
  Stacks[i][STACKSIZE-5] = 0x03030303; // R3
  Stacks[i][STACKSIZE-6] = 0x02020202; // R2
  Stacks[i][STACKSIZE-7] = 0x01010101; // R1
  Stacks[i][STACKSIZE-8] = 0x00000000; // R0
  Stacks[i][STACKSIZE-9] = 0x11111111; // R11
  Stacks[i][STACKSIZE-10] = 0x10101010; // R10
  Stacks[i][STACKSIZE-11] = 0x09090909; // R9
  Stacks[i][STACKSIZE-12] = 0x08080808; // R8
  Stacks[i][STACKSIZE-13] = 0x07070707; // R7
  Stacks[i][STACKSIZE-14] = 0x06060606; // R6
  Stacks[i][STACKSIZE-15] = 0x05050505; // R5
  Stacks[i][STACKSIZE-16] = 0x04040404; // R4
}

//******** OS_AddThreads ***************
// Add eight main threads to the scheduler
// Inputs: function pointers to eight void/void main threads
//         priorites for each main thread (0 highest)
// Outputs: 1 if successful, 0 if this thread can not be added
// This function will only be called once, after OS_Init and before OS_Launch
int OS_AddThreads(void(*thread0)(void), uint32_t p0,
                  void(*thread1)(void), uint32_t p1,
                  void(*thread2)(void), uint32_t p2,
                  void(*thread3)(void), uint32_t p3,
                  void(*thread4)(void), uint32_t p4,
                  void(*thread5)(void), uint32_t p5,
                  void(*thread6)(void), uint32_t p6,
                  void(*thread7)(void), uint32_t p7){
// **similar to Lab 3. initialize priority field****
										
  int32_t status = StartCritical();
										
	tcbs[0].next = &tcbs[1]; // 0 points to 1
	tcbs[1].next = &tcbs[2]; // 1 points to 2
	tcbs[2].next = &tcbs[3]; // 2 points to 3
	tcbs[3].next = &tcbs[4]; // 
	tcbs[4].next = &tcbs[5]; // 
	tcbs[5].next = &tcbs[6]; //
  tcbs[6].next = &tcbs[7]; // 	
  tcbs[7].next = &tcbs[0]; // 											
	

	for(int i = 0; i<NUMTHREADS; i++)
	{
		tcbs[i].blocked = 0;
		tcbs[i].sleepTime = 0;
	}
	
	tcbs[0].priority = p0;
	tcbs[1].priority = p1;
	tcbs[2].priority = p2;
	tcbs[3].priority = p3;
	tcbs[4].priority = p4;
	tcbs[5].priority = p5;
	tcbs[6].priority = p6;
	tcbs[7].priority = p7;
	
	SetInitialStack(0); Stacks[0][STACKSIZE-2] = (int32_t)(thread0); // PC					
	SetInitialStack(1); Stacks[1][STACKSIZE-2] = (int32_t)(thread1); // PC
	SetInitialStack(2); Stacks[2][STACKSIZE-2] = (int32_t)(thread2); // PC											
	SetInitialStack(3); Stacks[3][STACKSIZE-2] = (int32_t)(thread3); // PC
	SetInitialStack(4); Stacks[4][STACKSIZE-2] = (int32_t)(thread4); // PC
	SetInitialStack(5); Stacks[5][STACKSIZE-2] = (int32_t)(thread5); // PC
	SetInitialStack(6); Stacks[6][STACKSIZE-2] = (int32_t)(thread6); // PC
	SetInitialStack(7); Stacks[7][STACKSIZE-2] = (int32_t)(thread7); // PC
	

  RunPt = 	&tcbs[0];

  EndCritical(status);	

  return 1;               // successful
}


void static runperiodicevents(void){
// ****IMPLEMENT THIS****
// **DECREMENT SLEEP COUNTERS
// In Lab 4, handle periodic events in RealTimeEvents
	uint32_t i;
	
  for(i=0; i<NUMTHREADS; i++)
	{
		if(tcbs[i].sleepTime)
		{
		  tcbs[i].sleepTime--;
		}
	}
//	uint32_t i;
//	
//  for(i=0; i<NUMTHREADS; i++)
//	{
//		if(tcbs[i].sleepTime)
//		{
//		  tcbs[i].sleepTime--;
//		}
//		
//	}
//	
//	  for (i = 0; i < PeriodicThreadCounter; i++) {
//    if (PeriodicThreads[i].timeLeft == 0) {
//      PeriodicThreads[i].thread();
//      PeriodicThreads[i].timeLeft = PeriodicThreads[i].period-1; // reload
//    } else {
//      PeriodicThreads[i].timeLeft--;
//    }
//  }
//		
  
}

//******** OS_Launch ***************
// Start the scheduler, enable interrupts
// Inputs: number of clock cycles for each time slice
// Outputs: none (does not return)
// Errors: theTimeSlice must be less than 16,777,216
void OS_Launch(uint32_t theTimeSlice){
  STCTRL = 0;                  // disable SysTick during setup
  STCURRENT = 0;               // any write to current clears it
  SYSPRI3 =(SYSPRI3&0x00FFFFFF)|0xE0000000; // priority 7
  STRELOAD = theTimeSlice - 1; // reload value
  STCTRL = 0x00000007;         // enable, core clock and interrupt arm
	
	BSP_PeriodicTask_Init(&runperiodicevents, 1000,1);
	
  StartOS();                   // start on the first task
}
// runs every ms
void Scheduler(void){      // every time slice
// ****IMPLEMENT THIS****
// look at all threads in TCB list choose
// highest priority thread not blocked and not sleeping 
// If there are multiple highest priority (not blocked, not sleeping) run these round robin
  
	int curr_max = 255;
	tcbType *currPt = RunPt; 
	tcbType *bestPt;
	
	do // search for highest thread not blocked or sleeping
	{
		currPt = currPt->next; // skip at least one
	
	 if( (currPt->priority < curr_max) && (currPt->sleepTime == 0) && (currPt->blocked==0))
	  {
	  bestPt = currPt;
    curr_max = currPt->priority;
	  }
  }while(currPt != RunPt); // look at all possible threads
	
	RunPt = bestPt; 
	
}

//******** OS_Suspend ***************
// Called by main thread to cooperatively suspend operation
// Inputs: none
// Outputs: none
// Will be run again depending on sleep/block status
void OS_Suspend(void){
  STCURRENT = 0;        // any write to current clears it
  INTCTRL = 0x04000000; // trigger SysTick
// next thread gets a full time slice
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
// ****IMPLEMENT THIS****
// set sleep parameter in TCB, same as Lab 3
// suspend, stops running
	DisableInterrupts();
	RunPt->sleepTime = sleepTime;
	EnableInterrupts();
	
	OS_Suspend();

}

// ******** OS_InitSemaphore ************
// Initialize counting semaphore
// Inputs:  pointer to a semaphore
//          initial value of semaphore
// Outputs: none
void OS_InitSemaphore(int32_t *semaPt, int32_t value){
// ****IMPLEMENT THIS****
// Same as Lab 3
	*semaPt = value;
}

// ******** OS_Wait ************
// Decrement semaphore and block if less than zero
// Lab2 spinlock (does not suspend while spinning)
// Lab3 block if less than zero
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Wait(int32_t *semaPt){
// ****IMPLEMENT THIS****
// Same as Lab 3
	DisableInterrupts();
	*semaPt = (*semaPt) - 1;
	
	if (*semaPt < 0) // [?] why not less or equal to?
	{
		//block this thread
		RunPt -> blocked = semaPt;
		EnableInterrupts();
		OS_Suspend();
		
	}
	EnableInterrupts();
}

// ******** OS_Signal ************
// Increment semaphore
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Signal(int32_t *semaPt){
// ****IMPLEMENT THIS****
// Same as Lab 3
	tcbType *tempPt;
	DisableInterrupts();
	
	*semaPt = *semaPt + 1;
	if (*semaPt <= 0) // [?] why -> someone must have just woken up / if semaphore was 0 and u called signal, it would just increment it to one and 
	{
		//Wake up one thread
		tempPt = RunPt->next; // search for thread blocked on this semaphore (address) 
		while( tempPt->blocked != semaPt)
		{
			tempPt = tempPt -> next;
		}
		tempPt->blocked = 0; //wakeup this one
	}
	EnableInterrupts();
}

#define FSIZE 10    // can be any size
uint32_t PutI;      // index of where to put next
uint32_t GetI;      // index of where to get next
uint32_t Fifo[FSIZE];
int32_t CurrentSize;// 0 means FIFO empty, FSIZE means full
uint32_t LostData;  // number of lost pieces of data

// ******** OS_FIFO_Init ************
// Initialize FIFO.  The "put" and "get" indices initially
// are equal, which means that the FIFO is empty.  Also
// initialize semaphores to track properties of the FIFO
// such as size and busy status for Put and Get operations,
// which is important if there are multiple data producers
// or multiple data consumers.
// Inputs:  none
// Outputs: none
void OS_FIFO_Init(void){
// ****IMPLEMENT THIS****
// Same as Lab 3
	PutI = GetI = 0;
	OS_InitSemaphore(&CurrentSize, 0); // blocking semaphore initialized
	LostData = 0;
  
}

// ******** OS_FIFO_Put ************
// Put an entry in the FIFO.  Consider using a unique
// semaphore to wait on busy status if more than one thread
// is putting data into the FIFO and there is a chance that
// this function may interrupt itself.
// Inputs:  data to be stored
// Outputs: 0 if successful, -1 if the FIFO is full
int OS_FIFO_Put(uint32_t data){
// ****IMPLEMENT THIS****
// Same as Lab 3
	if(CurrentSize == FSIZE)
	{
		LostData++;
		return -1; // full
	}
	else
	{
		Fifo[PutI] = data; // Put
		PutI = (PutI+1)%FSIZE; // [?] what does this do -> update circular buffer index
	  OS_Signal(&CurrentSize);
		return 0; // success
	}
}

// ******** OS_FIFO_Get ************
// Get an entry from the FIFO.  Consider using a unique
// semaphore to wait on busy status if more than one thread
// is getting data from the FIFO and there is a chance that
// this function may interrupt itself.
// Inputs:  none
// Outputs: data retrieved
uint32_t OS_FIFO_Get(void){uint32_t data;
// ****IMPLEMENT THIS****
// Same as Lab 3
	OS_Wait(&CurrentSize); // block if empty
	data = Fifo[GetI]; // get
	GetI = (GetI+1)%FSIZE; // place to get next
 return data;
}
// *****periodic events****************
int32_t *PeriodicSemaphore0;
uint32_t Period0; // time between signals
int32_t *PeriodicSemaphore1;
uint32_t Period1; // time between signals
void RealTimeEvents(void){int flag=0;
  static int32_t realCount = -10; // let all the threads execute once
  // Note to students: we had to let the system run for a time so all user threads ran at least one
  // before signalling the periodic tasks
  realCount++;
  if(realCount >= 0){
		if((realCount%Period0)==0){
      OS_Signal(PeriodicSemaphore0);
      flag = 1;
		}
    if((realCount%Period1)==0){
      OS_Signal(PeriodicSemaphore1);
      flag=1;
		}
    if(flag){
      OS_Suspend();
    }
  }

}
// ******** OS_PeriodTrigger0_Init ************
// Initialize periodic timer interrupt to signal 
// Inputs:  semaphore to signal
//          period in ms
// priority level at 0 (highest
// Outputs: none
void OS_PeriodTrigger0_Init(int32_t *semaPt, uint32_t period){
	PeriodicSemaphore0 = semaPt;
	Period0 = period;
	BSP_PeriodicTask_InitC(&RealTimeEvents,1000,0);
}
// ******** OS_PeriodTrigger1_Init ************
// Initialize periodic timer interrupt to signal 
// Inputs:  semaphore to signal
//          period in ms
// priority level at 0 (highest
// Outputs: none
void OS_PeriodTrigger1_Init(int32_t *semaPt, uint32_t period){
	PeriodicSemaphore1 = semaPt;
	Period1 = period;
	BSP_PeriodicTask_InitC(&RealTimeEvents,1000,0);
}

//****edge-triggered event************
int32_t *edgeSemaphore;
// ******** OS_EdgeTrigger_Init ************
// Initialize button1, PD6, to signal on a falling edge interrupt
// Inputs:  semaphore to signal
//          priority
// Outputs: none
void OS_EdgeTrigger_Init(int32_t *semaPt, uint8_t priority){
	edgeSemaphore = semaPt;
//***IMPLEMENT THIS***
// 1) activate clock for Port D
// allow time for clock to stabilize
// 2) no need to unlock PD6
// 3) disable analog on PD6
// 4) configure PD6 as GPIO
// 5) make PD6 input
// 6) disable alt funct on PD6
// disable pull-up on PD6
// 7) enable digital I/O on PD6  
// (d) PD6 is edge-sensitive 
//     PD6 is not both edges 
//     PD6 is falling edge event 
// (e) clear PD6 flag
// (f) arm interrupt on PD6
// priority on Port D edge trigger is NVIC_PRI0_R	31 � 29
// enable is bit 3 in NVIC_EN0_R
	SYSCTL_RCGCGPIO_R |= 0x00000008; // 1) activate clock for Port D
	while((SYSCTL_PRGPIO_R&0x08) == 0){}; // allow time for clock to stabilize
	GPIO_PORTD_AMSEL_R &= ~0x40; // 2) disable analog on PD6
	GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xF0FFFFFF)+0x00000000; // 3) configure PD6 as GPIO
	GPIO_PORTD_DIR_R &= ~0x40; // 4) make PD6 input
  GPIO_PORTD_AFSEL_R &= ~0x40;// 5) disable alt funct on PD6
  GPIO_PORTD_PUR_R &= ~0x40; // disable pull-up on PD6
  GPIO_PORTD_DEN_R |= 0x40; // 6) enable digital I/O on PD6  
	GPIO_PORTD_IS_R &= ~0x40; // PD6 is edge-sensitive 
	GPIO_PORTD_IBE_R &= ~0x40; // PD6 is not both edges 
	GPIO_PORTD_IEV_R &= ~0x40;  // PD6 is falling edge event 
	GPIO_PORTD_ICR_R = 0x40; // clear PD6 flag
	GPIO_PORTD_IM_R |= 0x40; // arm interrupt on PD6
	NVIC_PRI0_R = (NVIC_PRI0_R&0x1FFFFFFF)|((uint32_t)(priority) << 29); // priority on Port D edge trigger is NVIC_PRI0_R	31 ? 29
	NVIC_EN0_R = 0x00000008; // enable is bit 3 in NVIC_EN0_R
	
	
 }

// ******** OS_EdgeTrigger_Restart ************
// restart button1 to signal on a falling edge interrupt
// rearm interrupt
// Inputs:  none
// Outputs: none
void OS_EdgeTrigger_Restart(void){
//***IMPLEMENT THIS***
// rearm interrupt 3 in NVIC
// clear flag6
	GPIO_PORTD_IM_R |= 0x40; // rearm interrupt 3 in NVIC
  GPIO_PORTD_ICR_R = 0x40; // clear flag6
}
void GPIOPortD_Handler(void){
//***IMPLEMENT THIS***
	// step 1 acknowledge by clearing flag
  // step 2 signal semaphore (no need to run scheduler)
  // step 3 disarm interrupt to prevent bouncing to create multiple signals
		GPIO_PORTD_ICR_R = 0x40; // step 1 acknowledge by clearing flag
  OS_Signal(edgeSemaphore); // step 2 signal semaphore (no need to run scheduler)
  GPIO_PORTD_IM_R &= ~0x40;// step 3 disarm interrupt to prevent bouncing to create multiple signals
	OS_Suspend();
}


