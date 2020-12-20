;/*****************************************************************************/
; OSasm.s: low-level OS commands, written in assembly                       */
; Runs on LM4F120/TM4C123/MSP432
; Lab 2 starter file
; February 10, 2016
;


        AREA |.text|, CODE, READONLY, ALIGN=2
        THUMB
        REQUIRE8
        PRESERVE8

        EXTERN  RunPt            ; currently running thread
        EXPORT  StartOS
        EXPORT  SysTick_Handler
        IMPORT  Scheduler


SysTick_Handler                ; 1) Saves R0-R3,R12,LR,PC,PSR
    CPSID   I                  ; 2) Prevent interrupt during switch
	PUSH  	{R4-R11}		    ; push to save the remaining registers manually
	LDR 	R0, =RunPt		    ; R0 = &RunPt
	LDR		R1, [R0]		    ; R1 = RunPt
	STR		SP, [R1]		    ; processor's SP is saved (updated) into SP field of running thread's TCB
	
	PUSH	{R0,LR}	
	BL		Scheduler
	POP		{R0,LR}
	LDR		R1, [R0]			;; 6) R1 = RunPt, new threa
	
	;	without linking C file
	;LDR		R1, [R1, #4]		; Get next field in running thread's TCB
	;STR		R1, [R0]			; Update RunPt to next
	
	LDR		SP, [R1]			; Update system's SP with next thread's SP
	POP		{R4-R11}
	
	
    CPSIE   I                  ; 9) tasks run with interrupts enabled
    BX      LR                 ; 10) restore R0-R3,R12,LR,PC,PSR

StartOS
	
	LDR R0, =RunPt	;
	LDR R1, [R0]	;	
	LDR SP, [R1];
	
	POP {R4-R11}
	POP {R0-R3}
	POP {R12}
	ADD SP, SP, #4
	POP {LR}			; why??
	ADD SP, SP, #4
		
    CPSIE   I                  ; Enable interrupts at processor level
    BX      LR                 ; start first thread

    ALIGN
    END
