            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
;PROGRAM EVALUATES THE EXPRESSION AND CHECKS FOR OVERFLOW IN THE EXPRESSION.
;Name:  Hrishikesh Moholkar
;Date:  09/09/2016
;Class:  CMPE-250
;Section:  01L1 Tuesday 2:00 TO 4:00
;---------------------------------------------------------------
;Keil Simulator Template for KL46
;R. W. Melton
;January 23, 2015
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;EQUates
MUL2 EQU 1
MUL4 EQU 2
;Vectors
VECTOR_TABLE_SIZE EQU 0x000000C0
VECTOR_SIZE       EQU 4           ;Bytes per vector
;Stack
SSTACK_SIZE EQU  0x00000100
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
Reset_Handler
main
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
            LDR R1,=P 			 ; loading address of variable p into R1
			LDR R1,[R1,#0]       ; storing the memory content from variable into r1
			LDR R2,=Q
			LDR R2,[R2,#0]
			LDR R3,=R
			LDR R3,[R3,#0]
			MOVS R0,R1
			LSLS R0,R0,#MUL4  	  ; r0=4*P
			CMP  R0,#127          ; compare with the greatest value
			BGT  ERROR            ; Branch to label if greater than the value
			MOVS R5,#128
			CMN  R0,R5            ; compare with negative number
	        BLT  ERROR            ; branch to label if less than the largest negativ evalue
        	MOVS R4,R2
			LSLS R4,R4,#MUL2 	  ; r4=2*q
	        CMP  R4,#127
			BGT  ERROR
			MOVS R5,#128
			CMN  R4,R5
			BLT  ERROR
			SUBS R0,R0,R4         ; r0=r0-r4
			CMP  R0,#127
			BGT  ERROR
			MOVS R5,#128
			CMN  R0,R5
			BLT  ERROR
	
			SUBS R0,R0,R3		 ; r0=4p-2q-r
			CMP  R0,#127
			BGT  ERROR
			MOVS R5,#128
			CMN  R0,R5
			BLT  ERROR
			
			LDR  R6,=const_F
			LDR  R6,[R6,#0]
			ADDS R0,R0,R6        ; r0 = r0+81
			CMP  R0,#127
			BGT  ERROR
			MOVS R5,#128
			CMN  R0,R5
			BLT  ERROR
			
			
			LDR R4,=F             ; GET ADDRESS OF NUMBER IN R4
			STR R0,[R4,#0]        ; STORE IN VARIABLE F THE VALUE OF EXPRESSION
	
CONTINUE1 	MOVS R0,R1
			LSLS R0,R0,#MUL2	   ; R0=2*P
			ADDS R0,R0,R1 	       ; R0= 3p
			CMP  R0,#127
			BGT  ERROR1
			MOVS R5,#128
			CMN  R0,R5
			BLT  ERROR1
			MOVS R4,R2
			LSLS R4,R4,#MUL4        ; R4=4*Q
			CMP  R4,#127
			BGT  ERROR1
			MOVS R5,#128
			CMN  R4,R5
			BLT  ERROR1
			MOVS R5,R2
			LSLS R5,R5,#MUL2
			ADDS R5,R5,R2           ; R5=3*Q
			CMP  R5,#127
			BGT  ERROR1
			MOVS R6,#128
			CMN  R5,R6
			BLE  ERROR1
			ADDS R4,R4,R5           ;R4=7*Q
			CMP  R4,#127
			BGT  ERROR1
			MOVS R5,#128
			CMN R4,R5
			BLT ERROR1
			SUBS R0,R0,R4           ; R0=3P-7Q
			CMP  R0,#127
			BGT  ERROR1
			MOVS R5,#128
			CMN R0,R5
			BLT ERROR1		
			LSLS R4,R3,#MUL2        ;R4=2R
			CMP R4,#127
			BGT ERROR1
			MOVS R5,#128
			CMN R4,R5
			BLT ERROR1
			ADDS R0,R0,R4 			; R0=3P-7Q+2R
			LDR  R5,=const_G
			LDR  R5,[R5,#0]
			ADDS R0,R0,R5           ;R0=3p-7q+2r+11
			LDR R4,=G
			STR R0,[R4,#0]	
			
CONTINUE2	
            MOVS R0,R1
			MOVS R4,R2
			SUBS R0,R0,R4            ;R0=P-Q
			MOVS R4,#0
			MOVS R4,R3
			LSLS R4,R4,#MUL2
			ADDS R4,R4,R3            ;R4=3R
			CMP R4,#127
			BGT ERROR2
			MOVS R5,#128
			CMN R4,R5
			BLT ERROR2
			ADDS R0,R0,R4             ; R0=P-Q+3R
			CMP R0,#127
			BGT ERROR2
			MOVS R6,#128
			CMN R0,R6
			BLT ERROR2
			LDR R4,=const_H
			LDR R4,[R4,#0]
			ADDS R0,R0,R4
			CMP R0,#127
			BGT ERROR2
			MOVS R6,#128
			CMN R0,R6
			BLT ERROR2                ;R0=P-Q+3R-107
			LDR R5,=H
			STR R0,[R5,#0]
		
CONTINUE3			LDR R0,=F
					LDR R0,[R0,#0]
			
					LDR R4,=G
					LDR R4,[R4,#0]
					
					LDR R5,=H
					LDR R5,[R5,#0]
					
					ADDS R0,R0,R4
					CMP R0,#127
					BGT ERROR3
					MOVS R6,#128
					CMN R0,R6
					BLT ERROR3
					ADDS R0,R0,R5          ;R0=F+G+H
					
					LDR R4,=Result
					STR R0,[R4,#0]        ;RESULT =F+G+H	

;>>>>>   end main program code <<<<<
;Stay here
            B       .
ERROR MOVS R0,#0
	  LDR  R4,=F
	  STR  R0,[R4,#0]       ;F=0
	   B   CONTINUE1		   
ERROR1 MOVS R0,#0
	   LDR  R4,=G
	   STR  R0,[R4,#0]
	   B	CONTINUE2       ;G=0	   
ERROR2 MOVS R0,#0
       LDR  R4,=H
	   STR  R0,[R4,#0]
	   B    CONTINUE3       ;H=0
	   
ERROR3 MOVS R0,#0
	   LDR R5,=Result
	   STR R0,[R5,#0]      ;Result=0
	   
;---------------------------------------------------------------
;>>>>> begin subroutine code <<<<<
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;reset vector
            SPACE  (VECTOR_TABLE_SIZE - (2 * VECTOR_SIZE))
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
const_F	DCD	81
const_G	DCD	11
const_H	DCD	-107
;>>>>>   end constants here <<<<<
;****************************************************************
            AREA    |.ARM.__at_0x1FFFE000|,DATA,READWRITE,ALIGN=3
            EXPORT  __initial_sp
;Allocate system stack
            IF      :LNOT::DEF:SSTACK_SIZE
SSTACK_SIZE EQU     0x00000100
            ENDIF
Stack_Mem   SPACE   SSTACK_SIZE
__initial_sp
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
F	SPACE 4
G	SPACE 4
H	SPACE 4
P	SPACE 4
Q	SPACE 4
R	SPACE 4
Result	SPACE 4
;>>>>>   end variables here <<<<<
            END