            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
;A subroutine used for performing the division operation on numbers
;Name:  Hrishikesh Moholkar
;Date:  09/13
;Class:  CMPE-250
;Section:  <01L1, Tuesday, 2-4>
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
MAX_DATA EQU 25
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
		BL 	InitData
		LDR R1,=P       ; ADDRESS OF P STORED IN R1
		LDR R2,=Q       ; ADDDRESS OF Q STORED IN R2

while	BL LoadData
		
		BCS QUIT
		LDR R1,=P       ; ADDRESS OF P STORED IN R1
		LDR R2,=Q       ; ADDDRESS OF Q STORED IN R2
		LDR R1,[R1,#0]  ; LOADING R1 i.e. P INTO R1
		LDR R0,[R2,#0]  ; LOADING R2 I.E. Q INTO R0
		BL DIVU         ; Branch to subroutine
		BCC  VALID

		BCS  INVALID
		
VALID  	  
   		  LDR R5,=P        ; STORE THE ADDRESS IN R5
		                    
		  LDR R4,=Q        ; STORE ADDRESS OF Q IN R4
		  
		  STR R0,[R5,#0]     ; STORE THE quotient IN THE ADDRESS OF P
		  STR R1,[R4,#0]     ; STORE THE remainder IN R4 IN THE ADDRESS OF Q
		  BL   TestData
		  B    while


INVALID   LDR R3,=P
		  LDR R4,=0xFFFFFFFF
		  STR R4,[R3,#0]        ; STORE THE VALUE IN THE ADDRESS OF P
		  LDR R5,=Q
		  STR R4,[R5,#0]        ; STORE THE VALUE IN THE ADDRESS OF Q
		  BL   TestData
          B    while
		  
		
		
		
;>>>>>   end main program code <<<<<
;Stay here 

QUIT
            B       .

		  
;---------------------------------------------------------------
;>>>>> begin subroutine code <<<<<
;****************************************************************
;Machine code provided for Exercise Four
;R. W. Melton 9/12/2016
;Place at the end of your MyCode AREA
DIVU	    PUSH {R2-R5}        ; pushing registers to be used in subroutine for operations
			CMP R0,#0           ; COMPARE DIVISOR WITH ZERO
			BEQ CARRYSET
			
			MOVS R5,R0          ;STORING DIVISOR INTO R5
			MOVS R0,#0          ; QUOTIENT IS ZERO
			
While1       CMP  R1,R5
            BLO  FINISH
            SUBS R1,R1,R5
            ADDS R0,R0,#1
			 B    While1
			
			
           
CARRYSET    MRS   R5,APSR         ; carry set 
            MOVS  R4,#0x20 
			LSLS  R4,R4,#24 
			ORRS  R5,R5,R4
			MSR   APSR,R5
			B     RETURN 

FINISH      MRS   R5,APSR 	           ;carry clear
			MOVS  R4,#0x20            
			LSLS  R4,R4,#24 
			BICS  R5,R5,R4
			MSR   APSR,R5
RETURN		POP	{R2-R5}        ; popping registers used in the registers
            BX    LR

;****************************************************************
;>>>>>   end subroutine code <<<<<
            ALIGN
							
            AREA    |.ARM.__at_0x4000|,CODE,READONLY
InitData    DCI.W   0x26002700
            DCI     0x4770
LoadData    DCI.W   0xB40FA316
            DCI.W   0x19DBA13D
            DCI.W   0x428BD209
            DCI.W   0xCB034A10
            DCI.W   0x4B116010
            DCI.W   0x60193708
            DCI.W   0x20000840
            DCI.W   0xBC0F4770
            DCI.W   0x20010840
            DCI     0xE7FA
TestData    DCI.W   0xB40F480C
            DCI.W   0xA13419C0
            DCI.W   0x19C93808
            DCI.W   0x39084A07
            DCI.W   0x4B076812
            DCI.W   0x681BC00C
            DCI.W   0x68084290
            DCI.W   0xD1046848
            DCI.W   0x4298D101
            DCI.W   0xBC0F4770
            DCI.W   0x1C76E7FB
            ALIGN
PPtr        DCD     P
QPtr        DCD     Q
ResultsPtr  DCD     Results
            DCQ     0x0000000000000000,0x0000000000000001
            DCQ     0x0000000100000000,0x0000000100000010
            DCQ     0x0000000200000010,0x0000000400000010
            DCQ     0x0000000800000010,0x0000001000000010
            DCQ     0x0000002000000010,0x0000000100000007
            DCQ     0x0000000200000007,0x0000000300000007
            DCQ     0x0000000400000007,0x0000000500000007
            DCQ     0x0000000600000007,0x0000000700000007
            DCQ     0x0000000800000007,0x8000000080000000
            DCQ     0x8000000180000000,0x000F0000FFFFFFFF
            DCQ     0xFFFFFFFFFFFFFFFF,0xFFFFFFFFFFFFFFFF
            DCQ     0x0000000000000000,0x0000000000000010
            DCQ     0x0000000000000008,0x0000000000000004
            DCQ     0x0000000000000002,0x0000000000000001
            DCQ     0x0000001000000000,0x0000000000000007
            DCQ     0x0000000100000003,0x0000000100000002
            DCQ     0x0000000300000001,0x0000000200000001
            DCQ     0x0000000100000001,0x0000000000000001
            DCQ     0x0000000700000000,0x0000000000000001
            DCQ     0x8000000000000000,0x0000FFFF00001111
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
P SPACE 4
Q SPACE 4
Results	SPACE (2*MAX_DATA*4)	
;>>>>>   end variables here <<<<<
            END