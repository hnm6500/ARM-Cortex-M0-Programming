           TTL Program Title for Listing Header Goes Here
;****************************************************************
;The lab deals with displaying the input string by taking care of the
;buffer overflow . it also calculates and displays the length of the string.
;it follows a set of commands to perform the operation.
;Name:  Hrishikesh Moholkar
;Date:  10/4/2016
;Class:  CMPE-250
;Section:  <01L1, Tuesday, 2:00-4:00>
;---------------------------------------------------------------
;Keil Template for KL46
;R. W. Melton
;April 3, 2015
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;Port A
PORT_PCR_SET_PTA1_UART0_RX EQU (PORT_PCR_ISF_MASK :OR: \
 PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX EQU (PORT_PCR_ISF_MASK :OR: \
 PORT_PCR_MUX_SELECT_2_MASK) 
 
SIM_SOPT2_UART0SRC_MCGPLLCLK EQU \
 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
 (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)

SIM_SOPT5_UART0_EXTERN_MASK_CLEAR EQU \
 (SIM_SOPT5_UART0ODE_MASK :OR: \
 SIM_SOPT5_UART0RXSRC_MASK :OR: \
 SIM_SOPT5_UART0TXSRC_MASK)
 
UART0_BDH_9600 EQU 0x01
UART0_BDL_9600 EQU 0x38 
UART0_C1_8N1 EQU 0x00
UART0_C2_T_R EQU (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C3_NO_TXINV EQU 0x00 

UART0_C4_OSR_16 EQU 0x0F
UART0_C4_NO_MATCH_OSR_16 EQU UART0_C4_OSR_16 
	
UART0_C5_NO_DMA_SSR_SYNC EQU 0x00
UART0_S1_CLEAR_FLAGS EQU 0x1F
	
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS EQU 0xC0 
; MAXIMUM BUFFER SIZE	
MAX_STRING EQU 9
	
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
Reset_Handler
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL46 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<

;---------------------------------
; connecting circuit to correct ports of pc


				BL Init_UART0_Polling 
;-----------------------------------

; R0 stores the address of the variable 


STEP2			LDR R0,=SHOW      

 ; display the string stored at address R0
 
				BL PutStringSB       
        
; R1 has the value of 79
         
				MOVS R1,#MAX_STRING	
				
 ;STORING ADDRESS OF VARIABLE IN R5
 
				LDR R5,=STOREDATA
			
; ASK USER TO  INPUT CHARACTER

LOOP5		    BL GETCHAR  

; taking input from the user

; comparing wether the letter is lowercase based on ASCII value

				CMP R0,#'Z'  
				
 ; converts the letter to uppercase	
 ;
				BGT UPPERCASE   
				
; compares wether the command is correct

SWITCH		    CMP R0,#"G"    

; goes to check for other command

				BNE  SWITCH1    
				
; goes to the functionality of the other command	

				BEQ OUTPUT1			
				
; similar checking for other commands 	

SWITCH1			CMP R0,#"I"     

				BNE SWITCH2
				
				BEQ OUTPUT2
				
				
SWITCH2			CMP R0,#"L"

				BNE SWITCH3
				
				BEQ OUTPUT3
				
SWITCH3			CMP R0,#"P"

				BNE LOOP5
				
				BEQ OUTPUT4
				
				
				
; COMMANDS OF G command

OUTPUT1			BL PutChar
     
 ;displays carriage return character on screen	
 
				MOVS R0,#0x0D		
				
				BL PutChar
				
; displays line feed character on the screen

				MOVS R0,#0x0A	
				
				BL PutChar		
				
 ; R5 containing the address of ro is stored in ro 
 
				;MOVS R0,R5   
				
; prints specific character

				MOVS R0, #'<'  
				
				BL PutChar
				
;storing the address of R0 back in R1 from temporary register r5

			    MOVS R0,R5 
				
 ; calling the getstring subroutine to store input subroutine
 
                MOVS R1,#MAX_STRING
				
                LDR R0,=STOREDATA	
				
				BL GetStringSB 
				
 ; the address of R0 back in R1 from temporary register r5	
 
				MOVS R0,R5
				
;returning the  loop for accepting more string	

				B  STEP2                
 
 ; list of instructions for I command
 
OUTPUT2         
				BL PutChar
				
				MOVS R5,R0
				
				MOVS R0,#0x0D  
				
				BL PutChar  
				
				MOVS R0,#0x0A
				
				BL PutChar
				
                MOVS R0,R5		
				
				MOVS R5,#0x00
				

				LDR R0,=STOREDATA
				
			    STRB R5,[R0,#0]
			

				LDR R0,=SHOW
				
				MOVS R1,#MAX_STRING
				
				BL PutStringSB
				
				B SWITCH2
				
				
				
 ; list of instructions for L command	
 
OUTPUT3			BL LengthStringSB    

				
				MOVS R5,R0
				
				MOVS R0,#0x0D
				
				BL PutChar
				
				MOVS R0,#0x0A
				
				BL PutChar
				
				LDR R0,=STOREDATA
				
				BL LengthStringSB
		        
				MOVS R0,R2 

            ; moving r2 that is the length of the passed string in r1
			
				BL PutNumU   				
				
				MOVS R0,#0x0D
				
				BL PutChar
				
			
				MOVS R0,#0x0A
				
				BL PutChar
				
				MOVS R0,R5
				
				B  STEP2
				

; list of conditions for P command				
       
OUTPUT4			BL PutChar

				MOVS R5,R0
				
				MOVS R0,#0x0D
				
				BL PutChar
				
				MOVS R0,#0x0A
				
				BL PutChar
				
				MOVS R0, #'>'
				
				BL PutChar
				
	            LDR R0,=STOREDATA
				
				BL PutStringSB
				
				MOVS R0, #'>'
				
				BL PutChar
				
				MOVS R0,R5
				
				MOVS R0,#0x0D
				
				BL PutChar
				
				MOVS R0,#0x0A
				
				BL PutChar
				
				MOVS R0,R5
				
                B  STEP2


				
 
		
;converts to uppercase the lowercase letters

UPPERCASE       SUBS R0,R0,#0x20

                B   SWITCH



;>>>>>   end main program code <<<<<
;Stay here
            B       .
;>>>>> begin subroutine code <<<<<
;Select MCGPLLCLK / 2 as UART0 clock source

Init_UART0_Polling	PUSH{R0-R5}
					LDR R0,=SIM_SOPT2
					LDR R1,=SIM_SOPT2_UART0SRC_MASK
					LDR R2,[R0,#0]
					BICS R2,R2,R1
					LDR R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
					ORRS R2,R2,R1
					STR R2,[R0,#0]
;Enable external connection for UART0
					LDR R0,=SIM_SOPT5
					LDR R1,= SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
					LDR R2,[R0,#0]
					BICS R2,R2,R1
					STR R2,[R0,#0]
;Enable clock for UART0 module
					LDR R0,=SIM_SCGC4
					LDR R1,= SIM_SCGC4_UART0_MASK
					LDR R2,[R0,#0]
					ORRS R2,R2,R1
					STR R2,[R0,#0]
;Enable clock for Port A module
					LDR R0,=SIM_SCGC5
					LDR R1,= SIM_SCGC5_PORTA_MASK
					LDR R2,[R0,#0]
					ORRS R2,R2,R1
					STR R2,[R0,#0]
;Connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
					LDR R0,=PORTA_PCR1
					LDR R1,=PORT_PCR_SET_PTA1_UART0_RX
					STR R1,[R0,#0]
;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
					LDR R0,=PORTA_PCR2 
					LDR R1,=PORT_PCR_SET_PTA2_UART0_TX
					STR R1,[R0,#0]
 
 ;Disable UART0 receiver and transmitter
					 LDR R0,=UART0_BASE
					 MOVS R1,#UART0_C2_T_R
					 LDRB R2,[R0,#UART0_C2_OFFSET]
					 BICS R2,R2,R1
					 STRB R2,[R0,#UART0_C2_OFFSET]
;Set UART0 for 9600 baud, 8N1 protocol
					 MOVS R1,#UART0_BDH_9600
					 STRB R1,[R0,#UART0_BDH_OFFSET]
					 MOVS R1,#UART0_BDL_9600
					 STRB R1,[R0,#UART0_BDL_OFFSET]
					 MOVS R1,#UART0_C1_8N1
					 STRB R1,[R0,#UART0_C1_OFFSET]
					 MOVS R1,#UART0_C3_NO_TXINV
					 STRB R1,[R0,#UART0_C3_OFFSET]
					 MOVS R1,#UART0_C4_NO_MATCH_OSR_16
					 STRB R1,[R0,#UART0_C4_OFFSET]
					 MOVS R1,#UART0_C5_NO_DMA_SSR_SYNC
					 STRB R1,[R0,#UART0_C5_OFFSET]
					 MOVS R1,#UART0_S1_CLEAR_FLAGS
					 STRB R1,[R0,#UART0_S1_OFFSET]
					 MOVS R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
					 STRB R1,[R0,#UART0_S2_OFFSET]
					 
					 ;Enable UART0 receiver and transmitter
					 MOVS R1,#UART0_C2_T_R
					 STRB R1,[R0,#UART0_C2_OFFSET] 
					 POP{R0-R5}
					 BX LR


;Poll TDRE until UART0 ready to transmit
PutChar	            PUSH{R1-R5}
					LDR R1,=UART0_BASE
					MOVS R2,#UART0_S1_TDRE_MASK
PollTx 				LDRB R3,[R1,#UART0_S1_OFFSET]
					ANDS R3,R3,R2
					BEQ PollTx
;Transmit character stored in Ri
					STRB R0,[R1,#UART0_D_OFFSET] 
					POP{R1-R5}
					BX LR
 
 ;Poll RDRF until UART0 ready to receive
GETCHAR		        PUSH{R1-R5}
					LDR R1,=UART0_BASE
					MOVS R2,#UART0_S1_RDRF_MASK
PollRx 				LDRB R3,[R1,#UART0_S1_OFFSET]
					ANDS R3,R3,R2
					BEQ PollRx
;Receive character and store in Ri
					LDRB R0,[R1,#UART0_D_OFFSET] 
					POP{R1-R5}
					BX LR
					
					;R0 : LOCATION OF STRING IN MEMORY		
					
;GetStringSB         ;PUSH{R0,R2,R3,R4,R5,LR}
					
					;MOVS R4,#0   ; OFFSET
					;MOVS R3,R0     ; CHARACTER IN R0 STORED IN R3
					
;LOOP                BL GETCHAR
					;CMP R3,#0x0D
					;BEQ ENDLOOP
					
					;BL PutChar
					;STRB R0,[R3,R4]
					;ADDS R4,R4,#1
					;CMP R4,R1
					;BEQ ENDLOOP
					;B LOOP
					
	
;ENDLOOP				MOVS R5,#0x00        ; NULL TERMINATTES THE STRING 
					;STRB R5,[R3,R4]
					;STRB R3,[R1,
					;POP{R0,R2,R3,R4,R5,PC}
;---------------------------------------------
;input : R1:MAX_STRING, R0:VARIABLE ADDRESS 
;output : R0: STRING STORED IN R0

;THIS SUBROUTINE TAKES INPUT FROM THE USER AND STORES
;VARIABLE MEMORY 
;---------------------------------------------

GetStringSB         PUSH{R0,R1,R2,R3,R4,R5,LR}


					MOVS R4,#0; POINTER
					
					MOVS R3,R0
					
					MOVS R5,R0
					SUBS R1,R1,#1
LOOP				BL GETCHAR
                    
					CMP R0,#0x0D
					BEQ ENDLOOP
					CMP R4,R1
					BGT LOOP2
					BL PutChar
					
					
					STRB R0,[R3,R4]
					ADDS R4,R4,#1
					
					CMP R4,R1
					BEQ LOOP2
					B LOOP
					
LOOP2               CMP R0,#0X0D
                    BEQ  ENDLOOP
					BL GETCHAR
					B LOOP2
			    
				  							

ENDLOOP				MOVS R0,#0x00    ; NULL TERMINATTES THE STRING 

					STRB R0,[R3,R4]
					MOVS R0,#0x0D
					BL PutChar
				    MOVS R0,#0x0A
				    BL PutChar
					MOVS R0,R5
					MOVS R0,R3
					
					
					POP{R0,R1,R2,R3,R4,R5,PC}
					
;------------------------------------------
;INPUT: R1:MAX_STRING, ;R0:VARIABLE ADDRESS
;OUTPUT: R2:LENGTH OF STRING
;THIS SUBROUTINE CALCUATES THE LENGTH OF STRING
;----------------------------------------

LengthStringSB      PUSH{R3,R4,R5,LR}
					
					
					MOVS R2,#0        ; LENGTH
					
					MOVS R5,#0        ;OFFSEt
					
					MOVS R3,R0
					
LOOP1               CMP R2,R1
					BGE ENDLOOP1
					
					LDRB R4,[R3,R5]   ;LOADING CHARACTER IN RO TO R4
					
					CMP R4,#0   ;NULLCHARACTER CHECK
					BEQ ENDLOOP1
					
					ADDS R5,R5,#1      ; INCREMEMT OFFSET TO GO TO OTHER LOCATION
					
					ADDS R2,R2,#1     ;INCREMENT LENGTH
					
					B LOOP1
					
									 
ENDLOOP1            POP{R3,R4,R5,PC}	

;--------------------------------------------
;input : R1:MAX_STRING, ;R0:VARIABLE ADDRESS
;output: R0:string in the memory address R0
; THIS SUBROUTINE DISPLAYS THE STORED STRING IN THE
;MEMORY ONTO THE SCREEN
;------------------------------------------
	
	
					
PutStringSB			PUSH{R0,R1,R3,R4,R5,LR}
					
					MOVS R4,#0       ; R4 IS THE OFFSET
					MOVS R5,R0       ; R5 HAS ADDRESS OF R0
					
				
					
LOOP4			    LDRB R3,[R5,R4]    ; LOADING THE CHARACTER IN R5 BYTE BY BYTE IN R3

				    CMP R3,#0x00       ; COMPARE TO SEE WETHER A NULL CHARACTER
					
					BEQ ENDLOOP3
			        
					CMP R4,R1
					
					BGE ENDLOOP3
					
					MOVS R0,R3
					
					BL PutChar
					
					ADDS R4,R4,#1	
					
					B LOOP4
					
ENDLOOP3           POP{R0,R1,R3,R4,R5,PC}					
					
		
;-------------------------------------
;INPUTS: R0 :THE NUMBER STORED IN R0, R1:MAX_STRING
;OUTPUT:  R0 : RETURNS  A NUMBER TO THE SCREEN 
;---------------------------------------------
					
PutNumU            PUSH{R1,R2,R3,R4,R5,LR}

				   MOVS R5,#0   
				   
                   ;MOVING CONTENTS OF RO IN R1
				   
				   MOVS R1,R0	
				   
				   MOVS R0,#10
				   
				   ;MOVING NUMBER 10 IN R0 ; R0 IS DIVISOR
				   
				   MOVS R3,#0  ; COUNTER
				   
		   
				   CMP R0,#0    ; COMPARE WETHER DIVISOR IS ZERO
				   
				   BEQ GOTOEND
				   
LOOP44			   MOVS R0,#10     ; MOVING 10 IN R0 EVERY TIME LOOP OCCURS TO DIVIDE R1 BY 10

        		   BL DIVU
				   
				   PUSH{R1}
				   
				   MOVS R1,R0
				   
				   ADDS R3,R3,#1      
				   
				   CMP R0,#0
				   
				   BNE LOOP44
				  
				   
				   
GOTOEND            POP {R1}   ; REMAINDER STORED IN R1

				   SUBS R3,R3,#1     ; KEEP TRACK TO PREVENT REMOVING NULL ELEMENT FROM STACK
				   
                   MOVS R0,R1
				   
				   ADDS R0,R0,#'0'   ; CONVERTS HEX TO DECIMAL
				   
				   BL PutChar
				   
                   CMP R3,#0
				   
				   BEQ ENDALL
				   
				 
				  
				  B  GOTOEND
				   
ENDALL              
				POP{R1,R2,R3,R4,R5,PC}	
				
				  
				   
				   
;SUBROUTINE PERFORMING DIVISION OPERATION				   
					
					
DIVU	    PUSH {R2-R5}        ; pushing registers to be used in subroutine for operations

			CMP R0,#0           ; COMPARE DIVISOR WITH ZERO
			
			BEQ CARRYSET
			
			MOVS R5,R0          ;STORING DIVISOR INTO R5
			MOVS R0,#0          ; QUOTIENT IS ZERO
			
While1      CMP  R1,R5
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
					
					
					
					
;>>>>>   end subroutine code <<<<<
            ALIGN

;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    Dummy_Handler      ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendableSrvReq (pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 xfer complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 xfer complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 xfer complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 xfer complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:command complete; read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:I2C1
            DCD    Dummy_Handler      ;26:SPI0 (all IRQ sources)
            DCD    Dummy_Handler      ;27:SPI1 (all IRQ sources)
            DCD    Dummy_Handler      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:UART1 (status; error)
            DCD    Dummy_Handler      ;30:UART2 (status; error)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:TPM2
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    Dummy_Handler      ;38:PIT (all IRQ sources)
            DCD    Dummy_Handler      ;39:I2S0
            DCD    Dummy_Handler      ;40:USB0
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:Segment LCD
            DCD    Dummy_Handler      ;46:PORTA pin detect
            DCD    Dummy_Handler      ;47:PORTC and PORTD pin detect
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
SHOW   DCB "Type a string command ( G, I, L,P):",0x00
Length DCB "Length:",0x00
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
STOREDATA SPACE MAX_STRING

;>>>>>   end variables here <<<<<
            ALIGN
            END