            TTL Program Title for Listing Header Goes Here
          
;****************************************************************
;Descriptive comment header goes here.
;THIS PROGRAM DEALS WITH THE APPLICATION OF THE CIRCULAR FIFO QUEUE USING 
;serial drivers INSTEAD OF POLLING.
;Name:  HRISHIKESH MOHOLKAR
;Date:  10/31/2016
;Class:  CMPE-250
;Section:  <01L1, Tuesday, 2:00-4:00 pm>
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




;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;12:UART0 IRQ mask
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;12:UART0 IRQ pending status
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;12:UART0 IRQ mask
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  0x1F
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  0xC0
;---------------------------------------------------------------

; Qrecord contents

IN_PTR		EQU	0
OUT_PTR		EQU	4
BUF_STRT	EQU	8
BUF_PAST	EQU	12
BUF_SIZE	EQU	16
NUM_ENQD	EQU	17

; queue:0x1fffe100

Q_BUF_SZ	EQU	4
Q_REC_SZ	EQU	18
Q_TR_SZ     EQU 80

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
           
            ;  initializing the interrupt request
			BL Init_UART0_IRQ
            
            ; initialize the queue
            LDR R0,=QBuffer
			LDR R1,=QRecord
			MOVS R2,#Q_BUF_SZ
			BL InitQueue


STEP3		;load the prompt

            LDR R0,=SHOW

			BL PutStringSB
			
			LDR R0,=QBuffer     ; queue
			MOVS R5,R0    ; has the address of QBuffer
			LDR R1,=QRecord
            

GETCHARACTER	BL GETCHAR
				CMP R0,#'Z'        ;checks for lowercase and converts to uppercase
				BHI UPPERCASE
				B   SWITCH
				

UPPERCASE   
			SUBS R0,R0,#0x20    ; converts to uppercase
            B   SWITCH

	
SWITCH	    CMP R0,#'D'      ;compares if dequeue command
			BEQ OUTPUT1
			B switch1
			
OUTPUT1 	BL PutChar         ; dequeue command instructions
			MOVS R0,#0x0D      ; carriage return character
			BL PutChar
			MOVS R0,#0x0A       ; newline character
			BL PutChar
			
			LDR R1,=QRecord        ;loading address of qrecord in R1
			BL Dequeue       
		
			BEQ FAILUREprint
			
			BL PutChar
			B STEP8
			
			
FAILUREprint	LDR R0,=FAILURE    ; failure prompt if queue empty
				BL PutStringSB
				B STEP8
				
switch1     
			CMP R0,#'E' ; compares if enqueue commmand
			BEQ OUTPUT2
			B SWITCH2

OUTPUT2		BL PutChar   ;enqueue instruction
			MOVS R3,R0     ; save address of QBuffer
			MOVS R0,#0x0D    ;carriage return 
			BL PutChar
			MOVS R0,#0x0A    ; new line 
			BL PutChar		
			LDR R0,=ENTERCHARACTER    ;character enter command
			BL PutStringSB       
			
			MOVS R0,R3   ; address of qbuffer in r0
			BL GETCHAR
			
			;MOVS R0,R3
            
			BL PutChar  
			LDR R1,=QRecord    
	
			BL Enqueue   ; enqueue operation
 			BCC SUCCESS
			BCS failure

SUCCESS		MOVS R0,#0x0D
			BL PutChar
			MOVS R0,#0x0A
			BL PutChar
			LDR R0,=SUCCESSMESSAGE   ;successful enqueue
			BL PutStringSB
			MOVS R0,R3
			B STEP8
			
failure		MOVS R0,#0x0D
			BL PutChar
			MOVS R0,#0x0A
			BL PutChar
			
			LDR R0,=FAILURE		;enqueue failed full queue
			BL PutStringSB
			MOVS R0,R3
			B	STEP8
			
			
SWITCH2		CMP R0,#'H'            ; help command
			BNE SWITCH3
			
			BL PutChar
			MOVS R0,#0x0D
			BL PutChar
			MOVS R0,#0x0A
			BL PutChar
			LDR R0,=HELPMESSAGE  ; display all instructions
			BL PutStringSB
			
			MOVS R0,#0x0D
			BL PutChar
			MOVS R0,#0x0A
			BL PutChar
			
			B STEP3
			
			
SWITCH3		CMP R0,#'P'     ; display queue contents
			BNE SWITCH4
			BL PutChar
			MOVS R0,#0x0D
			BL PutChar
			MOVS R0,#0x0A
			BL PutChar

			BL PRINTCHARACTER

			B	STEP3
			
			
SWITCH4		CMP R0,#'S'       ; status of queue
			BNE GETCHARACTER      
			BL PutChar
            MOVS R0,#0x0D
			BL PutChar
			MOVS R0,#0x0A
			BL PutChar
			LDR R0,=STATUS      ; print status prompt
            
			BL PutStringSB
			
			
			
			
STEP8		LDR R0,=INHEX       ; displaing the content in hex of queue contents   
			
			BL PutStringSB
			
			LDR R0,=QRecord       
			
			LDR R0,[R0,#IN_PTR]    ; loads address of queue pointed by IN_PTR
			
			BL PutNumHex           ; subroutine for displaying contents address
			
			LDR R0,=OUTHEX  ;prompt
			BL PutStringSB
			
			LDR R0,=QRecord
			LDR R0,[R0,#OUT_PTR]
			
			BL PutNumHex
			
			
			LDR R0,=NUMDEC     ; queue size print prompt
			BL PutStringSB
			
			LDR R0,=QRecord
			LDRB R0,[R0,#NUM_ENQD]
			
			BL PutNumUB
			
			
			MOVS R0,#0x0D
			BL PutChar
			MOVS R0,#0x0A
			BL PutChar
			B STEP3
			
			



;>>>>>   end main program code <<<<<
;Stay here
            B       .
			LTORG
;>>>>> begin subroutine code <<<<<

;----------------------------------
; hardcoded subroutine for printing queue contents on the screen.
;----------------------------------

PRINTCHARACTER		PUSH{R0-R7,LR}
			

					MOVS R0,#'>'    
					BL PutChar
					LDR R2,=QRecord   ; Address of QRecord in R2
					LDR R0,=QBuffer   ; Queue address in R0
					LDRB R5,[R1,#NUM_ENQD]     ; R5 contains queue size based on elements enqueued
					
					
					
					LDR R4,[R2,#OUT_PTR]           ; R4 has the address of content of queue pointed by OUT_PTR
                    LDR  R6,[R2,#BUF_STRT]          ; R6 has the address of BUF_START
                    LDR R7,[R2,#BUF_PAST]       ; R7 has the address of BUF_PAST
					
PRINTNEXT			CMP R5,#0
					
					BEQ BRANCHTOFIRST   ; SIZE ZERO THEN END
					
					LDRB R1,[R4,#0]    ; Loading the contents at the address of queue pointed by out_ptr
					PUSH{R0}
					MOVS R0,R1 
					BL PutChar       ;print content
					POP{R0}
					ADDS R4,R4,#1 ; increment outpointer
                    CMP R4,R7      ; compare address with buf_past
                    BGE SET       
                    B next
                    
SET                MOVS R4,R6  ; stores in R4 the address of buf_start
                   
next				SUBS R5,R5,#1  ; reduce queue size to go to next character
					
					B PRINTNEXT



BRANCHTOFIRST		MOVS R0,#'<'
					BL PutChar
					MOVS R0,#0x0D
					BL PutChar
					MOVS R0,#0x0A
					BL PutChar
					POP{R0-R7,PC}




; initialize the board with pc

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

;------------------------------
;PutChar subroutine for displaying single character on screen
;-------------------------------

;Poll TDRE until UART0 ready to transmit
PutChar	            PUSH{R1-R5,LR}
					LDR R1,=TxQRecord   ;loading address of transfer queue record
Puttoscreen 		CPSID I             ; masking all interrupt
					
          	        BL Enqueue          ; enqueue the character present in the location into txqrecord for display on the screen   
					CPSIE I             ; unmasking other interrupts 
					BCS Puttoscreen     
					
					LDR R3,=UART0_C2           ; enable transfer interrupt 
					MOVS R4,#UART0_C2_TI_RI
					STRB R4,[R3,#0]   ; character stored in the TxQueue of the TxQRecord
					
					POP{R1-R5,PC}    
					
					
					
 
 ;-----------------------------------
 ;GETCHAR SUBROUTINE ACCEPTS CHARACTER TYPED
 ;-----------------------------------
 
 ;Poll RDRF until UART0 ready to receive
GETCHAR		        PUSH{R1-R5,LR}
 					LDR R1,=RxQRecord    ;loading address of receive queue record
dequeuethechar		CPSID I             ;masking all interrupt
					BL Dequeue             ; dequeue the character present in the location into rxqrecord for storing it into the memory
					CPSIE I                ; unmasking other interrupts 
					
					BCS dequeuethechar     
                    
					
;Receive character and store in Ri
					
					POP{R1-R5,PC}
;--------------------------
;Input: R1 : 
;Output:
; This interrupt has two main functions of transferring character stored
;in the transfer queue to the screen when transfer interrupt is enabled and 
;accepting the character from the queuebuffer then storing into the rxqueue 
;then dequeueing the rxqueue to store the character into the memory.
;--------------------------
					

UART0_ISR			CPSID I    ; mask interrupts
					PUSH{R4,LR}
					
					LDR R2,=UART0_C2   ; loading control register
					
					LDRB R3,[R2,#0]
					
					MOVS R4,#UART_C2_TIE_MASK    
					
					ANDS R3,R3,R4     ; check if bit 7 of control register set 
					
					BNE  TRANSFER_ENABLE
					
					B VERIFY_RECEIVE_INTERRUPT
					
TRANSFER_ENABLE		LDR R2,=UART0_S1     ; check for TDRE in UART0_S1 register 
					LDRB R3,[R2,#0]              
					MOVS R4,#UART_S1_TDRE_MASK
					
					ANDS R3,R3,R4
				
					BEQ VERIFY_RECEIVE_INTERRUPT   ; if tdre not set then it is receive interrupt
					
					
					
					LDR R1,=TxQRecord     ;load the address of txqueue
					MOVS R2,#Q_TR_SZ      ; size of the queue storing data whose address is in TxQueuerecord
					
					BL Dequeue    ; dequeue character
					
					BCS UNSUCCESS_DEQUEUE
					
					LDR R1,=UART0_D   ; load the address of the data on the UART0 data section of the register
					
					STRB R0,[R1,#0] ; store in the uart0 which will be further displayed on the screen
					
					
					

					
					
VERIFY_RECEIVE_INTERRUPT	LDR R0,=UART0_S1 ; receive interrupt then enqueue the character to the receive queue and end
							
							LDRB R2,[R0,#0]
							
							MOVS R3,#UART_S1_RDRF_MASK
					 
					        ANDS R2,R2,R3
							

							
							BEQ ISR_COMPLETE         
							
						
							LDR R0,=UART0_D    ; load the data into r4 
							LDRB R4,[R0,#0]
							
							LDR R1,=RxQRecord        
							MOVS R0,R4    ; pass the character to the queue
							
							BL Enqueue
							
							
							B ISR_COMPLETE

UNSUCCESS_DEQUEUE			MOVS R2,#UART0_C2_T_RI    ; disable interrupt if no character in transfer queue
							LDR R3,=UART0_C2
							STRB R2,[R3,#0]
							


ISR_COMPLETE				CPSIE I
							POP{R4,PC}

;----------------------------
; this subroutine initializes the 
;Init_UART0_IRQ ;basically the txqueue and rxqueue
;----------------------------
							
Init_UART0_IRQ			PUSH{R0-R5,LR}
						LDR R1,=RxQRecord     ;initialize receive queue
						LDR R0,=RxQueue
							 
						MOVS R2,#Q_TR_SZ
							 
						BL InitQueue
							 
					    LDR R1,=TxQRecord  ; initialize transfer queue
						LDR R0,=TxQueue
							 
						MOVS R2,#Q_TR_SZ
						BL InitQueue
							 
						
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
						 MOVS R1,#UART0_C2_T_RI
						 STRB R1,[R0,#UART0_C2_OFFSET] 
						
						 
						 
						 ;Set UART0 IRQ priority
						 LDR R0,=UART0_IPR
						 ;LDR R1,=NVIC_IPR_UART0_MASK
						 LDR R2,=NVIC_IPR_UART0_PRI_3
						 LDR R3,[R0,#0]
						 ;BICS R3,R3,R1
						 ORRS R3,R3,R2
						 STR R3,[R0,#0]
						;Clear any pending UART0 interrupts
						 LDR R0,=NVIC_ICPR
						 LDR R1,=NVIC_ICPR_UART0_MASK
						 STR R1,[R0,#0]
						;Unmask UART0 interrupts
						 LDR R0,=NVIC_ISER 
						 
						 LDR R1,=NVIC_ISER_UART0_MASK
						 STR R1,[R0,#0] 
						 
						 POP{R0-R5,PC}
						 
						 
								
							
									 
							 
							
					
					
				
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

;--------------------------------
;INPUT : R0 (DIVISOR), R1 (DIVIDEND)
;OUTPUT: R0 :QUOTIENT , R1  (REMAINDER)
;------------------------------------


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
					
		
;------------------
; INPUT : R0 (QBuffer) , R1: (QRecord)

; Output : queue initialized
;-------------------



InitQueue	PUSH{R2-R5}
			
			STR R0,[R1,#IN_PTR]
			STR R0,[R1,#OUT_PTR]
			STR R0,[R1,#BUF_STRT]
		
			
			ADDS R0,R0,R2
			STR R0,[R1,#BUF_PAST]
			STRB R2,[R1,#BUF_SIZE]
			MOVS R0,#0
			STRB R0,[R1,#NUM_ENQD]
            
			POP{R2-R5}
			BX LR


;--------------------
;Input: R1 : QRecord structure address, R0 : Character to enqueue
;Output : carry set or cleared based on the size of queue

; enqueues character to the queue at the address pointed by in_ptr
;--------------------

Enqueue		PUSH{R2-R5}
			LDRB R4,[R1,#NUM_ENQD]   ; NUMBER OF ELEMENTS
			LDRB R3,[R1,#BUF_SIZE]     ; max size 
			CMP R4,R3    ; compare with max size
			BLT ENQUEUEop   ; if queue not full enqueue
			B QUEUECOMPLTE    ; CARRY FLAG SET IF FULL

ENQUEUEop
			
			
     		LDR R2,[R1,#IN_PTR]   ; address in queue pointed by IN_PTR in R2
			STRB R0,[R2,#0]        ; storing character in the address location in R2
			
			
			
LABEL3		LDR R3,[R1,#IN_PTR]     ; LOADING ADDRESS IN QUEUE POINTED BY IN_PTR
			ADDS R3,R3,#1          ; INCREMENTING THE ADDRESS TO NEXT LOCATION
			STR R3,[R1,#IN_PTR]    ; STORING THE NEW ADDRESS OF QUEUE POINTED BY IN_PTR
			
		
			LDRB R4,[R1,#NUM_ENQD]  ; LOADING INTO R4 THE CONTENT IN R1 WHICH CONTAINS ADDRESS POINTED BY NUM_ENQD
			ADDS R4,R4,#1           ; INCREMENTING SIZE AFTER ENQUEUEING
			
			STRB R4,[R1,#NUM_ENQD]  ; STORING BACK INTO THE ADDRESS
			
			
       

			LDR R4,[R1,#IN_PTR]        ; LOAD ADDRESS POINTED  BY IN_PTR
            LDR R3,[R1,#BUF_PAST]   ;LOAD ADDRESS POINTED BY BY BUF_PAST
                
		    CMP R4,R3  ;CMP  BOTH
			
			BGE SETENQUEUE  ; CLEAR FLAG TO INDICATE SUCCESSFUL ENQUEUE
		
		    B ENDTHEQUEUE
                 

		
		
SETENQUEUE      LDR R3,[R1,#BUF_STRT]    ; CARRY FLAG C=0
				STR R3,[R1,#IN_PTR]
				

				MRS R3, APSR
                MOVS R2,#0x20
				LSLS R3,R3,#24
                BICS R3,R3,R2
				MSR APSR,R3
		
				B ENDTHEQUEUE
		

QUEUECOMPLTE	                ; CARRY FLAG C=1
				MRS R5,APSR
				MOVS R4,#0x20
				LSLS R4,R4,#24
				ORRS R5,R5,R4
				MSR APSR,R5
				B ENDTHEQUEUE

ENDTHEQUEUE     POP{R2-R5}
				BX LR		

;------------------------------
;Input : R1: queue record structure address
;Output: R0 : character dequeued 

;character dequeued from the queue
;-------------------------------



Dequeue		PUSH{R1-R4}
		
			LDRB R4,[R1,#NUM_ENQD]          ; R4 HAS THE VALUE AT THE ADDRESS POINTED BY NUM_ENQD
			CMP R4,#0         ; COMPARES THE VALUE WITH ZERO 
			BEQ FAIL     ; EMPTY QUEUE
					
			LDR R0,[R1,#OUT_PTR] ; R0 CONTAINS THE ADDRESS POINTED BY OUT_PTR
			LDRB R0,[R0,#0]          ; LOAD THE CONTENT AT THE ADDRESS R0
			
			LDRB R4,[R1,#NUM_ENQD]    ; R4 HAS THE VALUE AT THE ADDRESS POINTED BY NUM_ENQD
			SUBS R4,R4,#1             ; DECREMENT THE SIZE OF QUEUE
			STRB R4,[R1,#NUM_ENQD]    ;STORES THE SIZE NEW IN ADDRESS POINTED BY NUM_ENQD

			LDR R4,[R1,#OUT_PTR]      ; LOADING THE ADDRESS POINTEDBY OUT_PTR INTO R4
			ADDS R4,R4,#1      ; INCREMENT THE ADDRESS POINTED BY OUT_PTR
			STR R4,[R1,#OUT_PTR]        ;STORE BACK THE NEW ADDRESS INTO THE OUT_PTR ADDRESS

			LDR R3,[R1,#BUF_PAST]      ;LOADING THE ADDRESS POINTED BY BUF_PAST INTO R3
			CMP R4,R3                  ; COMPARING BOTH THE IN_PTR ADDRESS AND BUFFER_PAST
			BGE RESTART
		
			MRS R1,APSR               ; CARRY CLEAR IF DEQUEUED SUCCESSFULLY
			MOVS R4,#0x20
			LSLS R1,R1,#24
			BICS R1,R1,R4
			MSR APSR, R1
			B ENDTHEQUEUE1

		
FAIL		MRS R1,APSR               ; CARRY SET IF NOT DEQUEUED SUCCESSFULLY
	        MOVS R4,#0x20
			LSLS R4,R4,#24
			ORRS R1,R1,R4
			MSR APSR,R1
			B ENDTHEQUEUE1

RESTART		LDR R4,[R1,#BUF_STRT]         ; LOADING INTO R4 THE ADDRESS POINTED BY BUF_STRT
			STR R4,[R1,#OUT_PTR]           ; STORING THE ADDRESS IN R4 INTO THE OUT_PTR  RESETS THE POINTER TO START OF THE QUEUE
			
			MRS R1,APSR     ; CARRY CLEARED WHEN DEQUEUED LAST ELEMENT 
			MOVS R4,#0x20
			LSLS R1,R1,#24
			BICS R1,R1,R4
			MSR APSR,R1
			B ENDTHEQUEUE1

ENDTHEQUEUE1	POP{R1-R4}
				BX LR
                
                
;--------------------------------------------
; INPUT: R0 : UNSIGNED WORD VALUE IN RO 
; THIS SUBROUTINE PRINTS THE ADDRESS IN HEXADECIMAL REPRESENTATION ON THE SCREEN
;----------------------------------------------
		
		
PutNumHex	PUSH{R0,R1-R5,LR}
			
            MOVS R3,R0      ;SAVING THE UNSIGNED WORD VALUE IN R3 FROM R0
			MOVS R4,#0       ;COUNTER 
			
LOOP8_TIME	MOVS R0,R3        ; SAVING THE UNSIGNED WORD VALUE IN R0 FROM R3
			CMP R4,#7         ; COMPARES THE COUNTER TO 7  AS THE ADDRESS IS 8 BYTE 
			BHI ENDPUTNUMHEX  ; IF GREATER THEN END PRINTING THE ADDRESS 
	
            
			
			MOVS R2,#28     
			RORS R0,R0,R2   ; CIRCULAR MOVEMENT OF THE ADDRESS IN ORDER TO RETURN TO THE SAME STARTING LOCATION
			MOVS R3,R0      
		
			MOVS R5,#NIBBLE_MASK     
			ANDS R0,R0,R5
			ADDS R0,R0,#'0'     ; CONVERT TO APPROPRIATE ASCII
			CMP R0,#'9'
			
			BHI BRANCH1
			BL PutChar
			ADDS R4,R4,#1
			B LOOP8_TIME
			
BRANCH1		ADDS R0,R0,#7		
            BL PutChar
			
			ADDS R4,R4,#1
			
			;BL PutChar
		    B LOOP8_TIME
			
			
ENDPUTNUMHEX	
				POP{R0,R1-R5,PC}		





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
				   
LOOPten			   MOVS R0,#10     ; MOVING 10 IN R0 EVERY TIME LOOP OCCURS TO DIVIDE R1 BY 10

        		   BL DIVU
				   
				   PUSH{R1}
				   
				   MOVS R1,R0
				   
				   ADDS R3,R3,#1      
				   
				   CMP R0,#0
				   
				   BNE LOOPten
				  
				   
				   
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
				


PutNumUB    PUSH{R1,LR}
			MOVS R1,#0xFF
			
			ANDS R0,R0,R1
			
			BL PutNumU
			
			POP{R1,PC}
			
				   

					

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
            DCD    UART0_ISR          ;28:UART0 (status; error)
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
SHOW   DCB "Type a queue command ( D, E, H, P, S ):",0x00
Length DCB "Length:",0x00
FAILURE DCB "FAILURE:",0x00
SUCCESSMESSAGE DCB "SUCCESS:",0x00
HELPMESSAGE DCB "d (dequeue) , e (enqueue) , h (help) , p (print) , s(status)",0X00
ENTERCHARACTER DCB "Character to enqueue:",0x00
STATUS DCB "Status:",0x00

INHEX DCB " In=0x",0x00
OUTHEX DCB " Out=0x",0x00
NUMDEC DCB " Num=",0x00

;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
QBuffer		SPACE	Q_BUF_SZ
	ALIGN
		
QRecord		SPACE	Q_REC_SZ
	ALIGN
		
RxQueue    SPACE    Q_TR_SZ
	ALIGN

RxQRecord   SPACE   Q_REC_SZ
	ALIGN
		
TxQueue		SPACE   Q_TR_SZ
	ALIGN
		
TxQRecord   SPACE   Q_REC_SZ
	ALIGN

;>>>>>   end variables here <<<<<
            ALIGN
            END