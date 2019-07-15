#include <iostm8s005k6.h>
#include <intrinsics.h>

#define TIM1_AutoReload_Value (1000-1)				// Giá tr? auto reload
#define TIM1_Prescaler_Value (unsigned int) 15		// S? chia t?n
#define Fmaster 	16000000                        // toc do chip
#define baudrate2          9600                         // toc do truyen UART
#define UART2_DIV          (unsigned int)(Fmaster/baudrate2)
#define UART2_DIV_32       (UART2_DIV >> 4)
#define UART2_DIV_41       (unsigned int)((UART2_DIV >>12) | (UART2_DIV & 0x0F))

#define         DHT_DATA_OUT    PD_ODR_ODR4
#define         DHT_DATA_IN     PD_IDR_IDR4

#define relay1     PC_ODR_ODR7
#define relay2     PC_ODR_ODR6
#define relay3     PC_ODR_ODR5     

#define IN_SENSOR1   PD_IDR_IDR3
#define IN_SENSOR2   PD_IDR_IDR2

#define         LCD_EN          PB_ODR_ODR0
#define         LCD_RW          PB_ODR_ODR1
#define         LCD_RS          PB_ODR_ODR2

#define         LCD_D4          PC_ODR_ODR1
#define         LCD_D5          PC_ODR_ODR2
#define         LCD_D6          PC_ODR_ODR3
#define         LCD_D7          PC_ODR_ODR4

/***************FUNCTION*******************/
void IO_Config();
void OSC_Config_Self();
void TIM1_Config_Self();
void UART2_Config();
void delay_us(int time);
void delay_ms(int time);
void LCD_Enable();
void LCD_Send4Bit(unsigned char Data);
void LCD_SendCommand(unsigned char command);
void LCD_Clear();
void LCD_Init();
void LCD_Gotoxy(unsigned char x, unsigned char y);
void LCD_PutChar(unsigned char Data);
void LCD_Puts(char *s);
unsigned char DHT_Getdata(unsigned char *Temp, unsigned char *Humi);
void UART2_SendByte(unsigned char Byte2Send);
void UART2_SendStr(unsigned char *Str);
unsigned char UART2_ReceiveData8(void);
unsigned char StringFind(char *StrSource,char *StrDest,unsigned char SourceLen,unsigned char DestLen);
unsigned char checkPhonenumber(char *StrSource,unsigned char *StrDest,unsigned char SourceLen,unsigned char DestLen);
unsigned char checkReceriver(char *StrSource,char *StrDest,char SourceLen,unsigned char DestLen,unsigned char startnumber);
char SIM_CheckOK(void);
void Send_SMS(unsigned char conTent[]);
void Send_DHT(unsigned char *conTent1,unsigned char *conTent2);
unsigned char ClearRx2Buff();
void test_SIM900A();
unsigned char changeStateOne();
unsigned char changeStateTwo();
void practicalFunction();

/******************************************/
unsigned char phoneNumber[]= "\"+84968830197\"";
unsigned char phoneNumber2[] = "\"+84353205105\"";

unsigned char state =0;
unsigned char stateSensor =0;
/**************************VARIABLE**************************/
  char Rx2Data = 0;
  char Rx2Index;
  char Rx2Buff[200];
  char strOK[] = "OK\r\n";
  char strSend[1] = ">";
  char strPhone_Msg[] = "\r\n+CMT:";
  unsigned char getNewphone[14];

/**************VARIABLE***OF***RX********************/
unsigned char Rx2_EndRead;
unsigned int Rx2TimeOut_mSec;
unsigned char Rx2ByteFind;


//BIEN TIMER
unsigned long u32mSec = 0, u32TimeSys_Sec = 0, timetemp =0;
unsigned int u32TimeSys_mSec;
unsigned char MALENH =0,trangthai=1;

char Second=0;

unsigned char Sts[159] ="X"; //ARRAY_CLEAR

/************************DHT11*************************************/
unsigned char nhietdo,doam;

/*************************GPIO_CONFIG******************************/
void IO_Config() // cau hinh chan vao ra
{
        /************DHT11****************/
	PD_DDR_DDR4=1;
	PD_CR1_C14=1;
        
        /************LCD_INIT*************/

        PC_DDR_DDR1=1; //LCD_D7
        PC_CR1_C11=1;
        PC_CR2_C21=1;
         
        PC_DDR_DDR2=1; //LCD_D6
        PC_CR1_C12=1;
        PC_CR2_C22=1;
         
        PC_DDR_DDR3=1; //LCD_D5
        PC_CR1_C13=1;
        PC_CR2_C23=1;
        
        PC_DDR_DDR4=1;  //LCD_D4
        PC_CR1_C14=1;
        PC_CR2_C24=1;
        
        PB_DDR_DDR0=1;  //LCD_EN
        PB_CR1_C10=1;
        PB_CR2_C20=1;
        
        PB_DDR_DDR1=1;  //LCD_RS
        PB_CR1_C11=1;
        PB_CR2_C21=1;
        
        PB_DDR_DDR2=1;  //LCD_RW
        PB_CR1_C12=1;
        PB_CR2_C22=1;
  
        /************END_LCD**************/    
        PD_DDR=0x20;
        PD_DDR_bit.DDR5=1;      // UART2 Tx -- Output
        PD_CR1=0x60;            // UART2 Tx,Rx Pull-up, push-pull
        PD_CR2 = 0x20;          // UART2 Tx -- enable ext interrupt
        
        PC_DDR_DDR7=1;  // out relay
        PC_CR1_C17=1;
        PC_CR2_C27=1;
        PC_ODR_ODR7=0;
         
        PC_DDR_DDR6=1;  // out relay
        PC_CR1_C16=1;
        PC_CR2_C26=1;
        PC_ODR_ODR6=0;
         
        PC_DDR_DDR5=1;  // out relay
        PC_CR1_C15=1;
        PC_CR2_C25=1;
        PC_ODR_ODR5=0;

        PD_DDR_DDR3=0; // in cambien
        PD_CR1_C13=0;
        
        PD_DDR_DDR2=0; // in cambien
        PD_CR1_C12=0;
        
}
/***************************CLK_HSI*******************************/
void OSC_Config_Self()                  //cau hinh toc do xung
{
    CLK_CKDIVR_HSIDIV = 0x00;       //16MHz
    
}
/***************************CONFIG_TIMER1*************************/
void TIM1_Config_Self()
{  
    TIM1_ARRH = ((unsigned int)TIM1_AutoReload_Value >> 8);
    TIM1_ARRL = (TIM1_AutoReload_Value & 0xFF);  
  
    TIM1_PSCRH = (TIM1_Prescaler_Value >> 8);
    TIM1_PSCRL = (TIM1_Prescaler_Value & 0xFF);    
    TIM1_CR1_CMS = 0x00;	//Vi tri d?m trong d?i d?m   00: edge   01: center1   10: center2   11: center3
 
    TIM1_CR1_DIR = 0;		//Up
    //TIM1_CR1_DIR = 1;		//Down
    TIM1_RCR = 0;   		//
    TIM1_CR1_ARPE = 1;  	//Enable Preload
    TIM1_IER_UIE = 1;   	//Cho phép ng?t update
    TIM1_CR1_CEN = 1;   	//TIM1 enable
}
/***************CONFIG_UART2*************************************/
void UART2_Config()  // cau hinh bat UART
{
    UART2_BRR2 = UART2_DIV_41;
    UART2_BRR1 = UART2_DIV_32;
    UART2_CR1_bit.M = 0;			// ÐO DÀI WORD LÀ 8 BITS
    UART2_CR2_bit.REN   = 1;        //Receiver enable
    UART2_CR2_bit.TEN   = 1;        //Transmit enable
    UART2_CR2_bit.RIEN  = 1;        //Receiver interrupt enable
    UART2_CR2_bit.TIEN  = 0;        //Transmit interrupt enable
    UART2_CR2_bit.TCIEN = 0;        //Transmission complete interrupt enable
}


/*************************************************************************/

void delay_us(int time){
	int value = time;
	while(value--){
		__asm("nop");
		__asm("nop");
		__asm("nop");
		__asm("nop");
		__asm("nop");
		__asm("nop");
		__asm("nop");
		__asm("nop");
		__asm("nop");
	}
}
void delay_ms(int time){
	int value = time;
	while(value--){
		delay_us(996);
	}
}
/*
void delay_ms(unsigned int mSec)        // ham delay
{
    unsigned int u16T1, u16T2;
    
    u16T1 = u32TimeSys_mSec;
    do
	{
        u16T2 = u32TimeSys_mSec;
    }
	while((u16T2 - u16T1) <= mSec);
}*/

/*****************SRT_LCD1602************************/

void LCD_Enable()
{
  LCD_EN = 1;
  delay_us(36);
  LCD_EN = 0;
  delay_us(400);
}

void LCD_Send4Bit(unsigned char Data)
{
        LCD_D4 = Data & 0x01;
        LCD_D5 = (Data>>1)&1;
        LCD_D6 = (Data>>2)&1;
        LCD_D7 = (Data>>3)&1;
}

void LCD_SendCommand(unsigned char command)
{
        LCD_Send4Bit(command >>4);/* Gui 4 bit cao */
        LCD_Enable();
        LCD_Send4Bit(command); /* Gui 4 bit thap*/
        LCD_Enable();
}

void LCD_Clear()
{                                       // Ham Xoa Man Hinh LCD
        LCD_SendCommand(0x01); 
        delay_us(92);
}

void LCD_Init(){
        LCD_Send4Bit(0x00);
        delay_ms(150);
        LCD_RS = 0;
        LCD_RW = 0;
        LCD_Send4Bit(0x03);
        LCD_Enable();
        delay_ms(40);
        LCD_Enable();
        delay_us(800);
        LCD_Enable();
        LCD_Send4Bit(0x02);
        LCD_Enable();
        LCD_SendCommand( 0x28 ); // giao thuc 4 bit, hien thi 2 hang, ki tu 5x8
        LCD_SendCommand( 0x0c); // cho phep hien thi man hinh
        LCD_SendCommand( 0x06 ); // tang ID, khong dich khung hinh
        LCD_SendCommand(0x01); // xoa toan bo khung hinh
}

void LCD_Gotoxy(unsigned char x, unsigned char y){
        unsigned char address;
        if(!y)address = (0x80+x);
        else address = (0xc0+x);
        delay_ms(8);
        LCD_SendCommand(address);
        delay_us(400);
}

void LCD_PutChar(unsigned char Data){//Ham Gui 1 Ki Tu
        LCD_RS = 1;
        LCD_SendCommand(Data);
        LCD_RS = 0 ;
}

void LCD_Puts(char *s){//Ham gui 1 chuoi ky tu
        while (*s){
                LCD_PutChar(*s);
                s++;

        }
}

/**************************DHT11*****************************************/
unsigned char DHT_Getdata(unsigned char *Temp, unsigned char *Humi)
{
  unsigned char Buffer[5]={0,0,0,0,0};
  unsigned char i, j, checksum;
  PD_DDR_DDR4 = 1;// Output
  DHT_DATA_OUT = 1;
  delay_us(60);
  DHT_DATA_OUT = 0;
  delay_ms(25);
  DHT_DATA_OUT = 1;
  PD_DDR_DDR4 = 0; // Input
  delay_us(60);
  if(DHT_DATA_IN) return 0; else while(!DHT_DATA_IN);
  delay_us(60);
  if(!DHT_DATA_IN) return 0; else while(DHT_DATA_IN);
  for(i=0;i<5;i++){
    for(j=0;j<8;j++){
      while(!DHT_DATA_IN);
      delay_us(50);
      if(DHT_DATA_IN){
        Buffer[i]|= ((1<<7-j));
        while(DHT_DATA_IN);
      }
    }
  }
  checksum = Buffer[0] + Buffer[1] + Buffer[2] + Buffer[3];//kiem tra da truyen du bit hay chua
  if(checksum != Buffer[4]) return 0;
  *Temp = Buffer[2];
  *Humi = Buffer[0];
  return 1;
}
//*************************Send_Function**********************************

void UART2_SendByte(unsigned char Byte2Send)  //ham truyen UART
{
    while(UART2_SR_bit.TXE == 0);
    UART2_DR = Byte2Send;
}

void UART2_SendStr(unsigned char *Str){ //Send 
    while(*Str != '\0'){
        UART2_SendByte(*Str++);
    }
}

//************************************************************************
unsigned char UART2_ReceiveData8(void){
    return((unsigned char)UART2_DR);
}
/****************************FIND_STRING**********************************/
unsigned char StringFind(char *StrSource,char *StrDest,unsigned char SourceLen,unsigned char DestLen){
    signed char i,j;
    j=0;
    for(i=0;i<SourceLen;i++){
        if(StrSource[i] == StrDest[j]){
                j++;
                if(j==DestLen){
                  return 1;
                }   
        }
        else j=0;
    }
    return 0;
}
unsigned char checkPhonenumber(char *StrSource,unsigned char *StrDest,unsigned char SourceLen,unsigned char DestLen){
    unsigned char i,j;
    j=0;
    for(i=0;i<SourceLen;i++){
        if(StrSource[i] == StrDest[j]){
            j++;
            if(j==DestLen){
                return i;
            }
        }
        else j=0;
    }
    return 0;
}

unsigned char checkReceriver(char *StrSource,char *StrDest,char SourceLen,unsigned char DestLen,unsigned char startnumber){
    unsigned char j=0;
    if(Rx2_EndRead){
        for(startnumber;startnumber<SourceLen;startnumber++){
            if(StrSource[startnumber]==StrDest[j]){
                j++;
                if(j==DestLen){
                    return 1;
                }
            }
            else j=0;
        }  
    }
    return 0; 
}

/*********************initialization_function_BASIC_SIM900A*****************/
/****************************CHECK_OK***************************************/
char SIM_CheckOK(void) 				// OK == 1
{
    int u32T1_mSec, u32T2_mSec;
    char u8Sts;
    signed char s8strcmp = -2;
    u8Sts = 0;
    u32T1_mSec = u32TimeSys_mSec;
    UART2_SendStr("AT\r\n");
    do
   {
        if(Rx2_EndRead){
            s8strcmp = StringFind(&Rx2Buff[0],&strOK[0], Rx2ByteFind, 4);
            if(s8strcmp >= 0){
                u8Sts = 1;
                return 1;
            }
        }
        u32T2_mSec = u32TimeSys_mSec;    
    } 
	while(!u8Sts && ((u32T2_mSec - u32T1_mSec) < 250));
    return 0;
}
/*****************************SEND_SMS*************************************/
void Send_SMS(unsigned char conTent[]){
    delay_ms(1000);
    int z=1000;
    do{ 
        z--;
        if(Rx2_EndRead){
            if(StringFind(&Rx2Buff[0],">",100,1) >= 1){
                UART2_SendStr(&conTent[0]);
                UART2_SendByte(0x1A);
                delay_ms(100);
                break;
            }
        }
    }
    while(z != 0);
}
void Send_DHT(unsigned char *conTent1,unsigned char *conTent2){
    unsigned char temp,humi;
    temp = (*conTent1/10+48);
    humi = (*conTent2/10+48);
    int z=1000;
    do{ 
        z--;
        if(Rx2_EndRead){
            if(StringFind(&Rx2Buff[0],">",100,1) >= 1){
                UART2_SendStr("Temp:  ");
                UART2_SendStr(&temp);
                UART2_SendStr("\nHumi:  ");
                UART2_SendStr(&humi);          
                UART2_SendByte(0x1A);
                delay_ms(100);
                break;
            }
        }
    }
    while(z != 0);
}

/***************************CLEAR_RX********************************/
unsigned char ClearRx2Buff()
{
    char i;
      for(i=0; i<200;i++)
          {
              Rx2Buff[i]='\0';
          } 
          return 1;
}
/***************************FUNCTION_USED**************************/
void test_SIM900A(){
    Rx2_EndRead=0;              //HOM QUA DANG CHECK O DAY
    unsigned char stateSIM=0;
    UART2_SendStr("AT\r\n");
    do{
        stateSIM=SIM_CheckOK();
        if(stateSIM==1){
        	LCD_Clear();
            LCD_Gotoxy(0,0);
            LCD_Puts("...Wait Start");
            delay_ms(2000);
            break;     
        } 
        delay_ms(100);
    }
    while(!stateSIM);
    /************************/
    UART2_SendStr("AT+IPR=9600\r\n"); // SPEED BAUDRATE 9600
    delay_ms(300);
    UART2_SendStr("ATH\r\n"); // TAT MAY KHI CO CUOC GOI DEN
    delay_ms(300);
    UART2_SendStr("ATH\r\n");
    delay_ms(300);
    UART2_SendStr("AT+CMGF=1\r\n"); // SMS TYPE TEXT
    delay_ms(300);
    
    UART2_SendStr("AT+CNMI=2,2\r\n"); // HIEN THI TRUC TIEP NOI DUNG TIN NHAN KHONG LUU VAO BO NHO
    delay_ms(300);
        
    UART2_SendStr("ATE0\r\n");      //Echo OFF
    delay_ms(300);
    UART2_SendStr("AT+CMGDA=\"DEL ALL\"\r\n"); // XOA TOAN BO MESSAGE
    delay_ms(2000);

    UART2_SendStr("AT+CMGS=\"0968830197\"\r\n"); 
    Send_SMS("ON1->3: ON TB1-3.\nONALL: ALL DEVICE ON.\nOFF1->3:OFF TB1-3.\nOFFALL:ALL DEVICE OFF.\nDHT11: Temp,Humi.");
    while(!ClearRx2Buff()); 
    delay_ms(5000);
    
    UART2_SendStr("AT+CMGS=\"0353205105\"\r\n"); 
    Send_SMS("ON1->3: ON TB1-3.\nONALL: ALL DEVICE ON.\nOFF1->3:OFF TB1-3.\nOFFALL:ALL DEVICE OFF.\nDHT11: Temp,Humi.");
    while(!ClearRx2Buff()); 
    delay_ms(2000);
    /**********LCD**********/
    LCD_Clear();
    LCD_Gotoxy(0,0);
    LCD_Puts("Start Successful");
}
unsigned char changeStateSP1(){
    unsigned char z=0;
    if(Rx2_EndRead){
         if(checkPhonenumber(&Rx2Buff[0],&phoneNumber[0],50,14)>0){ 
               z = checkPhonenumber(&Rx2Buff[0],&phoneNumber[0],50,14);
         }
         
         if(checkReceriver(&Rx2Buff[0],"OFFCB\r\n",100,7,z) == 1){ 
              stateSensor =0;
              relay1 = relay2= relay3= 0;
              LCD_Clear();
              LCD_Gotoxy(2,0);
              LCD_Puts("OFF SECURITY");
              
              UART2_SendStr("AT+CMGS="); 
              UART2_SendStr(&phoneNumber[0]);
              UART2_SendStr("\r\n");
              Send_SMS("OFF SECURITY");
              delay_ms(2000);
              while(!ClearRx2Buff()); 
              return 1;
         }
    }
    else 
      return 0;
}

unsigned char changeStateSP2(){
    unsigned char z=0;
    if(Rx2_EndRead){
         if(checkPhonenumber(&Rx2Buff[0],&phoneNumber2[0],50,14)>0){ 
               z = checkPhonenumber(&Rx2Buff[0],&phoneNumber2[0],50,14);
         }
         
         if(checkReceriver(&Rx2Buff[0],"OFFCB\r\n",100,7,z) == 1){ 
              stateSensor =0;
              relay1 = relay2= relay3= 0;
              LCD_Clear();
              LCD_Gotoxy(2,0);
              LCD_Puts("OFF SECURITY");

              UART2_SendStr("AT+CMGS="); 
              UART2_SendStr(&phoneNumber2[0]);
              UART2_SendStr("\r\n");
              Send_SMS("OFF SECURITY");
              delay_ms(2000);
              while(!ClearRx2Buff()); 
              return 1;
         }
    }
    else return 0;
}

void checkSensor(){
  int temp =0;
  while(stateSensor){
    
    temp++;
    if(temp==20000) temp=0;
    
    if(IN_SENSOR1==0) 
    {
      relay3=relay1=0;
    }
    if(IN_SENSOR2==0){
      relay3=relay2=0;
    }
    
    if(temp ==10000){
      if(IN_SENSOR1==1){
        relay3=relay1=1;
        
        UART2_SendStr("AT+CMGS=\"0968830197\"\r\n"); 
        Send_SMS("CB1 CO TROM");
        delay_ms(5000);
        
        UART2_SendStr("AT+CMGS=\"0353205105\"\r\n"); 
        Send_SMS("CB1 CO TROM");
        delay_ms(5000);
        
      }
      
      if(IN_SENSOR2==1){
        relay3=relay2=1;
        
        UART2_SendStr("AT+CMGS=\"0968830197\"\r\n"); 
        Send_SMS("CB2 CO TROM");
        delay_ms(5000);
        
        UART2_SendStr("AT+CMGS=\"0353205105\"\r\n"); 
        Send_SMS("CB2 CO TROM");
        delay_ms(5000);
      }
    }
    
    if(changeStateSP1()==1); //break;
    if(changeStateSP2()==1); //break;
  }
  
}

unsigned char changeStateOne(){
    unsigned char z=0;
    if(Rx2_EndRead){
        if(checkPhonenumber(&Rx2Buff[0],&phoneNumber[0],50,14)>0){ 
              z = checkPhonenumber(&Rx2Buff[0],&phoneNumber[0],50,14);
        }
        else
        {
              return 0;
        }
        
       if(checkReceriver(&Rx2Buff[0],"ON1\r\n",100,5,z) == 1){
            state = 1;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB1 ON");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       else if(checkReceriver(&Rx2Buff[0],"OFF1\r\n",100,6,z) == 1){
            state = 2;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB1 OFF");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       
       else if(checkReceriver(&Rx2Buff[0],"ON2\r\n",100,5,z) == 1){
            state = 3;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB2 ON");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       else if(checkReceriver(&Rx2Buff[0],"OFF2\r\n",100,6,z) == 1){
            state = 4;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB2 OFF");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       
       else if(checkReceriver(&Rx2Buff[0],"ON3\r\n",100,5,z) == 1){
            state = 5;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB3 ON");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       else if(checkReceriver(&Rx2Buff[0],"OFF3\r\n",100,6,z) == 1){
            state = 6;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB3 OFF");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       else if(checkReceriver(&Rx2Buff[0],"ONALL\r\n",100,7,z) == 1){
            state = 7; 
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n");
            Send_SMS("ALL ON");
            delay_ms(2000);
            while(!ClearRx2Buff()); 
            return 1; 
       }
       else if(checkReceriver(&Rx2Buff[0],"\nOFFALL\r",100,8,z) == 1){ 
            state = 8;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n");
            Send_SMS("ALL OFF");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1;  
       }
       else if(checkReceriver(&Rx2Buff[0],"ADDPHONE\r\n",100,10,z) == 1){
            state = 9;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n");   
            Send_SMS("ADD PHONE");
            delay_ms(2000);
            while(!ClearRx2Buff()); 
            return 1;  
       }
       else if(checkReceriver(&Rx2Buff[0],"DHT11\r\n",100,7,z) == 1){ 
            state = 10;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n");
            Send_SMS("DHT11 OK");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1;  
       }
       else if(checkReceriver(&Rx2Buff[0],"ONCB\r\n",100,6,z) == 1){ 
            state=11;
            stateSensor =1;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n");
            Send_SMS("ON SECURITY");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1;  
       }
       else{
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber[0]);
            UART2_SendStr("\r\n"); 
            Send_SMS("NOT COMMAND!!!");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 0;  
       }
    }
    else
     return 0;
}

unsigned char changeStateTwo(){
    unsigned char z=0;
    if(Rx2_EndRead){
        if(checkPhonenumber(&Rx2Buff[0],&phoneNumber2[0],50,14)>0){ 
              z = checkPhonenumber(&Rx2Buff[0],&phoneNumber2[0],50,14);
        }
        else
        {
              return 0;
        }
        
       if(checkReceriver(&Rx2Buff[0],"ON1\r\n",100,5,z) == 1){
            state = 1;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB1 ON");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       else if(checkReceriver(&Rx2Buff[0],"OFF1\r\n",100,6,z) == 1){
            state = 2;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB1 OFF");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       
       else if(checkReceriver(&Rx2Buff[0],"ON2\r\n",100,5,z) == 1){
            state = 3;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB2 ON");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       else if(checkReceriver(&Rx2Buff[0],"OFF2\r\n",100,6,z) == 1){
            state = 4;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB2 OFF");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       
       else if(checkReceriver(&Rx2Buff[0],"ON3\r\n",100,5,z) == 1){
            state = 5;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB3 ON");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       else if(checkReceriver(&Rx2Buff[0],"OFF3\r\n",100,6,z) == 1){
            state = 6;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n");
            Send_SMS("TB3 OFF");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1; 
       }
       else if(checkReceriver(&Rx2Buff[0],"ONALL\r\n",100,7,z) == 1){
            state = 7; 
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n");
            Send_SMS("ALL ON");
            delay_ms(2000);
            while(!ClearRx2Buff()); 
            return 1; 
       }
       else if(checkReceriver(&Rx2Buff[0],"\nOFFALL\r",100,8,z) == 1){ 
            state = 8;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n");
            Send_SMS("ALL OFF");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1;  
       }
       else if(checkReceriver(&Rx2Buff[0],"ADDPHONE\r\n",100,10,z) == 1){
            state = 9;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n");   
            Send_SMS("ADD PHONE");
            delay_ms(2000);
            while(!ClearRx2Buff()); 
            return 1;  
       }
       else if(checkReceriver(&Rx2Buff[0],"DHT11\r\n",100,7,z) == 1){
            state = 10;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n");
            Send_SMS("DHT11 OK");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1;  
       }
       else if(checkReceriver(&Rx2Buff[0],"ONCB\r\n",100,6,z) == 1){ 
            state=11;
            stateSensor =1;
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n");
            Send_SMS("ON SECURITY");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 1;  
       }
       else{
            UART2_SendStr("AT+CMGS="); 
            UART2_SendStr(&phoneNumber2[0]);
            UART2_SendStr("\r\n"); 
            Send_SMS("NOT COMMAND!!!");
            delay_ms(2000);
            while(!ClearRx2Buff());
            return 0;  
       }
    }
    else
     return 0;
}

void practicalFunction(){
    switch(state){
        case 1:{
            state=0;
            relay1 =1;
            LCD_Clear();
            LCD_Gotoxy(0,0);
            LCD_Puts("TB1 TURN ON");
            break; 
        }  
        case 2:{
            state=0;
            relay1 =0;
            LCD_Clear();
            LCD_Gotoxy(0,0);
            LCD_Puts("TB1 TURN OFF");
            break;
        }         
        case 3:{
            state=0;
            relay2 =1;
            LCD_Clear();
            LCD_Gotoxy(0,0);
            LCD_Puts("TB2 TURN ON");
            break;
        } 
        case 4:{
            state=0;
            relay2 =0;
            LCD_Clear();
            LCD_Gotoxy(0,0);
            LCD_Puts("TB2 TURN OFF");
            break;
        }
        case 5:{
            state=0;
            relay3 =1;
            LCD_Clear();
            LCD_Gotoxy(0,0);
            LCD_Puts("TB3 TURN ON");
            break;
        }
        case 6:{
            state=0;
            relay3 =0;
            LCD_Clear();
            LCD_Gotoxy(0,0);
            LCD_Puts("TB3 TURN OFF");
            break;
        }
        case 7:{
            state=0;
            relay1= relay2 =relay3 =1;
            LCD_Clear();
            LCD_Gotoxy(0,0);
            LCD_Puts("ALL THE DEVICE");
            LCD_Gotoxy(0,1);
            LCD_Puts("TURN ON");
            break;
        }
        case 8:{
            state=0;
            relay1= relay2 =relay3 =0;
            LCD_Clear();
            LCD_Gotoxy(0,0);
            LCD_Puts("ALL THE DEVICE");
            LCD_Gotoxy(0,1);
            LCD_Puts("TURN OFF");
            break;
        }  
        case 9:{
            state=0;
            LCD_Clear();
            LCD_Gotoxy(0,0);
            LCD_Puts("GET THE NEW PHONE NUMBER");
            break;
        }
        case 11:{
            state=0;
            LCD_Clear();
            LCD_Gotoxy(2,0);
            LCD_Puts("ON SECURITY");
            break;
        }
    }
}


/**************MAIN***************/
void main(void){
    OSC_Config_Self();
    IO_Config();
    TIM1_Config_Self();
    UART2_Config();

    __enable_interrupt();
    
    /************LCD*************/
    LCD_Init();
    LCD_Clear();
    LCD_Gotoxy(0,0);
    LCD_Puts("PHAN TRONG TIEN");
    LCD_Gotoxy(3,1);
    LCD_Puts("17/10/1997");
    delay_ms(2000);
    //***************************

    //kiemtrataikhoan
    test_SIM900A();
    while(!(ClearRx2Buff()));
    relay1=relay2=relay3=0;
    while(1){
      
          if(changeStateOne()==1){
             practicalFunction();
          }
          
          if(changeStateTwo()==1){
             practicalFunction();
          }
          
          if(state==10){
            if(DHT_Getdata(&nhietdo,&doam)){
            LCD_Gotoxy(0,0);
            LCD_Puts("Nhiet Do : ");
            LCD_PutChar(nhietdo/10+48);
            LCD_PutChar(nhietdo%10+48);
            LCD_Puts(" ");
            LCD_PutChar(223);//kí hieu nhiet do
            LCD_Puts("C");
            LCD_Gotoxy(0,1);
            LCD_Puts("Do Am    : ");
            LCD_PutChar(doam/10+48);
            LCD_PutChar(doam%10+48);
            LCD_Puts(" ");
            LCD_PutChar(37);
            delay_ms(150);
            }
          }
          

      
          //checkSensor(); //wait
     }
}
#pragma vector = 0x17 
__interrupt void UART2_Rx_Isr(void){
    if(UART2_SR_bit.RXNE == 1)
        {
                Rx2Data = UART2_ReceiveData8();
                if(Rx2TimeOut_mSec == 0)
                {
                    Rx2Index = 0;           //Timeout hoac bit dau tiên trong chuoi
                    
                }
            
                if(Rx2Index < 200)
                {
                    Rx2Buff[Rx2Index++] = Rx2Data;
                    Rx2_EndRead=0;
                }
                Rx2TimeOut_mSec = 50;
        }
}
#pragma vector = 0x0D            //  Ngat Timer1
__interrupt void TIM1_OVERFLOW_ISR(void)
{
    if(TIM1_SR1_bit.UIF)        //  Khi có ngat update
    {
        TIM1_SR1_bit.UIF=0;     //  Xoá co
        
        //checkCB();
        //u32TimeSys_mSec++; 
        //if(u32TimeSys_mSec == 20000) u32TimeSys_mSec=0;             
        if(Rx2TimeOut_mSec)         //Rx2 Timeout = 0 nhay xuong else
        {
                  
            Rx2TimeOut_mSec--;
        }
        else
        {
            if(Rx2Index)            
            {
                Rx2_EndRead = 1;
                Rx2ByteFind = Rx2Index;   
                Rx2Index = 0;
            }
        }
    }
}