/****************IMEI-866762029510326;BAUD-19200  *****************************/
/****************IMEI-866593020769383;BAUD-9600  *****************************/
/****************IMEI-866762027240124;BAUD-19200  *****************************/

#include <string.h>
#include <math.h>
#include <stdlib.h>
//#include <time.h>

#pin_select U1TX=PIN_b14
#pin_select U1RX=PIN_b15
#pin_select U2RX=PIN_b5
#pin_select U2TX=PIN_b4

//#use rs232(baud=9600,xmit=PIN_C6,rcv=PIN_C7,stream=GPS,errors)
//#use rs232(baud=19200,xmit=PIN_B14,rcv=PIN_B15,force_sw,stream=BUG,errors)
//#use rs232(baud=19200,UART1,stream=GPS,errors)
#use rs232(baud=57600,UART1,stream=BUG,errors)
#use rs232(baud=57600,UART2,stream=GSM,errors)
//#use rs232(baud=9600,xmit=PIN_C2,rcv=PIN_C3,force_sw,stream=GSM,errors)

#define GET_GSM_STATUS        fprintf(GSM,"AT+CGREG?%c",enter);
#define GPS_ON                fprintf(GSM,"AT+CGNSPWR=1%c",enter);
#define CANCEL_ECHO           fprintf(GSM,"ATE0%c",enter);
#define START_CIP             fprintf(GSM,"AT+CIPSTART=%cTCP%c,%cwww.gpstraces.com%c,%c80%c%c",set,set,set,set,set,set,enter);
#define GET_CIP_STATUS        fprintf(GSM,"AT+CIPSTATUS%c",enter);               //119.18.57.60
#define GET_GPS               fprintf(GSM,"AT+CGNSINF%c",enter);
#define GET_GPS_STATUS        fprintf(GSM,"AT+CGPSSTATUS?%c",enter);
#define GPS_REBOOT            fprintf(GSM,"AT+CGNSRST=0%c",enter);

//#define ACK_OK "OK" 
//#define ACK_ERROR "ERROR"

boolean exception(unsigned char bb,INT8 cc);
int init_gsm(int dat);
int init_gsm_1(int dat);

int8 cip_start();
void cip_send();
void cip_send1();
void cip_send2();
void cip_send3();
void cip_close();
char timed_getc();

boolean timeout_error=false,timeout_error_2=false,last_a=0,last_a4=0,p_s=0;
int8 j=0,post_var=0,init_var=0,nd=0;
unsigned char string2[30]={},v3=0;
unsigned char string6[30]={};
int8 num_attempt=0;
unsigned char set=0x22,enter=0x0d,sent=0x1a,lf=0x0a;
unsigned char ACK_OK[3]="OK",APN_RE[18]="CSTTairtelgprscom",STATUS_RE[17]="OKSTATEIPINITIAL",STATUS_CON[12]="OKCONNECTOK";
unsigned char GSM_STATE[8]="CGREG01",GSM_STATE_R[8]="CGREG05";
unsigned char string3[110]={};

char *ptr=0,term[2],*lat=null,*lon=null,*s_time=0,*s_date=0,*s_date1=0,*speed_k;
char string[20]={};

INT8 buf_var=0,cmp_val,p_status=0,dir=0;
int8 jl=1,jm=1,p_hrs=5; 
int8 ack_log=0,ack_log1=0,sec;
int old_min=0,old_sec=0,new_min=0,new_sec=0,date=0,month=0,year=0;
int v1=0,post_fail_count=0; 
signed int32 yy=0,f_time=0,i_date=0;
signed int48 s_date_time;
int32 timeout,timeout_v,timeout_3=0,r_boot=0;

float v4=0,v5=0,f_lat=0,f_lon=0,new_hrs=0,old_hrs=null;
float v6=0,old_la=null,old_ln=null,new_la,new_ln,distt=0,speed=0;

struct rtos_stas_struct {
                     unsigned int32 task_total_ticks;   //number of ticks the task has 
                                                                                 //used
                     unsigned int16 task_min_ticks;   //the minimum number of ticks 
                                                                                 //used
                     unsigned int16 task_max_ticks;  //the maximum number of ticks 
                                                                                 //used
                     unsigned int16 hns_per_tick;       //us = (ticks*hns_per_tick)/10
             }stats;

int init_gsm_1(int dat)
{
   switch (dat)
{
   case 1:  strncpy(string2,ACK_OK,2);
            fprintf(GSM,"A%c",enter);
            if(!exception(2,2))
            {     
               num_attempt+=1;
               if(num_attempt>20){output_high(fet1);reset_cpu();}               
               return 0;   
               break;
            }
               num_attempt=0;
               timeout_error=FALSE;                
               return 1;
               break;
   case 2:  strncpy(string2,ACK_OK,2);
            fprintf(GSM,"AT+IPR=57600%c",enter);
            if(!exception(2,2))
            {     
               num_attempt+=1;
               if(num_attempt>20){output_high(fet1);reset_cpu();}               
               return 0;   
               break;
            }
   
               num_attempt=0;
               timeout_error=FALSE;                
               return 1;
               break;
               
   default: break;              
   }
}

int init_gsm(int dat){ 
switch (dat)
{
   case 1:  strncpy(string2,ACK_OK,2);            
            CANCEL_ECHO
            if(!exception(2,2))
            {     
               num_attempt+=1;
               if(num_attempt>10){output_high(fet1);reset_cpu();}               
               return 0;   
               break;
            }
               num_attempt=0;
               timeout_error=FALSE;                
               return 1;
               break;
               
   case 2: strncpy(string2,GSM_STATE,7);
           strncpy(string6,GSM_STATE_R,7);
           GET_GSM_STATUS
               timeout_error=FALSE;
               if(!exception(7,7))
               {
                  num_attempt+=1;
                  if(num_attempt>20){output_high(fet1);reset_cpu();}   
                  return 0; 
                  break;
               }else{
                  num_attempt=0;
                  timeout_error=FALSE;                    
                  return 1;
                  break;}
               
   case 3:  strncpy(string2,ACK_OK,2);
            fprintf(BUG,"AT+GPS=1\n\r");
            GPS_ON
            if(!exception(2,2))
            {     
                num_attempt+=1;
                if(num_attempt>20){output_high(fet1);reset_cpu();}   
                return 0; 
                break;
            }
               num_attempt=0;
               timeout_error=FALSE;               
               return 1;  
               break;     
   case 4:  strncpy(string2,ACK_OK,2);
            fprintf(BUG,"AT+ECHARGE=1\n\r");
            fprintf(GSM,"AT+ECHARGE=1%c",enter);   
            if(!exception(2,2))
            {     
                num_attempt+=1;
                if(num_attempt>20){output_high(fet1);reset_cpu();}   
                return 0; 
                break;
            }
               num_attempt=0;
               timeout_error=FALSE;               
               return 1;  
               break;  
   case 5:  strncpy(string2,ACK_OK,2);
            fprintf(BUG,"AT+RGPS=1\n\r");
            GPS_REBOOT
            if(!exception(2,2))
            {     
               num_attempt+=1;
               if(num_attempt>10){output_high(fet1);reset_cpu();}               
               return 0;   
               break;
            }
               num_attempt=0;
               timeout_error=FALSE;               
               return 1;  
               break;                
   
    default: break;              
   }
}         

boolean exception(unsigned char bb,INT8 cc)
{   
//   string[1]='X';
   buf_var=0;cmp_val=bb;   
   fprintf(BUG,"CASE-%u\n\r",jm);   
   timeout_error=FALSE;
   timeout_v = 400000;
   while(buf_var <= cc && !timeout_error)   
   {
      v3 = timed_getc();
      if(isalpha(v3) || isdigit(v3)){
      string[buf_var]=v3;   
      fprintf(BUG,"%c",v3); 
      buf_var++;
      }      
   }   
//   fprintf(BUG,"%s",string);
   fprintf(BUG,"-\n\r");
//   timeout_error=FALSE;
//   gets(string,GSM);
   if(!strncmp(string,string2,cmp_val) || !strncmp(string,string6,cmp_val))  
   return 1;   
   else
   return 0;   
}

char timed_getc() {   

   timeout_error=FALSE;
   timeout=0;
   while(!kbhit(GSM) && (++timeout<timeout_v)) // 1/2 // second
          delay_us(5);
   if(kbhit(GSM)) {      
          return(getc(GSM));}
   else {
          timeout_error=TRUE; 
          return(0);
   }
}

boolean ack_1(char dat)
{
      timeout_v = 40000000;           
      if(timed_getc() == dat || timeout_error==TRUE)
      return 1;    
}

int8 cip_start()
{
    strncpy(string2,STATUS_CON,11);
    START_CIP
 //   fprintf(BUG,"AT+\n\r");    
    if(!exception(10,11))
    {
//      if(num_attempt++<=2)
//      goto l0;
//      else 
      return 80; 
      }
      else return 81;  
}

void post()
{ 
//     disable_interrupts(INT_CNI);
//      cip_send();
      cip_send1();
      cip_send2();  
      cip_send3();
//      timeout_v = 800000;
//      rtos_await(kbhit());
//      while(timed_getc()!='1' && timeout_error!=TRUE);            
//      cip_close();     
//      enable_interrupts(INT_CNI);
}

void cip_send()
{   
   fprintf(GSM,"AT+CIPSEND");
   fprintf(GSM,"%c",enter);   
//   while(timed_getc()!='>' && timeout_error!=TRUE);
//   if(timeout_error==TRUE){
//      cip_close();     
//      return;      
}

void cip_send1()
{                                                                    //139.59.8.190
   fprintf(GSM,"GET http://www.gpstraces.com/u/u?i=865067022435708&");   //imeiid=866762027240124//imeiid=866593020769383//866762027261112
   fprintf(GSM,"ln=");
   fprintf(GSM,"%2.5g",new_ln);//2.6   
   fprintf(GSM,"&");      
   fprintf(GSM,"lt=");
   fprintf(GSM,"%2.5g",new_la);   
}

void cip_send2()
{
   fprintf(GSM,"&"); 
   fprintf(GSM,"s=");   
//   if(dir == 1)fprintf(GSM"1"); else    
   fprintf(GSM,"%3.1g",speed);
   fprintf(GSM,"&t=");
   fprintf(GSM,"%04lu/%02u/%02u",year,month,date);   
}

void cip_send3()
{      
   fputc('%',GSM);
   fprintf(GSM,"20"); 
   fprintf(GSM,"%02u:%02u:%02u",v1,new_min,new_sec);  
   fprintf(GSM,"&"); 
   fprintf(GSM,"u=%02u",dir);  
   fprintf(BUG,"u=%02u\n\r",dir);  
//   delay_ms(50);
   fprintf(GSM,"%c",enter);
   fprintf(GSM,"%c",lf);
   fprintf(GSM,"%c",enter);
   fprintf(GSM,"%c",lf);
   fprintf(GSM,"%c",sent);  
//   for(sec=0;sec<=5;sec++)
//   fprintf(BUG,"A-%c\n\r",timed_getc());
}
void cip_close()
{
//   delay_ms(100);//100
   fprintf(GSM,"AT+CIPCLOSE");
   fprintf(GSM,"%c",enter);
} 



