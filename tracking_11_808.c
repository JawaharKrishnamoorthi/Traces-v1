
#include <24fj64ga002.h>
#device ADC=10
//#device ICD=TRUE
#use delay(clock=16000000)
#use rtos(timer=1,minor_cycle=100ms,statistics)//

#use fixed_io(b_outputs=PIN_b6)
#fuses nowdt,protect,FRC_PLL
//#build (stack=256)

#define fet1 PIN_b6//d5
//#define fet2 PIN_b14//d4

//#define led1 PIN_b10
//#define led2 PIN_b11
//#define led3 PIN_b12
//#define led4 PIN_b13

//#define TOSTRING(s)   #s
//#define nmea(ss) NMEA_##ss();
//------------------------------------------------------------------------------//
#include<gsm_7.h>
//------------------------------------------------------------------------------//
#task(rate=500ms,max=10ms)
void boot();
#task(rate=2000ms,max=20ms)
void process_data();
#task(rate=10000ms,max=50ms,enabled=false)
void post_data();
#task(rate=1500ms,max=15ms,enabled=false)
void post_data_fail();
#task(rate=300ms,max=15ms)
void battery_fail_check();
#task(rate=1000ms,max=50ms,enabled=false)
void gps_rx();
//#task(rate=60s,max=30ms)
//void post_status();

void key_scan();
void cmd_exe();
//void get_gps(); 
//void NMEA_GPRMC(); 
void process();
float rad2deg(float rad);
float deg2rad(float deg);
float distance(float lat1, float lon1, float lat2, float lon2, char unit);
void conversion(int dat,int add);
float conversion2(float yyy);

void boot()
{  
   fprintf(BUG,"-TASK1-\n\r");    
   ack_log=init_gsm(jl); 
   fprintf(BUG,"ACK-%u\n\r",ack_log);    
   if(ack_log){jl++;}
   if(jl>=4){
      init_var=1;
      post_var=0;
      jl=1;
      rtos_disable(boot);
      rtos_enable(gps_rx);   
      rtos_enable(post_data);   
      }     
   ack_log = 0;
}
   
void process_data()
{  
    process();     
    fprintf(BUG,"*");  
}

void gps_rx()
{    
//if (post_var == 0 ){
      GET_GPS   
      timeout_v = 400000;
      timeout_error=FALSE;
     
      while((string3 [0] != 'C' || string3 [j] != 0x0d) && !timeout_error){
      string3[j] = timed_getc();
      if(string3 [0] == 'C' && string3 [j] != 0x0d)
      j++;
      }
      if(string3 [0] == 'C' && string3 [j] == 0x0d){
         timeout_error_2 = true;
         fprintf(BUG,"LL-%s,%02u\n\r",string3,j);   
      }        
//      fprintf(BUG,"LL-%s,%c\n\r",string3,string3[j]);   
//   }
   if(timeout_error){
   j=0;
   string3 [0] = 0;
   fprintf(BUG,"TO\r\n");
   }   
}


void post_data()
{     
//   set_timer2(0);
   output_bit(fet1,0);
   if(post_var==1 && init_var==1){   
   ack_log1=cip_start();   
//   fprintf(BUG,"PF-%u\n\r",post_fail_count);    
   if(ack_log1==81)
   {  
      cip_send();
      fprintf(BUG,"Stage 1\n\r");
      rtos_await(ack_1('>'));
      if(timeout_error!=TRUE){
      post();
      fprintf(BUG,"Stage 2\n\r");
      rtos_await(ack_1('1'));
      if(timeout_error!=TRUE)
      fprintf(BUG,"Stage 3\n\r");
      }
      cip_close();  
      old_la=new_la;
      old_ln=new_ln;
      old_hrs=new_hrs;      
   //   output_low(led2);
  //    output_high(led1);
      post_fail_count=0;      
   }else{
 //     output_low(led1);
 //     output_high(led2); 
      cip_close();
      rtos_disable(post_data);   
      rtos_disable(gps_rx);   
 //     rtos_disable(post_status);   
      rtos_enable(post_data_fail);   
   }
   post_var=0;
   ack_log1=0;      
   }// else {delay_ms(25);}
//   rtos_stats(post_data,&stats);
//   fprintf(BUG,"TASK4:%lu-%lu-%lu-%lu",stats.task_total_ticks,stats.task_min_ticks,stats.task_max_ticks,stats.hns_per_tick); 
   //  rtos_stats(post_data);
}

void post_data_fail()
{
   ack_log1=cip_start();  
   if(ack_log1==81)
   {  
      post_fail_count=0;
      cip_close();
      rtos_disable(post_data_fail);
      rtos_enable(post_data);
      rtos_enable(gps_rx);
//      rtos_enable(post_status);   
      }
   else{
      post_fail_count+=1;
      cip_close();
      if(post_fail_count>=5){
      output_bit(fet1,1);
      post_fail_count=0;
      init_var=0;
      jl=1;
      rtos_disable(post_data_fail);
      rtos_enable(boot);
      rtos_enable(post_data);
//     rtos_enable(post_status);   
      }
   }
}

void battery_fail_check()
{
boolean x,x1;

   x = input(pin_A1);
   x1 = input(pin_A4);
   p_s = x;
   if(x ^ last_a){   
   p_status=1;       
   }else  
   if(x1 ^ last_a4){
   p_status=2;       
   }  
   last_a = x;
   last_a4 = x1;
}
/*
void post_status()
{
   fprintf(BUG,"ST\n\r"); 
   ack_log1=cip_start();      
   if(ack_log1==81)
   {     
      dir = 1;
      post();     
      dir = 0;
   }else{           
      cip_close();
      rtos_disable(post_data);   
      rtos_disable(post_status);   
      rtos_enable(post_data_fail);   
   }
}*/

void NMEA_GPRMC()
{
   while(ptr!=0) {  
      if(nd == 1){nd++;s_time = strtok(0, term);}
      if(nd == 2){nd++;lat = strtok(0, term);}
      if(nd == 3){nd++;lon = strtok(0, term);}
 //     if(nd == 7){nd++;s_date = strtok(0, term);}
 //     if(nd == 8){nd++;s_date1 = strtok(0, term);}
      ptr = strtok(0, term);  
      nd++;
      }
      f_lat = atof48(lat);
      f_lon = atof48(lon);      
      s_date_time = atoi48(s_time);
      i_date = s_date_time / 1000000;
      f_time = s_date_time % 1000000;
      
//      if(i_date < 500)
//      i_date = atoi32(s_date1);
      fprintf(BUG,"GC-%s,%s,%s\n\r",s_time,lat,lon); 
}

void NMEA_GPVTG()
{
   while(ptr!=0) {  
      if(nd == 4){nd++;speed_k = strtok(0, term);}         
      ptr = strtok(0, term);  
      nd++;
      }
//      fprintf(BUG,"GV-%s\n\r",speed_k);   
}

void NMEA_GPGLL()
{
   while(ptr!=0) {  
      if(nd == 0){nd++;lat = strtok(0, term);}
      if(nd == 2){nd++;lon = strtok(0, term);}
      if(nd == 4){nd++;s_time = strtok(0, term);}      
      ptr = strtok(0, term);  
      nd++;
      }
      f_lat = atof48(lat);
      f_lon = atof48(lon);
      f_time = atoi32(s_time);
//      fprintf(BUG,"GL-%s,%s,%s\n\r",time,lat,lon);    
}

void NMEA_GPGGA()
{
   while(ptr!=0) {  
      if(nd == 0){nd++;s_time = strtok(0, term);}
      if(nd == 1){nd++;lat = strtok(0, term);}
      if(nd == 3){nd++;lon = strtok(0, term);}
      ptr = strtok(0, term);  
      nd++;
      }
      f_lat = atof48(lat);
      f_lon = atof48(lon);
      f_time = atoi32(s_time);
//      fprintf(BUG,"GA-%s,%s,%s\n\r",s_time,lat,lon);   
}

//------------------------------------------------------------------------------
void main ( )
{
//   setup_oscillator(OSC_INTERNAL);      
//   set_tris_b( 0x00 );
   set_pullup(true,pin_b6);
//   set_pullup(false,pin_b15); 
   last_a = input_state(pin_A1);
   last_a4 = input_state(pin_A4);
   
   delay_ms(2000);
//   enable_interrupts(global);
//   enable_interrupts(INT_CNI);
//   enable_interrupts(INTR_CN_PIN | PIN_A1);
//   enable_interrupts(int_rda);
//   disable_interrupts(int_rda2);
 //  setup_timer2(TMR_INTERNAL | TMR_DIV_BY_64);
//   set_timer2(0); 
   
//   setup_adc_ports( sAN0 );
//   setup_adc(ADC_CLOCK_INTERNAL );
//   set_adc_channel( 0 );

//   port_b_pullups(true);    
   output_high(fet1);
   delay_ms(1000);
   output_low(fet1);
//   output_low(led1);output_high(led2);
//   output_low(led4);output_high(led3);
  // fprintf(GPS"Hello World");
   if(init_gsm_1(1))init_gsm_1(2); 
   switch ( restart_cause() ) {
   case RESTART_POWER_UP:delay_ms(10000); 
//                        if(init_gsm_1(1))init_gsm_1(2);                        
                        fprintf(GSM,"ATE0%c",enter);
//                        init_gsm_1(2);
//                        delay_ms(10);
//                        fprintf(GSM,"AT+CGREG=1%c",enter);
//                        delay_ms(10);
//                        reset_cpu();
                        break; 
   case RESTART_BROWNOUT:fprintf(BUG,"BOOT 1\n\r");
                        break; 
   case RESTART_WATCHDOG:fprintf(BUG,"BOOT 2\n\r");  
                        break; 
   case RESTART_SOFTWARE:fprintf(BUG,"BOOT 3\n\r");
                        break; 
   case RESTART_MCLR :  fprintf(BUG,"BOOT 4\n\r");                        
                        break;   
   case RESTART_ILLEGAL_OP:fprintf(BUG,"BOOT 5\n\r");
                        break;
   case RESTART_TRAP_CONFLICT:fprintf(BUG,"BOOT 6\n\r");
                        break;
   }  
//   output_low(led1);
//   output_low(led2);
//   output_low(led3);
//   output_low(led4);      
     
   rtos_run();
}   
//------------------------------------------------------------------------------

void process()
{  
p_hrs = 5;
int8 clng=0,ip=0;
char sstring[110]={},nmea[8]="CGNSINF",nmea1[7]="$GPGLL",nmea2[7]="$GPGGA",nmea3[7]="$GPVTG";
   nd=0;
//   strcpy(string3,"$GPRMC,073004.00,A,1120.62884,N,07743.04195,E,0.009,,070815,,,A*7E");   
   strcpy(term,",");
   if(timeout_error_2 == true){ 
   clng = strlen(string3);
   strcpy(sstring,string3);  
   fprintf(BUG,"L-%s>%u\n\r",sstring,clng);                                          //&& string3[strlen (string3)] == 0x0a
   ptr = strtok(sstring, term);
   if(!strncmp(sstring,nmea,7) && clng >= 62)NMEA_GPRMC();   
   if(!strncmp(sstring,nmea1,6) && clng >= 30)NMEA_GPGLL();   
   if(!strncmp(sstring,nmea2,6) && clng >= 50)NMEA_GPGGA();   
   if(!strncmp(sstring,nmea3,6) && clng >= 30)NMEA_GPVTG(); 

for(ip=0;ip<=j;ip++) 
      string3[ip]=0;
      j=0;      
      timeout_error_2 = false;     
   }
//   new_la = conversion2(f_lat);
//   new_ln = conversion2(f_lon);
   new_la = f_lat;
   new_ln = f_lon;
//   fprintf(BUG,"GC-%lu,%lu,%2.5f,%2.5f\n\r",f_time,i_date,new_la,new_ln);    
//---------------- TIME  ------------------------------------------------------            
  new_sec = f_time % 100;            
       v1 = f_time / 10000;            
  new_min = (f_time % 10000) / 100;
       v4 = (new_min * 60) + new_sec;
  new_hrs = v1 + (  v4 / 3600 );
  fprintf(BUG,"%02u:%02u:%02u\n\r",v1,new_min,new_sec);      
//  fprintf(BUG,"GT-%f\n\r",v4); 
//-----------------DATE -------------------------------------------------------            
  date = i_date % 100;            
  year = i_date / 10000;            
  month = (i_date % 10000) / 100;       
  fprintf(BUG,"%04lu/%02u/%02u\n\r",year,month,date); 
//----------------INIT DATE/TME------------------------------------------------            
  if(old_hrs == null)
  old_hrs = new_hrs;
  if(old_la == null && old_ln == null){
    old_la = new_la;
    old_ln = new_ln;
    }        
//------------------------------------------------------------------------------     
    if(abs(old_la - new_la) >= 0.0001 || abs(old_ln-new_ln) >= 0.0001){
    post_var = 1; 
    dir = 1;
    distt = distance(old_la,old_ln,new_la,new_ln,'K');  
    if(old_hrs > new_hrs){               
       speed = distt / (24 - (old_hrs - new_hrs));    
          }else {
           speed = distt / (new_hrs - old_hrs);
          }    
//    fprintf(BUG,"Di-%0.5f,%0.5f\n\r",abs(old_la-new_la),old_ln-new_ln);
    }else if(old_hrs != new_hrs && year >= 2016){
    post_var = 1; 
    dir = 0;
    speed = 0;
    }

 /*           
   p_min = min + 30;
   if(p_min>59){
      p_min -= 60 ;
      p_hrs+=1;
      }
      p_hrs += hrss;
      if(p_hrs>23){
      p_hrs -=24;
      }
      if(p_hrs > 12){am_pm="PM";
         p_hrs -= 12;
      }else am_pm="AM";
      if(p_hrs == 0)
      p_hrs = 12;
      */
}

float distance(float lat1, float lon1, float lat2, float lon2, char unit) {

    float dlat,dlon,a,c,d;
    int16 R = 6371;

   dlat = (lat2-lat1) * 0.0174532925; 
   dlon = (lon2-lon1) * 0.0174532925;

    a = sin(dlat/2) * sin(dlat/2) +
            cos((lat1 * 0.0174532925)) * cos((lat2 * 0.0174532925)) *
            sin(dlon/2) * sin(dlon/2);
            
    c = 2 * atan2(sqrt(a),sqrt(1-a));
    d = R * c;    
    d = d - 0.00144974; 
    return d;
}

float conversion2(float yyy)
{
   v4 = modf(yyy,&v5);
   v1 = v5;
   yy = v1 /100;
   v5 = v1 % 100;
//   v4 = v2;
//   v2 = v4 / 10000;
   v4 = (v5 + v4) / 60;       
   v6 = yy;
//   v6 = v6 + v5;  
   return v6 + v4;  
}

/*
void conversion(int dat,int add)
{
   int i,j,k;
   i=dat/100;
   j=dat%100;
   k=j%10;
   j=j/10;
   i=i|0x30;
   j=j|0x30;
   k=k|0x30;
   lcd_com(add);
  // lcd_data(i);
   lcd_data(j);
   lcd_data(k);
}*/

#INT_RDA
void rda_isp()
{
   unsigned char vv3;  
   
 //  if(string3[j] != 0x0d)string3[j] = getch(GPS);   
   if(string3[j] == 0x0d){
      if(string3[0] != 'C')
      {
      string3[j]=0;
      j=0;
      }
      else{
//      vv3 = getch(GPS);
      timeout_error_2 = true;
      }
   }
   else
   if(string3[0] == 'C' && string3[j] != 0x0d){
   j++;
   }
}

#INT_ADDRERR 
void default_isr() {

//   printf("Unexplained interrupt\r\n");
}

#INT_CNI
void cni_isr() {

   if((!input(pin_A1)) && (last_a)){
   p_status=1; 
   last_a = 0;
   disable_interrupts(INT_CNI);
   }  
//   clear_interrupt(INT_CNI);
//   clear_interrupt(INTR_CN_PIN | PIN_A1);
}







