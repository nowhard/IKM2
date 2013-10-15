#include "ADuC845.h"
#include <stdio.H>
#include <stdlib.H>
#include <string.H>
#include <math.h>
#include <intrins.h>
#include "flash.h"


#define Adress_ustroistva 0x01
#define Nomer_ustroistva 0x01

unsigned char xdata Primechanie[30]="GEOSPHERA 05.03.2009";

void beep(int pik);

#define kol_izmer_data 3
#define period_oprosa_klavi 5 // опрос клавы в миллисекундах 5*20=100 мсек
#define delay_miganie 45	  // задержка мигания цифры
#define delay_led -1
#define max_napr_acp 1.25//2.048   // 1.25
#define max_napr_vhod 9.999//5.0
#define kof_ves 5.0	// в тоннах
#define max_dac 1.82
#define min_dac 0.0 
#define kof_dac ((float)max_dac - (float)min_dac)  // 2.21 - max=2.25 min=0.04
#define DEBUG  0


const float xdata ves_raz=0.000000076293954407448101931584111640943;//0.000000195312511641533000000000 ;
//0.00000012207031977595804786432074691777;//0.000000076293954407448101931584111640943;
const float xdata ves_raz_dac=0.00061050061050061050061050061050061;
long xdata dac_peremen=0;
volatile long data adc_data=0;	 // для накапления показаний ацп
volatile long data adc_datchik=0;  // показания ацп усреднённое
volatile long adc=0;	   // значение ацп
unsigned char adc_gotov=0;

float xdata adc_filtr_ves[2]=0; 
float xdata adc_filtr_napr[2]=0;

float xdata verhniy_pridel=0;
float xdata nijniy_pridel=0;
float xdata interval=0;										  
float xdata koef_usileniya_ves=0;
float xdata koef_usileniya_dac=0;

unsigned char idata kol_vivod=0;
unsigned char idata str_led[10]={0}; // формирование строки для вывода на линейку
unsigned char idata str_dec[10]={0}; // формирование строки для вывода на цифру
unsigned char idata str_yarkost_dec[2]={0};	// коэффициент яркости цифрового индикатора
unsigned char idata str_led_procent_yarkosti_dec[10]={0}; // индикатор процента яркости цифрового индикатора
bit perestroit_dec=0;


unsigned char idata op[30]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
unsigned int data tim_h_0=0;
unsigned int data tim_l_0=0;
unsigned char chastota_vivoda_na_indikator=0;
unsigned int restart_led_dec_indikator=0;
char klava=-1;	// номер нажатой клавиши
bit out_port=0;	// сработала нужная кнопка
unsigned int period=0; // период опроса клавы

bit start_miganie=0;	// включение мигания
char idata cifra=0;	// отслеживание значения одного знакоместа в режиме перебора
unsigned char idata nomer_cifri=0; // перемещение от одной цифры к другой
unsigned char miganie=0;   // вспомогательная переменная для мигания
bit _miganie_ =1;		// 1 - убрать цифру, 0 - высветить её

bit BIP_i_LED=0;	   // включение диамика и диода
unsigned char idata vr_bip_led=0; // время звучания динамика и диода
char idata vremay_piska[5]={0};
unsigned char idata kol_piskov=0;
bit	cickl_bip_led=0;
unsigned char idata incriment_vremeni=0;

float xdata moment_srednee=0; //
float xdata moment_srednee_out_data=0;
float xdata moment_mgnovennoe=0; //
float xdata moment_mgnovennoe_out_data=0;
float xdata ustavka=1.0;  // значение уставки, при привышении которого произходит смена цветов на шк.ин.
float xdata koef_tar=0;	 // коэф. татировки
long idata temp_us_tar=0;
bit vhod_v_edit=0;
unsigned char write_flash_pc_us_tar=0;
unsigned char rejim=1;
unsigned char mashtab=1;
long idata int_ustavka=100;
long idata int_tar_koef=100;
float ves_dioda=0.04; // вес одного диода РАСЧИТЫВАЕТСЯ и ЗАВИСИТ от маштаба

#if DEBUG==0
// Для протокола
unsigned long xdata pos=0;
unsigned char xdata Prinyato=0;
unsigned char xdata Peredano=0;										// количество передаваемых данных
unsigned char xdata uart_gotov=0;								    // запрет приема во время передачи а так же сигнал о том что пришел кадр
unsigned char xdata vivedeno=0;
unsigned char xdata kol_byte_prin=0;	
unsigned char xdata in_out_buffer[172]={0};
#endif

bit pipech=0;

//--------------------------------------------------------------
#if DEBUG==0
unsigned char CyclicControl(void *Spool_temp,unsigned int Count)	 //  НЕОБХОДИМЫЙ КОД ДЛЯ ПРОТОКОЛА++++++++++++++++
{
    unsigned char *Spool=(unsigned char *)Spool_temp;
    unsigned char data CRC = 0;
	//char C=0;
	unsigned int data i=1;
    for(i=1; Count>i; i++)
        {
		//C=Spool[i];
        CRC = CRC ^ Spool[i];
        // циклический сдвиг вправо
        CRC = ((CRC & 0x01) ? (unsigned char)0x80: (unsigned char)0x00) | (unsigned char)(CRC >> 1);
        // инверсия битов с 2 по 5, если бит 7 равен 1
        if (CRC & (unsigned char)0x80) CRC = CRC ^ (unsigned char)0x3C;
        }
    return CRC;
} 
//-----------------------------------------------------------------
void delay(long length)  
{
while(length >=0)
length--;
}
//------------------------------
void LED_OUT_LINE(unsigned char *out,bit initch,unsigned char *inith)  // 1-массив данных 2-размер массива данных 3- бит режима вывода:0 - вывод информации 1- инициализация устройства 4- массив инициализации микросхемы вывода Мах7219
{
unsigned char init[2]={0};	 // init[0] - адрес регистра init[1] - данные
char kol_pered=0;			 // перебор знакомест
unsigned char kol_byt=0;	 // перебор двух байтных данных
unsigned char byte=0;		 // для разбора байта данных
char tmp_kol=0;
unsigned char bit_count=0;

 if(initch==0)
  {
	for(kol_pered=1;kol_pered<9;kol_pered++)  // выводим восемь знакомест
     {
	   init[0]=kol_pered;
	   //delay(delay_led);
       CS_LED=0;
	   
	   init[1]=out[tmp_kol];

	   for(kol_byt=0;kol_byt<2;kol_byt++)  	  // вывод двух байтных данных
        {
          byte=init[kol_byt];
	      for(bit_count=0;bit_count<8;bit_count++) // побитный вывод байта
           {
            byte=byte<<1;
            DIN=CY;	 
	        //delay(delay_led);//  задержка
            CLK=1;
            //delay(delay_led);
            CLK=0;
	        //delay(delay_led);
           }
        }
		CS_LED=1;
		delay(1);
		CS_LED=0;
       tmp_kol++;
	 }
   
  }
  else
   {
     CS_LED=0;
     init[0]=inith[0];
     init[1]=inith[1];

      for(kol_byt=0;kol_byt<2;kol_byt++)  // включение питания
       {
        byte=init[kol_byt];
	    for(bit_count=0;bit_count<8;bit_count++)
         {
          byte=byte<<1;
          DIN=CY;	 
	      delay(delay_led);//  задержка	  1.67 мсек
          CLK=1;
          delay(delay_led);
          CLK=0;
	      delay(delay_led);
         }
	    delay(delay_led);	 
       }
		CS_LED=1;
		delay(1);
		CS_LED=0;
   }
}
//------------------------------
void LED_OUT_DEC(unsigned char *out,char kol,bit initch,unsigned char *inith, bit decode)  // 1-массив данных 2-размер массива данных 3- бит режима вывода:0 - вывод информации 1- инициализация устройства 4- массив инициализации микросхемы вывода Мах7219, 5- декодер: 1- вывод с помощью внутреннего декодера, 0- без внутреннего декодера 
{
unsigned char init[2]={0};	 // init[0] - адрес регистра init[1] - данные
bit tocka=0;				 // признак точки 
unsigned char propusk=0;	 // не используется
char kol_pered=0;			 // перебор знакомест
unsigned char kol_byt=0;	 // перебор двух байтных данных
unsigned char byte=0;		 // для разбора байта данных

unsigned char bit_count=0;

if(initch==0)
{
for(kol_pered=4;kol_pered>0;kol_pered--)  // выводим пять знакомест
  {
   init[0]=kol_pered;
   //delay(delay_led);
   CS_DEC=0;
   if(kol<=0)
   {
   init[1]=decode?0x0F:0x00;
   kol=1;
   }
   else
   {
    if(out[kol-1]==0x2E)   // обнаружение точки 
     {
     tocka=1;
     kol_pered++;	// позуцию знакоместа не меняем
	 kol--;			// сдвигаемся на следующий символ
     continue;
     }
    if(tocka==1)
     {
     init[1]=(out[kol-1]&0x0F)|0x80; //- вывод с точкой
     tocka=0;	// - вывод с точкой
     }
    else
     init[1]=decode?out[kol-1]&0x0F:out[kol-1]; //- вывод чисел после точки
   }
  for(kol_byt=0;kol_byt<2;kol_byt++)  	  // вывод двух байтных данных
   {
    byte=init[kol_byt];
	 for(bit_count=0;bit_count<8;bit_count++) // побитный вывод байта
     {
     byte=byte<<1;
     DIN=CY;	 
	 //delay(delay_led);//  задержка
     CLK=1;
     //delay(delay_led);
     CLK=0;
	 //delay(delay_led);
     }
   }
CS_DEC=1;
delay(1);
CS_DEC=0;

kol--;	 
  }
}
else
{
CS_DEC=0;
init[0]=inith[0];
init[1]=inith[1];

for(kol_byt=0;kol_byt<2;kol_byt++)  // включение питания
   {
    byte=init[kol_byt];
	 for(bit_count=0;bit_count<8;bit_count++)
     {
     byte=byte<<1;
     DIN=CY;	 
	 delay(delay_led);//  задержка	  1.67 мсек
     CLK=1;
     delay(delay_led);
     CLK=0;
	 delay(delay_led);
     }
	 delay(delay_led);	 
   }
CS_DEC=1;
delay(1);
CS_DEC=0;
}
}
//-------------------------------------
void LED_OUT_INDIK(unsigned char *out,char kol,bit initch,unsigned char *inith)  // 1-массив данных 2-размер массива данных 3- бит режима вывода:0 - вывод информации 1- инициализация устройства 4- массив инициализации микросхемы вывода Мах7219
{
unsigned char init[2]={0};	 // init[0] - адрес регистра init[1] - данные
bit tocka=0;				 // признак точки 
unsigned char propusk=0;	 // не используется
char kol_pered=0;			 // перебор знакомест
unsigned char kol_byt=0;	 // перебор двух байтных данных
unsigned char byte=0;		 // для разбора байта данных

unsigned char bit_count=0;

if(initch==0)
{
for(kol_pered=6;kol_pered>0;kol_pered--)  // выводим пять знакомест
  {
   init[0]=kol_pered;
   //delay(delay_led);
   CS_INDIK=0;
   if(kol<=0)
   {
   init[1]=0x00;
   kol=1;
   }
   else
   {
     init[1]=out[kol-1]; //- вывод чисел после точки
   }
  for(kol_byt=0;kol_byt<2;kol_byt++)  	  // вывод двух байтных данных
   {
    byte=init[kol_byt];
	 for(bit_count=0;bit_count<8;bit_count++) // побитный вывод байта
     {
     byte=byte<<1;
     DIN=CY;	 
	 //delay(delay_led);//  задержка
     CLK=1;
     //delay(delay_led);
     CLK=0;
	 //delay(delay_led);
     }
   }
CS_INDIK=1;
delay(1);
CS_INDIK=0;
kol--;	 
  }
}
else
{
CS_INDIK=0;
init[0]=inith[0];
init[1]=inith[1];

for(kol_byt=0;kol_byt<2;kol_byt++)  // включение питания
   {
    byte=init[kol_byt];
	 for(bit_count=0;bit_count<8;bit_count++)
     {
     byte=byte<<1;
     DIN=CY;	 
	 delay(delay_led);//  задержка	  1.67 мсек
     CLK=1;
     delay(delay_led);
     CLK=0;
	 delay(delay_led);
     }
	 delay(delay_led);	 
   }
CS_INDIK=1;
delay(1);
CS_INDIK=0;
}
}
//-------------------------------------
void _ADC_ (void) interrupt 6
{
RDY0=0;
adc=ADC0H;
adc=(((adc<<8)|ADC0M)<<8)|ADC0L;
if(adc_gotov<kol_izmer_data)
    {
     adc_data+=adc;
	 adc_gotov++;
	}

#if DEBUG==1
   kol_vivod++;
 #endif
}
//------------------------------------
unsigned char strlench(unsigned char *s)
{
unsigned char i=0;
for(i=0;i<255;i++)
{
if(s[i]==0x00)
return i;
}
}
//-----------------------------------
void perevod_FloatToString(float adc,long ladc,unsigned char *s,int p,bit all,bit it_fl) 
// adc - число с плавающей точкой, ladc - целое число, 
//s - указатель на строку в которую будет переведено число, p - сколько знаков после точки, 
//all - не убирать не значущие нули перед числом, it_fl - какое число передаётся 0 - с п.т., 1 - целое 
{
unsigned char i1=0;
unsigned char y1=0;
unsigned char tochka=3;	 // знаков перед точкой 000. если есть минус то на один знак больше для минуса
unsigned char p1=(char)p;
unsigned long t=1;
bit znak=0;
bit zameni=1;
char r=0;

if(!it_fl)
{
if(adc<0)
{
adc=adc*(-1.0);
znak=1;
r++;
}
for(i1=0;i1<p;i1++)		// 4 значит сколько знаков перед точкой вместе с ней т.е. 000.=4, p это	сколько знаков после/t*=10;
t*=10;
t=(long)(adc*t);
}
else
{
if(ladc<0)
{
ladc*=-1;
znak=1;
r++;
}
t=ladc;
}
 for(i1=0;i1<=(int)(p+tochka);i1++)		// 4 значит сколько знаков перед точкой вместе с ней т.е. 000.=4, p это	сколько знаков после
  {							    // точки. Этот цикл переводит переменную adc в символы но не в ASCII и записывает эти симловы
  if(i1<(int)(p+tochka))				// в массив s, вместе с точкой которая является 4-ым символом, наоборот, и в конец вставляет 0х00.
  {
  if(i1==p)
  s[i1]=0x2E;
  else
  {
  s[i1]=(char)((t%10)|0x30);
  t=t/10;
  }
  }
  else
  s[i1]=0x00;
  }
 for(i1=0;i1<(int)((int)(p+tochka+r)/2);i1++)	// переворачиваем строчку и удаляем лишние нули(0х30), вставляя вместо них 0х00,                                        
  {									// до первой значещей цифры перед запятой
   y1=s[i1];
   if(s[(int)(p+tochka)-1-i1]==0x30&&(int)(p+tochka)>p1+2&&zameni==1&&all==0)
    {
	if(znak)
	 {
     s[(int)(p+tochka)-1-i1]=0x2D;  // вставляем знак минус если отрицательное число
	  if(p!=p1)
	   {
	   s[(int)(p+tochka)-1-i1+1]=0x00;
	
	   }
	 }
	else
	  s[(int)(p+tochka)-1-i1]=0x00;
	  i1=-1;
      p=p-1;
	}
   else
    {
	if(znak)
	{
	zameni=0;
    s[i1]=s[(int)(p+tochka)-i1];
	s[(int)(p+tochka)-i1]=y1;
	}
	else
	{
	zameni=0;
	s[i1]=s[(int)(p+tochka)-1-i1];
    s[(int)(p+tochka)-1-i1]=y1;
	}
	}
  }
}
//---------------------------------
float float_ceil_floor(float fl)
{
float rezult=0;

rezult=fl-(long)fl;

if(rezult>0.5)
 {
 fl=(fl)+1;
 return fl;
 }
else
 {
 fl=(long)fl;
 return fl;
 }
}
//-------------------------------------

void _TR0_ (void) interrupt 1
{
static unsigned int idata i=0;
unsigned char data nom=0x00;
auto unsigned int data knop=0; 
auto unsigned int data line=0;
unsigned char data a=0x00;

int y=0;
int u=0;
char chisl=0;
char uk=0;

char cel=0;
char sel=0;
TH0=tim_h_0;
TL0=tim_l_0;

if(start_miganie==1)
{
if(miganie<delay_miganie)
  miganie++;
 else
 {
  _miganie_^=1;
   miganie=0;
 }
}

 if(i<19)	   // клавиатура опрашивается каждые 100 мсек
   {
   chastota_vivoda_na_indikator++;
   restart_led_dec_indikator++;

   	if(BIP_i_LED==1)
    while(incriment_vremeni<kol_piskov)
	{
	 if(cickl_bip_led==0)
	 {	                    //динамик и диод горит в течении 500 мсек++++++++++++++++++++++++++++
      if(vremay_piska[incriment_vremeni]>vr_bip_led)
	    {  
	     vr_bip_led++;
		 break;
		}
	  else
	    {
	     cickl_bip_led=1;
		 vr_bip_led--;
		 //LED=1;
         BIP=1;
		 break;	  
		}
	  }
	  else
	  {
	   if(vr_bip_led>0)
	    {
		 vr_bip_led--;
		 break;
		}
	   else
	    {
		  incriment_vremeni++;
		  if(incriment_vremeni==kol_piskov)
		  {
	       BIP_i_LED=0;
		   cickl_bip_led=0;
		   break;
		  }
		  else
		  {
		   cickl_bip_led=0;
		   //LED=0;
           BIP=0;
		   break;
		  }
		}
	     
	  }
   	 }     
    for (y=0;y<8;y++)					  //проверка на дребизг два раза в течении 10мсек+++++++++++++
      {
	    if(op[y]<=2)
		  {
		  cel=y/6; 
		  sel=(y-(cel*6))+1;
		  switch(cel)
            {
             case 0: P0_0=0;
                     break; 
             case 1: P0_1=0;
                     break;
             case 2: P0_2=0;
                     break;
             case 3: P0_3=0;
                     break;
            // case 4: P0_4=0;
            //         break;
            }
	      a=P1;
		  a=a>>sel;
		  if(CY==0)
            {
			 if(op[y]==2)
			  {
			  op[y]=3;

			  klava=y;
			  out_port=1;
			  }
			 if(op[y]==1)
			  op[y]=2;
			}
			else
			op[y]=0xFF;	  // ложное срабатывание кнопки

		  switch(cel)
            {
             case 0: P0_0=1;
                     break; 
             case 1: P0_1=1;
                     break;
             case 2: P0_2=1;
                     break;
             case 3: P0_3=1;
                     break;
             //case 4: P0_4=1;
             //        break;
            }

		  }
      }
	
	  								 //--------------------------------------------------------------
    i++;
   }
 else
 {
i=0;
 for(line=0;line<2;line++)
  {
  switch(line)
   {
   case 0: P0_0=0;
           break; 
   case 1: P0_1=0;
           break;
   case 2: P0_2=0;
           break;
   case 3: P0_3=0;
           break;
   //case 4: P0_4=0;
   //        break;
   }
   a=P1;
   for(knop=0;knop<2;knop++)
     {
	 nom=(6*line)+knop;		// вычисления номер кнопки
     a=a>>1;
     if(CY==0)
      {	  
	  if(op[nom]==3||op[nom]==0xFE)	 // проверка на удержание клавиши
	  op[nom]=0xFE;
	  else
	  op[nom]=1;
      }
	  else
	  if(op[nom]==3||op[nom]==0xFE)
	  op[nom]=0xFF;
     }
   switch(line)
   {
   case 0: P0_0=1;
           break; 
   case 1: P0_1=1;
           break;
   case 2: P0_2=1;
           break;
   case 3: P0_3=1;
           break;
   //case 4: P0_4=1;
   //        break;
   }
  }
 }  
}

//-----------------------------------------
void beep(int pik)
{
 if(BIP_i_LED==0)
 {
 // LED=0;
  BIP=0;
  vremay_piska[0]=pik;
  kol_piskov=1;
  vr_bip_led=0;
  incriment_vremeni=0;
  BIP_i_LED=1;
 }

}
//-----------------------------------------
void Indikatornaya_panel(unsigned char rejim,unsigned char mashtab)
{
unsigned char str=0;

if(rejim!=5)
{
  switch(rejim)
  {
   case 1:
           str=1;
           break;
   case 2:
           str=64;
           break;
   case 3:
           str=16;
           break;
   case 4:
           str=8;
           break;
  }

  switch(mashtab)
  {
   case 1:
           str+=4;
           break;
   case 2:
           str+=32;
           break;
   case 4:
           str+=128;
           break;
   case 8:
           str+=2;
           break;
  }
}

LED_OUT_INDIK(&str,1,0,NULL);

}//------------------------------ +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#if DEBUG==0
void _TI_RI_ (void) interrupt 4   // НЕОБХОДИМЫЙ КОД ДЛЯ ПРОТОКОЛА++++++++++++++++
{
if(TI==1)
{  
TI=0;

  if(vivedeno<=Peredano)	 // передача данных
  {
    SBUF=in_out_buffer[vivedeno];
    vivedeno++;    
  }
  else
  {
  P3_5=0;
  vivedeno=0;
  uart_gotov=0;
  Prinyato=0;
  RI=0;
  }
}
if(RI==1)
{
 RI=0;

 //T2=0;
 if(uart_gotov==0)	 // запрет приема вовремя передачи
  {
  if(Prinyato<4)		// общая переменная, подсчитывает количество прининятых и переданных байт
   {
    switch(Prinyato)				 // проверка с начало ли кадр принимается
	   {
		case 0: if(SBUF!=(unsigned char)(0xAA))
		          {
		          Prinyato=0;
				  return;
				  }
				break;
		case 1: if(SBUF!=(unsigned char)Adress_ustroistva)
	     		  {
		          Prinyato=0;
				  return;
				  }
				break;
		case 2: if(SBUF>(unsigned char)(0x05))
	     		  {
		          Prinyato=0;
				  return;
				  }
				break;
		case 3: if(SBUF>(unsigned char)(0x0A))
	     		  {
		          Prinyato=0;
				  return;
				  }
				break;

	   default: break;
	   }
	   
   in_out_buffer[Prinyato]=SBUF;
 
   Prinyato++;
   
     if(Prinyato==4)
	   {     
        kol_byte_prin=in_out_buffer[3];  // получаем длинну данных после заголовка
	   }
 }
 else
 {
  kol_byte_prin--;
  if(kol_byte_prin)	  // принимаем указанное в kol_byte_prin число байт данные
   {
     
     in_out_buffer[Prinyato]=SBUF;
     Prinyato++;
   }
  else
   {
   in_out_buffer[Prinyato]=SBUF;
   Prinyato=Prinyato+1;
   uart_gotov=1;
   
	/*if(in_out_buffer[0]!=(unsigned char)(0xAA))
		          T2=1;
	if(in_out_buffer[1]!=(unsigned char)(0x07))
		          T2=1;
	if(in_out_buffer[2]!=(unsigned char)(0x1D))
		          T2=1;
	if(in_out_buffer[3]!=(unsigned char)(0x00))
		          T2=1;	*/
	/*if(in_out_buffer[4]!=(unsigned char)(0x00))
		          T2=1;
	if(in_out_buffer[5]!=(unsigned char)(0x00))
		          T2=1;
	if(in_out_buffer[6]!=(unsigned char)(0x00))
		          T2=1;
	if(in_out_buffer[7]!=(unsigned char)(0x4C))
		          T2=1;	*/
	
	 
     /*if((Peredano=Protocol_IMSP(in_out_buffer,Prinyato))!=0) 
     {
	     vivedeno=0;
	     SBUF=in_out_buffer[vivedeno];
	     vivedeno++;
		 
	   }
	 else
	   {
		 vivedeno=0;
		 uart_gotov=0;
	   }   */
   }   	  
 }
}
}
}
#endif
//------------------------------ +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char Protocol_IKM(void *in_out_buffer_temp,unsigned char Prinyato) // В переменную Prinyato нужно передавать количество принятых данных начиная с 1.
{
unsigned char idata i=0;
unsigned char idata j=0;
unsigned char xdata * in_out_buffer=(unsigned char xdata*)in_out_buffer_temp;
unsigned long addr=0;
unsigned long temp_data=0;

if(in_out_buffer[0]==(unsigned char)0xAA) // проверка заголовка кадра
 {	 
    if(CyclicControl(in_out_buffer,Prinyato-1)==in_out_buffer[Prinyato-1])	// проверка контрольной суммы
	    {

		 if(in_out_buffer[1]==(unsigned char)Adress_ustroistva)	// проверка адреса устройства
		  {

          if(in_out_buffer[2]==0x01)	// выполняем действия в зависимости от кода операции
		   {
		     
			            in_out_buffer[0]=0xAA;
						in_out_buffer[1]=(unsigned char)Adress_ustroistva;  // адрес устройства
			            in_out_buffer[2]=0x02;  // код операции 
						in_out_buffer[3]=0x21;	// Длина данных
						in_out_buffer[4]=Nomer_ustroistva&0x00FF;
						in_out_buffer[5]=(Nomer_ustroistva>>8)&0x00FF;

						for(i=0;i<30;i++)					// записываем примечание
						 in_out_buffer[i+6]=Primechanie[i];
						  
						in_out_buffer[36]=CyclicControl(in_out_buffer,36);
						return (unsigned char)36;
			}
			if(in_out_buffer[2]==0x03)	
		    {
			            in_out_buffer[0]=0xAA;
						in_out_buffer[1]=(unsigned char)Adress_ustroistva;  // адрес устройства
			            in_out_buffer[2]=0x04;  // код операции 
						in_out_buffer[3]=0x12;	// Длина данных
					
						in_out_buffer[4]=((unsigned char*)&ustavka)[3];
						in_out_buffer[5]=((unsigned char*)&ustavka)[2];
						in_out_buffer[6]=((unsigned char*)&ustavka)[1];
						in_out_buffer[7]=((unsigned char*)&ustavka)[0];

						in_out_buffer[8]=((unsigned char*)&koef_tar)[3];
						in_out_buffer[9]=((unsigned char*)&koef_tar)[2];
						in_out_buffer[10]=((unsigned char*)&koef_tar)[1];
						in_out_buffer[11]=((unsigned char*)&koef_tar)[0];

						in_out_buffer[12]=(rejim<<4)|mashtab;

						in_out_buffer[13]=((unsigned char*)&moment_mgnovennoe_out_data)[3];
						in_out_buffer[14]=((unsigned char*)&moment_mgnovennoe_out_data)[2];
						in_out_buffer[15]=((unsigned char*)&moment_mgnovennoe_out_data)[1];
						in_out_buffer[16]=((unsigned char*)&moment_mgnovennoe_out_data)[0];

						in_out_buffer[17]=((unsigned char*)&moment_srednee_out_data)[3];
						in_out_buffer[18]=((unsigned char*)&moment_srednee_out_data)[2];
						in_out_buffer[19]=((unsigned char*)&moment_srednee_out_data)[1];
						in_out_buffer[20]=((unsigned char*)&moment_srednee_out_data)[0];

						in_out_buffer[21]=CyclicControl(in_out_buffer,21);
						return (unsigned char)21;
			}
			if(in_out_buffer[2]==0x05)		  // чтение данных из памяти 
		    {
						
						((unsigned char*)&ustavka)[0]=in_out_buffer[7];	 // уставка
						((unsigned char*)&ustavka)[1]=in_out_buffer[6];
						((unsigned char*)&ustavka)[2]=in_out_buffer[5];
						((unsigned char*)&ustavka)[3]=in_out_buffer[4];

						int_ustavka=ustavka*100.0;

					
						((unsigned char*)&koef_tar)[0]=in_out_buffer[11];	 // каллибровка
						((unsigned char*)&koef_tar)[1]=in_out_buffer[10];
						((unsigned char*)&koef_tar)[2]=in_out_buffer[9];
						((unsigned char*)&koef_tar)[3]=in_out_buffer[8];

						int_tar_koef=koef_tar*100.0;

					
						if(int_tar_koef<=100)
				         {
				          if(int_ustavka>=500)
					       {
				            ustavka=5;
				            int_ustavka=500;
					       }
				         }
				        else
				         {
				           if(int_ustavka>=6400)
					        {
				              ustavka=64;
				              int_ustavka=6400;
					        }

				         }
						 
						 if(int_tar_koef<=100)
				          {
				            koef_tar=1;
				            int_tar_koef=100;
				          }

				          if(int_tar_koef<=100)
				            if(int_ustavka>=500)
				             {
					        	ustavka=5;
				                int_ustavka=500;
				             }			
						 
						if ( rejim==5 )
						{
						 perestroit_dec = 1;
						 restart_led_dec_indikator=198;
						 chastota_vivoda_na_indikator=86;
						}
							 			
						rejim=(in_out_buffer[12]>>4);		  // масштаб и режим
						mashtab=(in_out_buffer[12]&0x0F);



						if(rejim!=1&&rejim!=2&&rejim!=3&&rejim!=4)
						   rejim=1;
					    if(mashtab!=1&&mashtab!=2&&mashtab!=4&&mashtab!=8)
						  mashtab=1;

						if(int_tar_koef==100)
                           ves_dioda=(float)(5.0/mashtab)/32.0;
                        else
                           ves_dioda=(float)(64.0/mashtab)/32.0;

					     if(rejim==3)
						   temp_us_tar=int_ustavka;

						 if(rejim==4)
						   temp_us_tar=int_tar_koef;

						 write_flash_pc_us_tar=1;
					        
			            in_out_buffer[0]=0xAA;
						in_out_buffer[1]=(unsigned char)Adress_ustroistva;  // адрес устройства
			            in_out_buffer[2]=0x06;  // код операции 
						in_out_buffer[3]=0x01;	// Длина данных

						in_out_buffer[4]=CyclicControl(in_out_buffer,4);
						beep(10); 
			}			return (unsigned char)4;
		
		}
		else
         return (unsigned char)0;
	  }
		else
         return (unsigned char)0;
 }
 else
 return (unsigned char)0;

}
#endif
//---------------------------------
void main (void)
{
char len=0; // длина строки
bit vivod_dec=0; // значение готово для вывода на циф.инд.
float xdata koef_usl=(float)max_napr_vhod/(float)max_napr_acp; // множитель для получения истинного значения напряжения
float xdata pokazanie_napryajeniya=0; //напряжение на датчике
float xdata ves_srednee=0; //
float xdata ves_mgnovennoe=0; //

long idata koef=1;

unsigned char xdata massiv_znacheniy_cvetov[16]={0,64,8,72,0,2,128,130,0,32,4,36,0,1,16,17}; // массив цветов для шкального индикатора 0-3 -первый диод,4-7 - 2 диод, 8-11 - 3,12-15 - 4. Четыре диода состовляют один байт в протоколе вывода.
float xdata pokazanie_datchika_mgnovennoe=0; // мгновенное значение показания датчика
float xdata pokazanie_datchika_srednee=0;	// усреднённое значение показания датчика
float xdata tmp_pokazanie_datchika_srednee=0;

//float xdata raznica=0;	// вспомогательная переменная для формирования строки вывода на шк.ин., разница между усреднённым и мгновенным значением
// Фильтр+++++
bit first_prohod_adc=1;
bit filtr_ves=0;
bit filtr_napr=0;
bit peregruz=0;
//------------
unsigned int green_led=0; // количество выводимых диодов зелённого цвета(т.е. усреднённого значения показания датчика)
unsigned int orange_led=0;	// количество выводимых диодов оранжевого цвета(т.е. мгновенное значения показания датчика)
unsigned int ustavka_led=0; // номер диода на шк.ин. для уставки
unsigned char i=0; // вспомогательная переменная для перебора

unsigned long pisk_pregruz_vvremya=120; // время через которое сработает beep после перегрузки 300 = 3 секунды

struct Led	   // структура с массивом для хранения определённого цвета для каждого из четырёх диодов
{
unsigned char cvet[4];
};
struct Led xdata led[8];  // всего 32 диода


PLLCON=0x02;
P3_5=0;

CS_DEC=0;	 // выбор кристала
CS_LED=0;
CS_INDIK=0; 

CLK=0;
// КОНФИГ АЦП
ADC0CON1=0x27;	   // 1.28
//ADC0CON2=0x44;	  //  AIN5 -> AINCOM	   опора 1.25
ADC0CON2=0x4;//внутренняя опора
SF=0x16;	  // 186.18 Hz
ADCMODE=0x2B;

#if DEBUG==1
SCON=0x52;	  // Режим 1 - 8 бит, прием запрещен
T3CON=0x80;	//UART = 115200 Baud rate
T3FD=0x2D;
#else
SCON=0x50;	  // Режим 1 - 8 бит, прием запрещен
T3CON=0x81;	//UART = 57600 Baud rate
T3FD=0x2D;
ES=1; //разрешение прер-я от UART
#endif


// КОНФИГ ТАЙМЕР0
TMOD=0x01; //16 - разрядный таймер, внутренние стробирование
//TH0=0x0A;	 // 5-ть миллисекунд
//TL0=0x4F;
period=0xFFFF-(int)(((float)period_oprosa_klavi*0.001)/0.00000031789143880208333333333333333333); // задержка после P2.6=1; 76 расчитано от переключения Р2_6 и до входа в подпрограмму ADC
tim_h_0=TH0=(period&0xFF00)>>8;	 // 20 микросекунд
tim_l_0=TL0=period&0x00FF;

P0=0xFF;   //P0.0 - P0.7 конфиг-ся как цифровые входы
P1=P1&0xC0; //P1.0 - P1.5 конфиг-ся как цифровые входы

DACH=0x00;
DACL=0x00;
DACCON=0x03;

EADC=1;	  // разрешение прерывания для АЦП
//PADC=1;
PS=1; //наивысший приоритет у UART
ET0=1; //разрешение прер-я от Timer0
EA=1; //разрешение всех прерываний
TR0=1; //включение Timer0

delay(10000);
BIP=1;

str_yarkost_dec[0]=0x0A;
flash_read (&str_yarkost_dec[1],sizeof(str_yarkost_dec[1]),0x0000000C);

if(str_yarkost_dec[1]==0xFF)
 str_yarkost_dec[1]=0x0F;

//delay(50000);
LED_OUT_DEC(NULL,NULL,1,"\x0C\x00",1); // включение питания
LED_OUT_DEC(NULL,NULL,1,"\x09\xFF",1); // инициализация decode-on
LED_OUT_DEC(NULL,NULL,1,"\x0B\x04",1); // сканирование 5
LED_OUT_DEC(NULL,NULL,1,str_yarkost_dec,1); // яркость
LED_OUT_DEC("\xFF\xFF\xFF\xFF\xFF",5,0,NULL,1);

LED_OUT_LINE(NULL,1,"\x0C\x00"); // включение питания
LED_OUT_LINE(NULL,1,"\x09\x00"); // инициализация decode-off
LED_OUT_LINE(NULL,1,"\x0B\x07"); // сканирование 8
LED_OUT_LINE(NULL,1,"\x0A\x0A"); // яркость
LED_OUT_LINE("\x00\x00\x00\x00\x00\x00\x00\x00",0,NULL);

LED_OUT_INDIK(NULL,NULL,1,"\x0C\x00"); // включение питания
LED_OUT_INDIK(NULL,NULL,1,"\x09\x00"); // инициализация decode-off
LED_OUT_INDIK(NULL,NULL,1,"\x0B\x07"); // сканирование 5
LED_OUT_INDIK(NULL,NULL,1,"\x0A\x0A"); // яркость
LED_OUT_INDIK("\x00",1,0,NULL);

delay(10000);



flash_read (&int_ustavka,sizeof(int_ustavka),0x00000000);
flash_read (&int_tar_koef,sizeof(int_tar_koef),0x00000004);
flash_read (&mashtab,sizeof(mashtab),0x00000008); 

if(int_ustavka==0xFFFFFFFF)
 int_ustavka=500;

if(int_tar_koef==0xFFFFFFFF)
 int_tar_koef=100;

if(mashtab==0xFF)
 mashtab=1;

koef_tar=int_tar_koef/100.0;
ustavka=int_ustavka/100.0;

verhniy_pridel=5.0;
nijniy_pridel=0.2;
interval=verhniy_pridel-nijniy_pridel;
koef_usileniya_ves=kof_ves/interval;
koef_usileniya_dac=kof_dac/interval;

if(int_tar_koef==100)
  ves_dioda=(float)(5.0/mashtab)/32.0;
else
  ves_dioda=(float)(64.0/mashtab)/32.0;

 while(1)
 {
//----------------------------------------------------------------------------------------------
  if(adc_gotov==kol_izmer_data)  // усреднение и проход через фильтр показания датчика
      {
	    adc_datchik=adc_data/kol_izmer_data;

		if(first_prohod_adc)
	     {
	       adc_filtr_ves[0]=adc_filtr_ves[1]=adc_datchik;
	       first_prohod_adc=0;
	     }

	    adc_filtr_ves[filtr_ves]=adc_filtr_ves[filtr_ves^1]+0.04*(adc_datchik-adc_filtr_ves[filtr_ves^1]);
		adc_filtr_napr[filtr_napr]=adc_filtr_napr[filtr_napr^1]+0.13*(adc_datchik-adc_filtr_napr[filtr_napr^1]);

	    pokazanie_datchika_srednee=(adc_filtr_ves[filtr_ves]*ves_raz)*koef_usl; // усреднённое значение
		pokazanie_napryajeniya=(adc_filtr_napr[filtr_napr]*ves_raz)*koef_usl; // усреднённое значение
	    filtr_ves^=1;
		filtr_napr^=1;
		adc_data=0;
	    adc_gotov=0;
		vivod_dec=1;
		
	  }
   
 //----------------------------------------------------------------------------------------------
   if(pokazanie_napryajeniya>=max_napr_vhod)
   {
	  if(pisk_pregruz_vvremya==0)
	    beep(50);
	  else
	    pisk_pregruz_vvremya--;
   }
   else
     pisk_pregruz_vvremya=120;
 //----------------------------------------------------------------------------------------------
   pokazanie_datchika_mgnovennoe=((float)adc_datchik*ves_raz)*koef_usl; // мгновенное значение

   ves_srednee=((interval-(verhniy_pridel-pokazanie_datchika_srednee))*koef_usileniya_ves);	 // вес среднее в тоннах
   ves_mgnovennoe=((interval-(verhniy_pridel-pokazanie_datchika_mgnovennoe))*koef_usileniya_ves); // вес мгновенное в тоннах


   dac_peremen=(long)float_ceil_floor(((((interval-(verhniy_pridel-pokazanie_napryajeniya))*koef_usileniya_dac)+(float)min_dac)/ves_raz_dac));   
   if(dac_peremen>(((float)max_dac/ves_raz_dac)))
     dac_peremen=(((float)max_dac/ves_raz_dac));

   if(dac_peremen<(((float)min_dac/ves_raz_dac)))
     dac_peremen=(((float)min_dac/ves_raz_dac));

   DACH=(dac_peremen&0x00000F00)>>8;
   DACL=(dac_peremen&0x000000FF); 
   
   if(ves_srednee < 0 || ves_mgnovennoe < 0)
     ves_srednee=ves_mgnovennoe=0;

   if(ves_srednee >= kof_ves || ves_mgnovennoe >= kof_ves)
     ves_srednee=ves_mgnovennoe=kof_ves;

   if(koef_tar <= 1)
     koef_tar=1;

   moment_srednee_out_data=moment_srednee=ves_srednee*koef_tar;
   moment_mgnovennoe_out_data=moment_mgnovennoe=ves_mgnovennoe*koef_tar;

   if(moment_srednee >= 99.99 || moment_mgnovennoe >= 99.99)
     moment_srednee=moment_mgnovennoe=99.99;

//----------------------------------------------------------------------------------------------	

   green_led=(moment_srednee/ves_dioda);	// рассчёт количество выводимых диодов зелённого цвета
   orange_led=(moment_mgnovennoe/ves_dioda); // рассчёт количество выводимых диодов оранжевого цвета
   ustavka_led=(ustavka/ves_dioda);  // рассчёт позиции уставки

   if(ustavka_led==0)	// если уставка равна нуль всё равно зажигаем первый светодиод
     ustavka_led=1;

   if(green_led==0)	// если уставка равна нуль всё равно зажигаем первый светодиод
     green_led=1;
	 	  
      if(green_led>=ustavka_led)  // проверка на перегрузку, т.е. переход за уставку
        peregruz=1;
      else
	    peregruz=0;

      /*if(green_led<orange_led)	// рассчёт количества оранжевых светодиодов после зелёных
        raznica=orange_led-green_led;
      else
        raznica=0;*/

      for(i=0;i<=31;i++)  // формирование зелёной линейки(т.е. усреднённого значения) в str_green
       {
         if(i<green_led)
          {
  
            if(peregruz==0)
               led[(i)/4].cvet[i%4]=0x1;
            else
               led[(i)/4].cvet[i%4]=i!=ustavka_led-1?0x2:0x1;
          }
          else
            if(i>=green_led&&i<orange_led)
               led[(i)/4].cvet[i%4]=i!=ustavka_led-1?0x3:0x2;
            else
              if(i==ustavka_led-1)
	             led[(i)/4].cvet[i%4]=0x2;
	          else
	             led[(i)/4].cvet[i%4]=0;

         str_led[(i)/4]+=massiv_znacheniy_cvetov[led[(i)/4].cvet[i%4]+((i%4)*4)];

       }
 //----------------------------------------------------------------------------------------------
   if(out_port==1)
   {
    
 //----------------------------------------------------------------------------------------------
    switch(klava)
	   {
	    case 24:
	    case  6: 
		 		 beep(10);
		         if(rejim==3)	 // если предыдущий режим был ввод уставки значит запоминаем новое значение уставки
				 {
				  int_ustavka=temp_us_tar;
				  ustavka=(float)int_ustavka/100.0;
				  
				  if(int_tar_koef<=100)
				  {
				    if(int_ustavka>=500)
					{
				      ustavka=5;
				      int_ustavka=500;
					}
				  }
				  else
				  {
				    if(int_ustavka>=6400)
					{
				      ustavka=64;
				      int_ustavka=6400;
					}

				  }
				  
				  flash_write (&int_ustavka,sizeof(int_ustavka),0x00000000);
				  if(vhod_v_edit)
				    rejim=2;
				  vhod_v_edit=0;
				 }
				 if(rejim==4)	// если предыдущий режим был ввод коэфф. значит запоминаем новое значение коэфф.
				 {
				  int_tar_koef=temp_us_tar;
				  
				  if(temp_us_tar<=100)
				  {
				   koef_tar=1;
				   int_tar_koef=100;
				   temp_us_tar=100;
				  }

				  /*if(koef_tar <= 0)
				  {
                     koef_tar=1;
					 int_tar_koef=100;
				  }	*/

				  if(int_tar_koef<=100)
				  if(int_ustavka>=500)
				   {
						ustavka=5;
				        int_ustavka=500;
				   }

				  koef_tar=(float)int_tar_koef/100.0;
				  /*if(int_tar_koef>=1280)
					{
					  koef_tar=12.8;
					  int_tar_koef=1280;
					}	*/
				  flash_write (&int_tar_koef,sizeof(int_tar_koef),0x00000004);

				  if(vhod_v_edit)
				    rejim=3;
				  vhod_v_edit=0;
				 }

				 if(rejim==5)  // если предыдущий режим был регулировка яркости (режим=5) то включаем декодер на цифровом индикаторе
				 {
				     flash_write (&str_yarkost_dec[1],sizeof(str_yarkost_dec[1]),0x0000000C);
				     LED_OUT_LINE("\x00\x00\x00\x00\x00\x00\x00\x00",0,NULL);
					 LED_OUT_DEC(NULL,NULL,1,"\x0C\x00",0); // выключение питания
				     LED_OUT_DEC("\xFF\xFF\xFF\xFF\xFF",5,0,NULL,1); // обнуляем индикаторы
				   	 LED_OUT_DEC(NULL,NULL,1,"\x09\xFF",1); // инициализация decode-on
                     LED_OUT_DEC(NULL,NULL,1,"\x0B\x04",1); // сканирование 5
                     LED_OUT_DEC(NULL,NULL,1,str_yarkost_dec,1); // яркость
                     LED_OUT_DEC(NULL,NULL,1,"\x0C\x01",1); // включение питания
					
				 }

		          rejim+=1;	// перключение режимов
		         if(rejim>=6)
				   rejim=1;
		         if(rejim==1||rejim==2)
				  {
				   start_miganie=0;
				   miganie=0;
				   _miganie_=1; 
				  }
				  else
				  {
				   start_miganie=1;
				   vhod_v_edit=0;
				   nomer_cifri=0;
				   temp_us_tar=rejim==3?int_ustavka:int_tar_koef;
				   perevod_FloatToString(0,temp_us_tar,str_dec,2,1,1);
				   cifra=str_dec[0]&0x0F;
				   _miganie_=1;
				   miganie=0;
				  }

				  if(rejim==5) // регулировка яркости цифрового индикатора
                   { 
					 LED_OUT_DEC(NULL,NULL,1,"\x0C\x00",1); // выключение питания
                     LED_OUT_DEC(NULL,NULL,1,"\x09\x00",1); // инициализация decode-off
					 LED_OUT_DEC("\x00\x00\x00\x00\x00",5,0,NULL,0);
                     LED_OUT_DEC(NULL,NULL,1,"\x0B\x04",1); // сканирование 5
                     LED_OUT_DEC(NULL,NULL,1,str_yarkost_dec,1); // яркость
                     LED_OUT_DEC(NULL,NULL,1,"\x0C\x01",1); // включение питания
					 
                     LED_OUT_DEC("\x67\x4F\x46",3,0,NULL,0);

					 for (i=0;i<8;i++)
					  if(i<((str_yarkost_dec[1]/2)+1))
					   str_led_procent_yarkosti_dec[i]=0x63;
					  else
					   str_led_procent_yarkosti_dec[i]=0x00;

					 LED_OUT_LINE(str_led_procent_yarkosti_dec,0,NULL);
					 
                   }

				   restart_led_dec_indikator=0;
				   chastota_vivoda_na_indikator=86;
	             break;

		case 26:
		case 1:  if(rejim==3||rejim==4)
		          {
				   beep(10);
				   if(!vhod_v_edit)
				    {
					 vhod_v_edit=1;
					 nomer_cifri=-1;
					}
		           if(nomer_cifri<3)
		           {
                      nomer_cifri++;
					  perevod_FloatToString(0,temp_us_tar,str_dec,2,1,1);
					  if(nomer_cifri<=1)
					    cifra=str_dec[nomer_cifri]&0x0F;
					  else
					    cifra=str_dec[nomer_cifri+1]&0x0F;

				   }
				  else
				  {
				   nomer_cifri=0;
				   perevod_FloatToString(0,temp_us_tar,str_dec,2,1,1);
				   cifra=str_dec[nomer_cifri]&0x0F;
				  }
				  start_miganie=1;
				  _miganie_=1;
				  miganie=0;
				 }     
		          break;
		case 28:		  
		case 7: 
		         if(rejim==3||rejim==4)			// перебор 0...9
				 {
				 if(vhod_v_edit)
				  {	
				  beep(10);
		          if(cifra<9)	 // перебор от 1...9
				     {
		              koef=1;
					  for(i=0;i<nomer_cifri;i++)
					  koef*=10;
					  temp_us_tar+=1000/koef;

                      if(rejim==3) //если режим ввода уставки
					  {
						/* if(int_tar_koef==100) // если тар.коэф. 1
						 {
						  if(temp_us_tar>500) // то тогда значение уставки не может быть больше 5
						    {
							  temp_us_tar-=(cifra+1)*(1000/koef);
							  cifra=-1;
							}
						 }
						 else  // если тар.коэф. >1
						 {
						   if(temp_us_tar>6400)	// то тогда значение уставки не может быть больше 64
						    {
							  temp_us_tar-=(cifra+1)*(1000/koef);
							  cifra=-1;
							}

						 }*/
					  }
					  else	  //если режим ввода тар. коэф.
					  {
						/*if(temp_us_tar>1280)
						    {
							  temp_us_tar-=(cifra+1)*(1000/koef);
							  cifra=-1;
							}*/
						 if(temp_us_tar<=100)
                            ves_dioda=(float)(5.0/mashtab)/32.0;
                         else
	                        ves_dioda=(float)(64.0/mashtab)/32.0;

					  }

					  if(rejim==3)
					     ustavka=(float)temp_us_tar/100.0;
					  else
					     koef_tar=(float)temp_us_tar/100.0;
				      cifra++;
				     }
				   else	 // перебор 0
				    {
				     cifra=0;
		             koef=1;
					 for(i=0;i<nomer_cifri;i++)
					 koef*=10;
		             temp_us_tar-=9*(1000/koef);

					 if(rejim==3) // если режим уставки, уставка может быть 0
					     ustavka=(float)temp_us_tar/100.0;
					  else	  // если режим тарировки
					  {
					     if(temp_us_tar<=100)  // если коэф.тар. 1, то тогда диапозон шкального индик. от 0 до 5
                            ves_dioda=(float)(5.0/mashtab)/32.0;	 // рассчёт веса одного диода линейки
                         else	 // если коэф.тар. >1, то тогда диапозон шкального индик. от 0 до 64
	                        ves_dioda=(float)(64.0/mashtab)/32.0;
					     koef_tar=(float)temp_us_tar/100.0;
					  }
				    }
				   start_miganie=1;
				   _miganie_=0;
				   miganie=0;
				   }
				  }

				 if(rejim==5)
				 {
				   beep(10);

				   if(str_yarkost_dec[1]==0x0F)
				     str_yarkost_dec[1]=1;
				   else
				     str_yarkost_dec[1]+=2;

				   for (i=0;i<8;i++)
					 if(i<((str_yarkost_dec[1]/2)+1))
					  str_led_procent_yarkosti_dec[i]=0x63;
					 else
					  str_led_procent_yarkosti_dec[i]=0x00;

				   LED_OUT_LINE(str_led_procent_yarkosti_dec,0,NULL);

				   LED_OUT_DEC(NULL,NULL,1,str_yarkost_dec,1); // яркость

				 }
		            break;
		case 25:		  
		case  0:
		        if(rejim!=5)
				{
		         if((rejim==3||rejim==4)&&vhod_v_edit)
				  {				  
				   beep(10);
				   temp_us_tar=rejim==3?int_ustavka:int_tar_koef;
				   if(rejim==3)
					   ustavka=(float)temp_us_tar/100.0;
				   else
					   koef_tar=(float)temp_us_tar/100.0;
				   vhod_v_edit=0;
				  }
				 else
				  {
				      beep(10);
				      switch(mashtab)
					  {
					   case 1: mashtab=2;
					           break;
					   case 2: mashtab=4;
					           break;
					   case 4: mashtab=8;
					           break;
					   case 8: mashtab=1;
					           break;
					  }

					  
				  }

				 if(int_tar_koef==100)
                   ves_dioda=(float)(5.0/mashtab)/32.0;
                 else
	               ves_dioda=(float)(64.0/mashtab)/32.0;
				}

		        break;


	   }
	
	out_port=0;
  }
 //----------------------------------------------------------------------------------------------
 if(restart_led_dec_indikator>197)
 {
   LED_OUT_LINE(NULL,1,"\x09\x00"); // инициализация decode-off
   LED_OUT_LINE(NULL,1,"\x0B\x07"); // сканирование 8
   LED_OUT_LINE(NULL,1,"\x0A\x0A"); // яркость
   LED_OUT_LINE(NULL,1,"\x0C\x01"); // включение питания

   if (rejim!=5 && perestroit_dec == 0)
   {
   LED_OUT_DEC(NULL,NULL,1,"\x09\xFF",1); // инициализация decode-on
   LED_OUT_DEC(NULL,NULL,1,"\x0B\x04",1); // сканирование 5
   LED_OUT_DEC(NULL,NULL,1,str_yarkost_dec,1); // яркость
   LED_OUT_DEC(NULL,NULL,1,"\x0C\x01",1); // включение питания
   }
   else
   if ( perestroit_dec == 1 )
   {
	LED_OUT_LINE("\x00\x00\x00\x00\x00\x00\x00\x00",0,NULL);
	LED_OUT_DEC(NULL,NULL,1,"\x0C\x00",0); // выключение питания
	LED_OUT_DEC("\xFF\xFF\xFF\xFF\xFF",5,0,NULL,1); // обнуляем индикаторы
	LED_OUT_DEC(NULL,NULL,1,"\x09\xFF",1); // инициализация decode-on
    LED_OUT_DEC(NULL,NULL,1,"\x0B\x04",1); // сканирование 5
    LED_OUT_DEC(NULL,NULL,1,str_yarkost_dec,1); // яркость
    LED_OUT_DEC(NULL,NULL,1,"\x0C\x01",1); // включение питания
	perestroit_dec = 0;

   }

   LED_OUT_INDIK(NULL,NULL,1,"\x09\x00"); // инициализация decode-off
   LED_OUT_INDIK(NULL,NULL,1,"\x0B\x07"); // сканирование 5
   LED_OUT_INDIK(NULL,NULL,1,"\x0A\x0A"); // яркость
   LED_OUT_INDIK(NULL,NULL,1,"\x0C\x01"); // включение питания

   restart_led_dec_indikator=0;
 } 
 //----------------------------------------------------------------------------------------------
 Indikatornaya_panel(rejim,mashtab); 

 if(rejim==1)  // измерение момента
 {

  if(vivod_dec&&chastota_vivoda_na_indikator>85)
  {
   tmp_pokazanie_datchika_srednee=moment_srednee;
   moment_srednee*=1000.0;
   moment_srednee=float_ceil_floor(moment_srednee);
   moment_srednee/=1000.0;
   perevod_FloatToString(moment_srednee,0,str_dec,2,0,0); // перевод усреднённого значения показания датчика
   len=strlench(str_dec);			  // посчёт длины строки
   LED_OUT_DEC(str_dec,len,0,NULL,1); // вывод на цифровой индикатор
   moment_srednee=tmp_pokazanie_datchika_srednee;
   vivod_dec=0;
   chastota_vivoda_na_indikator=0;
  }
  LED_OUT_LINE(str_led,0,NULL);  // вывод на шкальный индикатор
 }
//----------------------------------------------------------------------------------------------
 if(rejim==2) // отображение напряжения
 {
  if(vivod_dec&&chastota_vivoda_na_indikator>85)
  {
    tmp_pokazanie_datchika_srednee=pokazanie_napryajeniya;
//------------------------
	if(pokazanie_napryajeniya>=max_napr_vhod)
	{
		pokazanie_napryajeniya=max_napr_vhod;
	}
//------------------------

    pokazanie_napryajeniya*=10000.0;
    pokazanie_napryajeniya=float_ceil_floor(pokazanie_napryajeniya);
    pokazanie_napryajeniya/=10000.0;
    perevod_FloatToString(pokazanie_napryajeniya,0,str_dec,3,0,0);
	pokazanie_napryajeniya=tmp_pokazanie_datchika_srednee;
   
	 len=strlench(str_dec);			  // посчёт длины строки
     LED_OUT_DEC(str_dec,len,0,NULL,1); // вывод на цифровой индикатор
	 chastota_vivoda_na_indikator=0;
	 vivod_dec=0;
  }
  LED_OUT_LINE(str_led,0,NULL); 
 }
//----------------------------------------------------------------------------------------------
 if(rejim==3) // задание уставки
 {
  if(vhod_v_edit)
   {
    perevod_FloatToString(0,temp_us_tar,str_dec,2,1,1);
    if(_miganie_)
     {
	   if(nomer_cifri<=1)
	    str_dec[nomer_cifri]=0x0F;
	   else
	    str_dec[nomer_cifri+1]=0x0F;	  
     }
	len=5;
   }
   else
   {
    perevod_FloatToString(0,int_ustavka,str_dec,2,0,1);
	len=strlench(str_dec);
   }
    LED_OUT_DEC(str_dec,len,0,NULL,1); // вывод на цифровой индикатор
	LED_OUT_LINE(str_led,0,NULL);  // вывод на шкальный индикатор

 }
//----------------------------------------------------------------------------------------------
 if(rejim==4) // ввод тарировочного коэффициента
 {
  if(vhod_v_edit)
   {
    perevod_FloatToString(0,temp_us_tar,str_dec,2,1,1);
    if(_miganie_)
     {
	   if(nomer_cifri<=1)
	    str_dec[nomer_cifri]=0x0F;
	   else
	    str_dec[nomer_cifri+1]=0x0F;	  
     }
	len=5;
   }
   else
   {
    perevod_FloatToString(0,int_tar_koef,str_dec,2,0,1);
	len=strlench(str_dec);
   }
    LED_OUT_DEC(str_dec,len,0,NULL,1); // вывод на цифровой индикатор
	LED_OUT_LINE(str_led,0,NULL);  // вывод на шкальный индикатор

 }


   for(i=0;i<=8;i++)   // обнуление строки после вывода на шкальный индикатор
       str_led[i]=0x00;
   

   #if DEBUG==1	   // режим для отладки
	if(kol_vivod>25)
	{
    //printf("\rACP_sr: %.5f",pokazanie_datchika_srednee);
	printf("\r DAC: %.5f",dac_peremen*ves_raz_dac);
	//printf(" D: %d  ",(int)(0xFFFF&klava));
	//printf("%.5f",pokazanie_datchika_srednee);
	kol_vivod=0;
	}
   #else
	if(uart_gotov==1)
    { 	   
	 uart_gotov=2;
   
      if((Peredano=Protocol_IKM(in_out_buffer,Prinyato))!=0) 
       {
	     vivedeno=0;
		 P3_5=1;
	     SBUF=in_out_buffer[vivedeno];
	     vivedeno++;
		 
	   }
	 else
	   {
		 vivedeno=0;
		 uart_gotov=0;
	   } 
   }
   
   if(write_flash_pc_us_tar==1)
   {
   	 flash_write (&int_ustavka,sizeof(int_ustavka),0x00000000);
	 flash_write (&int_tar_koef,sizeof(int_tar_koef),0x00000004);
   	 write_flash_pc_us_tar=0;
   }
   #endif


    
 }

}
