/** WarTunnel Board Rev 4.0
* Warthog Robotics
* VITOR HUGO DE C GOMES
* DIVISAO ELETRONICA E POTENCIA
* 14/12/2017
* University of Sao Paulo at Sao Carlos
* http://www.warthog.sc.usp.br
* This file is part of the Warthog Robotics WarTunnel project
**/

/** Copyright (C) 2017 Warthog Robotics
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
**/

// BIBLIOTECAS

#include <xc.h>
#include <p18f4550.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


// CONFIGURACAO REGISTRADORES

#pragma config FOSC = INTOSC_EC //internal oscillator 4MHz
#pragma config PLLDIV = 1 //divide PLL
#pragma config FCMEN = OFF //no fail safe clock monitor
#pragma config IESO = ON //oscillator switchover disabled
#pragma config VREGEN = OFF
#pragma config PWRT = OFF //oscillator power up timer enabled (release version only)
#pragma config BOR = OFF //hardware brown out reset
#pragma config WDT = OFF //watchdog timer disabled
#pragma config MCLRE = OFF //MCLR pin enabled
//#pragma config LPT1OSC = ON //timer1 low power operation
#pragma config PBADEN = OFF //portB 0to 4 digital - not analogue
#pragma config LVP = OFF //low voltage programming disabled
#pragma config STVREN = ON //Stack full/underflow will not cause Reset.
#pragma config CCP2MX = OFF //CCP2 input/output is multiplexed with RB3.

// DEFINES

//I/O - FOTOTRANSISTORES
#define FTI4 PORTCbits.RC7
#define FTI3 PORTDbits.RD4
#define FTI2 PORTDbits.RD5
#define FTI1 PORTDbits.RD6

// FLAGS
#define FLAG_T0 INTCONbits.TMR0IF
#define FLAG_T1 PIR1bits.TMR1IF
#define FLAG_T3 PIR2bits.TMR3IF

#define FLAG_INT0 INTCONbits.INT0IF
#define FLAG_INT1 INTCON3bits.INT1IF
#define FLAG_INT2 INTCON3bits.INT2IF

// LCD
#define RW LATCbits.LATC1
#define RS LATCbits.LATC2
#define E LATCbits.LATC0
#define D7 LATAbits.LATA1
#define D6 LATAbits.LATA2
#define D5 LATAbits.LATA3
#define D4 LATAbits.LATA4
#define D3 LATAbits.LATA5
#define D2 LATEbits.LATE0
#define D1 LATEbits.LATE1
#define D0 LATEbits.LATE2

// DELAY
#define _XTAL_FREQ 1000000 //FOSC frequency used
#define DELAY_S __delay_us(100)
#define DELAY_L __delay_ms(5)

#define COMP_FT 1000000; // Comprimento do intervalo entre fotos equivalente a 0.01m

// VARIAVEIS GLOBAIS

unsigned long teste_long; //teste para variavel teste unsigned long

unsigned int fti1_flag=0; // flags para saber se a bola já passou pelo primeiro ao ultimo fototransistor
unsigned int fti2_flag=0; 
unsigned int fti3_flag=0;
unsigned int fti4_flag=0;

unsigned int error_flag=0; // Flag que indica se está ou não em erro!

unsigned long comp_ft = COMP_FT; //realiza um cast

unsigned long time_data[5]; // vetor para armazenar os valores dos intervalos de tempo
unsigned long vel_data[4]; // vetor para armazenar os valores dos intervalos de velocidade
unsigned long acel_data[3]; // vetor para armazenar os valores dos intervalos de aceleracao
unsigned long time_aux=0; //aux para calculo do intervalo de tempo

unsigned long cont_global=0; // contador de quantas vezes há estouro do timer0
unsigned int cont_ft=0; //contador fototransistores

unsigned long num=0;

unsigned long velocidade_med = 0; //velocidade media
unsigned long vel_tot = 0; //velocidade total
unsigned long acel_tot = 0; //aceleracao total
unsigned long tempo_med = 0; //tempo medio
unsigned long aceleracao_med = 0; //aceleracao media

// PROTOTIPOS DE FUNCOES

void interrupt inter(); // Função da interrupção

void tmr_int();
void int0_int();
void int1_int();
void int2_int();

void pulse(); //Função Pulso
void clear_scr(); //Função para limpar a tela do display LCD
void clear_data(); // Função para limpar os valores dos dados
void clear_cont(); //Função escrota de algum problema zuado para zerar o contador global timer0
void config_io(); // Função de configuração de portas I/O
void config_lcd(); // Função configuração do display LCD
void config_tmr(); // Função configuração dos timer e suas interrupçoes
void config_int(); // Função configuração interrupções externas
void config_osc(); // Função configuração interrupções externas
void set_inst (unsigned char a,unsigned char b,unsigned char c,unsigned char d,unsigned char e,unsigned char f,unsigned char g,unsigned char h); //Função setar INSTRUÇÃO
void set_data (unsigned char data); //Função escrever DADOS
void write_string(unsigned char string[]); //Função escrever String
void data_print(unsigned long data); //Função escreve valores calculados - máximo são 7 digitos xxx,xxxx
void jump_line(); // Função Pula Linha
void delay_sec(float sec); // Delay de "sec" segundos
void msg_init(); // Mensagem de Inicialização
void func_error(); // Função de ERRO
void acq_data(); // Função para aquisição dos valores dos intervalos de tempo
void acq_print(); // Função para impressão de mensagem informativa de aquisição de dados
void data_tmr(unsigned int ftp_n); // Função retorna valor do timer/contador
void vel_med(); // Funçao calculo velocidade media
void acel_med(); // Funçao calculo aceleração media
void vel_print(unsigned long data); // Função para impressão do valor e informações velocidade media
void cronometro(); // Função cronômetro
void test_ms(); // Função para teste de tempo
void time_print(unsigned long data);// Função para impressão do valor e informações tempo medio
void acel_print(unsigned long data);//Função para impressão do valor e informações aceleracao media

// FUNÇÕES
  
void interrupt inter(){
    if (T0IF) {
        num++;
        T0IF=0;
    }
}

void config_osc(){
    OSCCONbits.IRCF0=0;
    OSCCONbits.IRCF1=0; //1MHz = XTAL_FREQ
    OSCCONbits.IRCF2=1;
    OSCCONbits.IOFS=1; //INTOSC frequency is stable
    OSCCONbits.SCS1=1; //Internal oscillator
}

void config_io(){
    //1 - INPUT / 0 - OUTPUT
    TRISCbits.TRISC0 = 0; //E saída
    TRISCbits.TRISC1 = 0; //RW saída
    TRISCbits.TRISC2 = 0; //RS saída
    //DADOS -> D0-D7 saídas
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA3 = 0;
    TRISAbits.TRISA4 = 0;
    TRISAbits.TRISA5 = 0;
    TRISEbits.TRISE0 = 0;
    TRISEbits.TRISE1 = 0;
    TRISEbits.TRISE2 = 0;

    RW=0;
    RS=0;
    E=0;
    D7=0;
    D6=0;
    D5=0;
    D4=0;
    D3=0;
    D2=0;
    D1=0;
    D0=0;
}

void config_int(){

    RCONbits.IPEN = 1; //Enable priority levels on interrupts

    INTCONbits.RBIE = 0; //Disables the RB port change interrupt
    INTCON2bits.NOT_RBPU = 1; //All PORTB pull-ups are disabled

    INTCONbits.GIE_GIEH = 1; //Enables all high-priority interrupts
    INTCONbits.PEIE_GIEL = 1; //Enables all low-priority peripheral interrupts

    INTCONbits.INT0IE = 0; //disEnables the INT0 external interrupt
    INTCON3bits.INT1IE = 0; //disEnables the INT1 external interrupt
    INTCON3bits.INT2IE = 0; //disEnables the INT2 external interrupt

    INTCON2bits.INTEDG0 = 0; //Interrupt on falling edge
    INTCON2bits.INTEDG1 = 0; //Interrupt on falling edge
    INTCON2bits.INTEDG2 = 1; //Interrupt on rising edge

    INTCON3bits.INT1IP = 1; //High priority
    INTCON3bits.INT2IP = 1; //High priority
    INTCON2bits.RBIP = 1; //High priority
    
    PIE1 = 0x00; //disables all peripheral1 interrupts
    PIE2 = 0x00; //disables all peripheral2 interrupts
}

void config_tmr(){
    // CONFIGURAÇÃO TIMER0
    T0CONbits.TMR0ON = 0; //timer0 module enabled
    T0CONbits.T08BIT = 0; //16-bit timer
    T0CONbits.T0CS = 0; //temporizador - clock source - Internal instruction cycle clock (CLKO)
    T0CONbits.T0SE = 1; //Increment on high-to-low transition on T0CKI pin

    T0CONbits.PSA = 1; //Timer0 prescaler is NOT assigned. Timer0 clock input DOESN'T come from prescaler output.
    T0CONbits.T0PS2 = 0;
    T0CONbits.T0PS1 = 1; //1:16 -> 62,5kHz
    T0CONbits.T0PS0 = 1;

    TMR0L = 0; // Zera bits menos sign timer 0
    TMR0H = 0; // Zera bits mais sign timer 0

    // CONFIGURAÇÃO INTERRUPÇÃO TIMERS
    INTCONbits.TMR0IE = 1; //Enables the TMR0 overflow interrupt
    PIE1bits.TMR1IE = 0; //Disables the TMR1 overflow interrupt
    PIE1bits.TMR2IE = 0; //Disables the TMR2 to PR2 match interrupt
    PIE2bits.TMR3IE = 0; //Disables the TMR3 overflow interrupt

    INTCON2bits.TMR0IP = 1; //High priority
    IPR1bits.TMR1IP = 0; //Low priority
    IPR2bits.TMR3IP = 0; //Low priority
}

void pulse(){
    E = 1;
    DELAY_S ;
    E = 0 ;
    DELAY_S ;
}

void set_inst(unsigned char a,unsigned char b,unsigned char c,unsigned char d,unsigned char e,unsigned char f,unsigned char g,unsigned char h) {
    RS = 0; //instrucao
    DELAY_S;
    D7=a;
    D6=b;
    D5=c;
    D4=d;             
    D3=e;
    D2=f;
    D1=g;
    D0=h;
    DELAY_L;
    pulse();
}

void config_lcd(){  
    set_inst(0,0,1,1,1,0,0,0); //8bit, 2 linhas, fonte 5*8
    set_inst(0,0,0,0,1,1,0,0); //liga LCD, cursor ligado, pisca ligado
    set_inst(0,0,0,0,0,1,1,0); //AC adresss 0 ddram
    //D7=0;
    //D6=0;
    //D5=0;
    //D4=0;
    //D3=0;
    //D2=0;
    //D1=0;
    //D0=0;
    __delay_ms(100);
    DELAY_L;
}

void set_data(unsigned char data) {
    RS = 1 ; //dados
    unsigned char v[8], i;
    for(i = 0; i < 8; ++i) {
        v[i] = (unsigned char) data % 2;
        data /= 2;
    }
    DELAY_S;
    D0 = v[0];
    D1 = v[1];
    D2 = v[2];
    D3 = v[3];
    D4 = v[4];
    D5 = v[5];
    D6 = v[6];
    D7 = v[7];
    DELAY_L;
    pulse();
}

void write_string(unsigned char string[]){
    int i;
    for (i=0; string[i]!='\0'; i++)
        set_data(string[i]);
}

void jump_line(){
    set_inst(1,1,0,0,0,0,0,0);
}

void clear_scr(){
    set_inst(0,0,0,0,0,0,0,1);
    set_inst(1,0,0,0,0,0,0,0);
    D7=0;
    D6=0;
    D5=0;
    D4=0;
    D3=0;
    D2=0;
    D1=0;
    D0=0;
}

void clear_data(){
    int i;
    static unsigned long cont_aux=0;
    for(i=0;i<6;i++){
        time_data[i]=0; // Para limpar os valores do vetor time_data
    }
    for(i=0;i<5;i++){
        vel_data[i]=0; // Para limpar os valores do vetor vel_data
    }
    for(i=0;i<4;i++){
        acel_data[i]=0; // Para limpar os valores do vetor acel_data
    }
    TMR0L=0;
    TMR0H=0;
    num=0;
    fti1_flag=0; // flags para saber se a bola já passou por fti1 a fti4
    fti2_flag=0; 
    fti3_flag=0;
    fti4_flag=0;
    error_flag=0; // Flag que indica se está ou não em erro!
    time_aux=0; //aux para calculo do intervalo de tempo
    cont_ft=0;
    cont_global=cont_aux;
    velocidade_med=0; //velocidade media
    aceleracao_med=0; //aceleracao media
    tempo_med = 0;
    vel_tot = 0;
    acel_tot = 0;
    }

void delay_sec(float sec){
    int i,j;
    j=sec;
    for (i=0; i<10*sec; i++)
    __delay_ms(100);
}

void msg_init(){
    clear_scr();
    write_string(" WARTHOG ROBOTICS");
    jump_line();
    write_string(" EESC - USP");
    delay_sec(2);
    clear_scr();
    write_string(" WRTUNNEL 4.0");
    jump_line();
    write_string(" PROTOTIPO");
    delay_sec(2);
    clear_scr();
    write_string("Configurando");
    delay_sec(1);
    for(int i=0;i<6;i++) {   
        write_string(".");
        delay_sec(0.25);
        if(i==2){
            clear_scr();
        write_string("Configurando");
        }
    }
}

void print_number(unsigned long x) {
    char v[20];
    int cnt = 0;
    while(x!= 0) {
        v[cnt++] = x%10 + '0';
        x/=10;
    }
    
    int beg = 0, end = cnt-1;
    while(end >= beg) {
        char t = v[end];
        v[end] = v[beg];
        v[beg] = t;
        end--;
        beg++;
    }
    v[cnt] = '\0';
    write_string(v);
    
    
    
}

void data_print(unsigned long data){

    unsigned long data00=0,data0=0,data1=0,data2=0,data3=0,data4=0,data5=0,data6=0,data7=0,data8=0,data9=0,data10=0;
    data00 = data/100000000;
    if (data00 != 0){
        set_data(data00+'0');
    }
    data0 = (data%100000000)/10000000;
    if (data0 != 0){
        set_data(data0+'0');
    }
    data1 = (data%10000000)/1000000;
    set_data(data1+'0');
    set_data('.');
    data2 = (data%1000000)/100000;
    set_data(data2+'0');
    data3 = (data%100000)/10000;
    set_data(data3+'0');
    data4 = (data%10000)/1000;
    set_data(data4+'0');
    data5 = (data%1000)/100;
    set_data(data5+'0');
    data6 = (data%100)/10;
    set_data(data6+'0');
    data7 = data%10;
    set_data(data7+'0');
    
} 
   
void vel_med(){
    int i;
    for (i=1;i<4;i++) {
        if (i == 3) {
            vel_data[i] = 100000000000/(float)time_data[i+1];
            vel_data[i] = 200*vel_data[i];
        }
        else {
            vel_data[i] = 100000000000/(float)time_data[i+1];
            vel_data[i] = 100*vel_data[i];
        }
    }
    for (i=1;i<4;i++) {
        vel_tot = vel_tot + vel_data[i];
    }
    velocidade_med = vel_tot/3;
}

void acel_med(){
    int i;
    for (i=1;i<3;i++) {
        acel_data[i] = vel_data[i]-vel_data[i+1];
        acel_data[i] = 1000000*acel_data[i];
        acel_data[i] = acel_data[i]/(float)time_data[i+2];
        acel_data[i] = 100*acel_data[i];
    }
    for (i=1;i<3;i++) {
        acel_tot = acel_tot + acel_data[i];
    }
    aceleracao_med = acel_tot/2;
}

void data_tmr(unsigned int ftp) {
    time_aux = (unsigned long)TMR0H*256 + (unsigned long)TMR0L;
    time_aux = time_aux + 65536*num;
    time_aux = (time_aux)*4;
    
    time_data[cont_ft] = time_aux;
    
    if (cont_ft == 4) {
        vel_med();
        acel_med();
        
        write_string("VEL: ");
        data_print(velocidade_med);
        jump_line();
        write_string("ACEL: -");
        data_print(time_data[2]);
        //data_print(aceleracao_med);
    }
}

void acq_data() {
    time_data[1] = 0;
    cont_ft++;
    T0CONbits.TMR0ON = 1;
    while(FTI2 == 0);
    T0CONbits.TMR0ON = 0; // para timer 0
    //clear_scr();
    //write_string(" FAZENDO LEITURA.");
    cont_ft++;
    data_tmr(0);
    
    TMR0 = 0;
    T0CONbits.TMR0ON = 1;
    while(FTI3 == 0);
    T0CONbits.TMR0ON = 0; // para timer 0
    //clear_scr();
    //write_string(" FAZENDO LEITURA..");
    cont_ft++;
    data_tmr(0);
    
    TMR0 = 0;
    T0CONbits.TMR0ON = 1;
    while(FTI4 == 0);
    T0CONbits.TMR0ON = 0; // para timer 0
    //clear_scr();
    //write_string(" FAZENDO LEITURA...");
    cont_ft++;
    __delay_ms(500);
    //clear_scr();
    data_tmr(0);
    
}

void main() {

    clear_scr();
    config_osc();
    config_io();  //Configura o I/O
    config_int(); // Configura o External Interrupt
    config_tmr(); // Configura o Timer Interrupt
    config_lcd(); // Configura o LCD
    
    clear_scr();
    clear_data();
    
    msg_init();
    
    clear_scr();
    
    while(1) {
        
    clear_data();
    write_string("WRTUNNEL PRONTO");
    jump_line();
    write_string("PARA CHUTE");
    TMR0 = 0;
    while(FTI1 == 0);   
    
    clear_scr();
    
    //write_string(" FAZENDO LEITURA");
    
    acq_data();
   
    __delay_ms(10000);
    
    clear_scr(); // Limpa o screen do LCD
    
    }
    //clear_data(); //Limpa os resultados
    
    //msg_init(); // Imprime uma tela de inicialização

}