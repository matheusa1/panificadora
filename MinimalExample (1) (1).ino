#include "LCD.h" // LCD

// temperature related
#define MAX_TEMP  212.f
#define MIN_TEMP  11.f
#define MAX_COUNT 1011
#define MIN_COUNT 79
//212 1011
//190 912
//160 771
//138 670
//120 585
//29 154
//11 79

#define BOTAO_UP PC5
#define BOTAO_DOWN PC3
#define BOTAO_LEFT PC2
#define BOTAO_RIGHT PC4

unsigned int adc_res; // temperatura

// --------------------
//Variables
float temperature_read = 0.0;

#define num_ADC_average 32
volatile unsigned int adc_result0[num_ADC_average]; // average 32 conversions channel 0
volatile unsigned int adc_pos0 = num_ADC_average-1; // current ADC value for channel 0

volatile unsigned long my_millis = 0;

#define MOTORON   PORTD |= 1<<PD3
#define MOTOROFF  PORTD &= ~(1<<PD3)

#define HEATON   PORTD |= 1<<PD2
#define HEATOFF  PORTD &= ~(1<<PD2)

#define BUZZON   PORTC |= 1<<PC1
#define BUZZOFF  PORTC &= ~(1<<PC1)
#define BUZZCOM PORTC ^= (1<<PC1)

#define COUNT2TEMP(c)  (MIN_TEMP+(((MAX_TEMP-MIN_TEMP)/(MAX_COUNT-MIN_COUNT))*(((float)(c))-MIN_COUNT)))

volatile unsigned int beep_period;

#define BEEP()  beep_period = 200

int main() {
  //void setup() {
  // disable interrupts
  cli();

  DDRC &= ~(1<<BOTAO_UP);
  DDRC &= ~(1<<BOTAO_DOWN);
  DDRC &= ~(1<<BOTAO_RIGHT);
  DDRC &= ~(1<<BOTAO_LEFT);

  PORTC |= (1<<BOTAO_UP);
  PORTC |= (1<<BOTAO_DOWN);
  PORTC |= (1<<BOTAO_LEFT);
  PORTC |= (1<<BOTAO_RIGHT);


  MOTOROFF;
  HEATOFF;
  BUZZON;
  
  ADMUX = 0; // ADC0, AREF
  ADCSRA = 1<<ADEN|1<<ADIE;
  ADCSRA |= 1<<ADPS2|1<<ADPS1|1<<ADPS0; // 128 ADC prescaler / 9615 conversions/second
  ADCSRB = 0;
  DIDR0 = 1<<ADC0D; // disable digital input on A0

  // configuracao timer3
  // use mode 0 normal
  TCCR1A = 0;
  TCCR1B = (1<<CS11); // clkio/8 prescaler
  TCCR1C = 0;
  OCR1A = 0x07CF; //1999 that counts 2000 = 1ms
  TIMSK1 = 1<< OCIE1A; // output compare unit A

  DDRD = 0xFF; // LCD e Motor e Resistencia
  DDRB = 1<<PB0|1<<PB1; // LCD
  DDRC = 1<<PC1; // BUZZER

  // enable interrupts
  sei();

  Serial.begin(115200);

  ADCSRA |= 1<<ADSC; // start ADC conversion

  inic_LCD_4bits();

  BEEP();
  MOTORON;

  int set_sova = 0; //Valor do tempo de sova em segundos
  int set_grow = 0;//Valor do tempo de grow  em segundos
  int set_assar = 0;//Valor do tempo de assar em segundos

  int etapa = 1; // Indica a etapa do processo de fazer pão

  int config_step = 1; //Prende no primeiro laço para a configuração do tempo
  int cursor_index = 0; // Indica o valor do cursor;

  unsigned long btn_clicked = my_millis();

  while(1) {
    while(config_step) { // Laço para a configuração dos tempos
      if(!tst_bit(PINC,BOTAO_UP) && (my_millis()- btn_clicked)>150) {
        switch(cursor_index) {
          case 0  :
            set_sova++; // aumenta o valor do tempo de sova
          case 1 :
            set_grow++; // aumenta o valor do tempo de grow
          case 2 :
            set_assar++; // aumenta o valor do tempo de assar
          case 3 :
            config_step = 0; // sai do laço de configuração
        }
        btn_clicked = my_millis();
      }

      if(!tst_bit(PINC,BOTAO_DOWN) && (my_millis()- btn_clicked)>150) {
        switch(cursor_index) {
          case 0  :
            set_sova--; // diminui o valor do tempo de sova
          case 1 :
            set_grow--; // diminui o valor do tempo de grow
          case 2 :
            set_assar--; // diminui o valor do tempo de assar
          case 3 :
            config_step = 0; // sai do laço de configuração
        }
        btn_clicked = my_millis();
      }

      if(!tst_bit(PINC,BOTAO_RIGHT) && (my_millis()- btn_clicked)>150) {
        if(cursor_index < 3)  cursor_index++; // passa o cursor para a direita.
        btn_clicked = my_millis();
      }

      if(!tst_bit(PINC,BOTAO_LEFT) && (my_millis()- btn_clicked)>150) {
        if(cursor_index > 0)  cursor_index--; // passa o cursor para a esquerda
        btn_clicked = my_millis();
      }

      cmd_LCD(0x80, 0); // escreve os valores do tempo atual
      escreve_LCD(set_sova+"");

      cmd_LCD(0x82, 0);
      escreve_LCD(set_grow+"");

      cmd_LCD(0x84, 0);
      escreve_LCD(set_assar+"");

      cmd_LCD(0x86, 0);
      escreve_LCD("Fim");
    }

    int tempo_sova = set_sova * 1000;
    int tempo_grow = set_grow * 1000;
    int tempo_assar = set_assar * 1000;

    if (my_millis%1000==0) {
      Serial.print(", ADC0: ");
      adc_res = 0;
      for (int i=0; i<num_ADC_average; i++)
        adc_res += adc_result0[i];
      adc_res /= num_ADC_average;
      Serial.print(adc_res);
      Serial.print(", ");

      temperature_read = COUNT2TEMP(adc_res);
      unsigned char digitos[tam_vetor];
      ident_num(temperature_read,digitos);
      Serial.print(temperature_read);
      Serial.println();
      cmd_LCD(0x8D,0);      //desloca o cursor para que os 3 digitos fiquem a direita do LCD
      cmd_LCD(digitos[2],1);
      cmd_LCD(digitos[1],1);
      cmd_LCD(digitos[0],1);

    }

    if (my_millis % tempo_sova == 0 && my_millis / tempo_sova == 1 && etapa == 1) { // se esta na primeira etapa e compara com o tempo para desativar o motor
      MOTOROFF;
      my_millis = 0;
      etapa++;
    }

    if(my_millis % tempo_grow == 0 && my_millis / tempo_grow == 1 && etapa == 2) { // se esta na segunda etapa e o tempo acaba liga o HEATON
      my_millis = 0;
      etapa++;
      HEATON;
    }

    temperature_read = COUNT2TEMP(adc_res); 
    if(temperature_read >= MAX_TEMP - 2 && etapa == 3) { // se a temperatura passar do maximo desliga o HEAT
      HEATOFF;
    }
    if(temperature_read <= MAX_TEMP - 10 && etapa == 3) { // caso a temperatura caia muito liga o HEAT
      HEATON;
    }

    if(my_millis % tempo_assar == 0 && my_millis / tempo_assar == 1 && etapa == 3) { // ao acabar o tempo de assar desliga tudo e da um beep para indicar que o processo foi finalizado
      my_millis = 0;
      HEATOFF;
      BEEP();
    }    
    
  }//while 1

  return 0;
}

ISR (ADC_vect)
{
  adc_result0[adc_pos0++%num_ADC_average] = ADC;
  ADCSRA |= 1<<ADSC; // start new ADC conversion  
}

ISR(TIMER1_COMPA_vect)
{
  OCR1A += 0x07CF;
  my_millis += 1;

  if (beep_period > 0) {
    beep_period--;
    BUZZCOM;
  }
}
