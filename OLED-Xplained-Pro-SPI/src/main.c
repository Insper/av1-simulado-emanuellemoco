#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/
// Configuracoes do botoes OLED
#define BUT1_PIO			PIOD
#define BUT1_PIO_ID			16
#define BUT1_PIO_IDX		28
#define BUT1_PIO_IDX_MASK	(1u << BUT1_PIO_IDX)

#define BUT2_PIO			PIOC
#define BUT2_PIO_ID			12
#define BUT2_PIO_IDX		31
#define BUT2_PIO_IDX_MASK	(1u << BUT2_PIO_IDX)

#define BUT3_PIO			PIOA
#define BUT3_PIO_ID			10
#define BUT3_PIO_IDX		19
#define BUT3_PIO_IDX_MASK	(1u << BUT3_PIO_IDX)

// Configuracoes dos LEDs
#define LED1_PIO			PIOA
#define LED1_PIO_ID			ID_PIOA
#define LED1_PIO_IDX		0
#define LED1_PIO_IDX_MASK	(1 << LED1_PIO_IDX)

#define LED2_PIO			PIOC
#define LED2_PIO_ID			ID_PIOC
#define LED2_PIO_IDX		30
#define LED2_PIO_IDX_MASK	(1 << LED2_PIO_IDX)

#define LED3_PIO			PIOB
#define LED3_PIO_ID			ID_PIOB
#define LED3_PIO_IDX		2
#define LED3_PIO_IDX_MASK	(1 << LED3_PIO_IDX)

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile char but1_flag = 0;
volatile char but2_flag = 0;
volatile char but3_flag = 0;

volatile char flag1_tc = 0; 
volatile char flag2_tc = 0; 
volatile char flag3_tc = 0; 

volatile Bool f_rtt_alarme = false; //RTT
volatile char flag_rtt = 0; //RTT

volatile char flag_rtc = 0; // RTC


/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t seccond;
} calendar;


/* prototypes                                                           */
/************************************************************************/
// void init(void);
// void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
// static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
//void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void but1_callBack(void){
	but1_flag =! but1_flag;
}
void but2_callBack(void){
	but2_flag =! but2_flag;
}
void but3_callBack(void){
	but3_flag =! but3_flag;
}

// Fun��o de inicializa��o do uC e config dos perifericos e pinos
void init(void)
{
	//Ativa o PIO na qual o LED foi conectado
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);

	//Inicializa cada PIO como saida
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 1, 0, 0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
	
	// Inicializa PIO do botao
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);

	// configura pino ligado ao bot�o como entrada com um pull-up.
	pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, PIO_PULLUP); 
	pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, PIO_PULLUP);
	pio_set_input(BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_PULLUP);
	
	// Configura interrup��o no pino referente ao botao e associa
	// fun��o de callback caso uma interrup��o for gerada
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_RISE_EDGE, but1_callBack);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but2_callBack);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_RISE_EDGE, but3_callBack);
	
	//Ativar o pull-up
	pio_pull_up(BUT1_PIO, BUT1_PIO_IDX_MASK, 1);
	pio_pull_up(BUT2_PIO, BUT2_PIO_IDX_MASK, 1);
	pio_pull_up(BUT3_PIO, BUT3_PIO_IDX_MASK, 1);

	// Ativa interrup��o
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr�ximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_EnableIRQ(BUT3_PIO_ID);

	NVIC_SetPriority(BUT1_PIO_ID, 4); 
	NVIC_SetPriority(BUT2_PIO_ID, 4);
	NVIC_SetPriority(BUT3_PIO_ID, 4);
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag1_tc = !flag1_tc;
}

void TC4_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC1, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag2_tc = ! flag2_tc;
}

void TC7_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC2, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag3_tc = ! flag3_tc;
}



void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		//pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);    // BLINK Led
		flag_rtt =! flag_rtt;
		f_rtt_alarme = true;                  // flag RTT alarme
	}
	
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		//ENTROU AQUI
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		flag_rtc = 1;
	}
	

	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	

}


void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter � meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	//BY LE
	pmc_enable_periph_clk(ID_TC);*/

	pmc_enable_periph_clk(ID_TC);
	// Configura o TC para operar em  4Mhz e interrup�c�o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup�c�o no TC canal 0 */
	/* Interrup��o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);

}

void show_time(calendar rtc_initial){
// rtc_get_time(RTC, &h, &m, &s);

	//Exiba a hora no formato (HH:MM:SS) no display OLED
	char b[512];

 	sprintf(b, "%2d : %2d : %2d", rtc_initial.hour, rtc_initial.minute, rtc_initial.seccond);

    gfx_mono_draw_string(b, 10,16, &sysfont);

}


int main (void)
{
	// inicializa sistema e IOs
	init();
	board_init();
	sysclk_init();
	delay_init();

  	/** Configura timer TC0, canal e frequencia */
	TC_init(TC0, ID_TC1, 1, 5);
	TC_init(TC1, ID_TC4, 1, 10);
	TC_init(TC2, ID_TC7, 1, 1);

	// Inicializa RTT com IRQ no alarme.
	f_rtt_alarme = true;

	// Init OLED
	gfx_mono_ssd1306_init();
	
		/** Configura RTC */
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};

	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_SECEN);

	/* configura alarme do RTC */
	rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
	rtc_set_time_alarm(RTC, 1, rtc_initial.hour, 1, rtc_initial.minute, 1, rtc_initial.seccond + 20);
	
	gfx_mono_draw_string("          ", 1,16, &sysfont);
  
  // Escreve na tela um circulo e um texto
	//gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
  //gfx_mono_draw_string("5  10  1", 30,16, &sysfont);

  /* Insert application code here, after the board has been initialized. */
	while(1) {
	if(flag_rtc){
		rtc_get_time(RTC, &rtc_initial.hour, &rtc_initial.minute, &rtc_initial.seccond);
		flag_rtc = 0;
		show_time(rtc_initial);

	}

	if(f_rtt_alarme){
		uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
		uint32_t irqRTTvalue = 20;
		RTT_init(pllPreScale, irqRTTvalue); 
		f_rtt_alarme = false;
	}

	if (flag_rtt){
		if (but1_flag){		
			if(flag1_tc) pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
			else pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);}

		if (but2_flag){
			if(flag2_tc) pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			else pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);}

		if (but3_flag){
			if(flag3_tc) pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			else pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);}	
	}
	else{ 	
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
		pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
		
	}

	}
}
