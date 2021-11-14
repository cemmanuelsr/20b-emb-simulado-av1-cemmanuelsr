#include <asf.h>

// Buttons

#define BUT_PIO			PIOA
#define BUT_PIO_ID		ID_PIOA
#define BUT_IDX			11
#define BUT_IDX_MASK	(1 << BUT_IDX)

#define BUT1_PIO		PIOC
#define BUT1_PIO_ID		ID_PIOC
#define BUT1_IDX		31
#define BUT1_IDX_MASK	(1 << BUT1_IDX)

#define BUT2_PIO		PIOA
#define BUT2_PIO_ID		ID_PIOA
#define BUT2_IDX		19
#define BUT2_IDX_MASK	(1 << BUT2_IDX)

// LEDs

#define LED1_PIO		PIOA
#define LED1_PIO_ID		ID_PIOA
#define LED1_IDX		0
#define LED1_IDX_MASK	(1 << LED1_IDX)

#define LED2_PIO		PIOC
#define LED2_PIO_ID		ID_PIOC
#define LED2_IDX		30
#define LED2_IDX_MASK	(1 << LED2_IDX)

#define LED3_PIO		PIOB
#define LED3_PIO_ID		ID_PIOB
#define LED3_IDX		2
#define LED3_IDX_MASK	(1 << LED3_IDX)

// functions declaration

void init(void);
void but_callback(void);
void but1_callback(void);
void but2_callback(void);
void pin_toggle(Pio *pio, uint32_t mask);

// flags

volatile char flag_tc1 = 1;
volatile char flag_tc2 = 1;
volatile char flag_tc3 = 1;

// functions implementation

void but_callback(void) {
	flag_tc1 = 1 - flag_tc1;
}
void but1_callback(void) {
	flag_tc2 = 1 - flag_tc2;
}
void but2_callback(void) {
	flag_tc3 = 1 - flag_tc3;
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask)) pio_clear(pio, mask);
	else pio_set(pio,mask);
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
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrup�c�o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup�c�o no TC canal 0 */
	/* Interrup��o no C */
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	//flag_tc = 1;
	//if(flag_tc1 == 1) pisca_led(LED1_PIO, LED1_IDX_MASK, 1, 10);
	if(flag_tc1 == 1) pin_toggle(LED1_PIO, LED1_IDX_MASK);
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
	//flag_tc = 1;
	//if(flag_tc2 == 1) pisca_led(LED2_PIO, LED2_IDX_MASK, 1, 10);
	if(flag_tc2 == 1) pin_toggle(LED2_PIO, LED2_IDX_MASK);
}

void TC2_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	//flag_tc = 1;
	//if(flag_tc3 == 1) pisca_led(LED3_PIO, LED3_IDX_MASK, 1, 10);
	if(flag_tc3 == 1) pin_toggle(LED3_PIO, LED3_IDX_MASK);
}

void init(void) {
	// Enable LEDs
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_PULLUP);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_IDX_MASK, PIO_DEFAULT);
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_IDX_MASK, PIO_PULLUP);
	
	pio_clear(LED1_PIO, LED1_IDX_MASK);
	pio_set(LED2_PIO, LED2_IDX_MASK);
	pio_clear(LED3_PIO, LED3_IDX_MASK);

	// Configura interrup��o no pino referente ao botao e associa
	// fun��o de callback caso uma interrup��o for gerada
	pio_handler_set(BUT_PIO,
					BUT_PIO_ID,
					BUT_IDX_MASK,
					PIO_IT_RISE_EDGE,
					but_callback);
	pio_handler_set(BUT1_PIO,
					BUT1_PIO_ID,
					BUT1_IDX_MASK,
					PIO_IT_FALL_EDGE,
					but1_callback);
	pio_handler_set(BUT2_PIO,
					BUT2_PIO_ID,
					BUT2_IDX_MASK,
					PIO_IT_RISE_EDGE,
					but2_callback);

	// Ativa interrup��o
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
	pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr�ximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
	NVIC_SetPriority(BUT2_PIO_ID, 4);
	
	// TC init
	TC_init(TC0, ID_TC0, 0, 5);
	TC_init(TC0, ID_TC1, 1, 10);
	TC_init(TC0, ID_TC2, 2, 1);
}

int main (void)
{
	sysclk_init();
	delay_init();
	init();
	
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
