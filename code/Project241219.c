/*
button 3/2 : accel/brake - DC motor, 7Seg, LED(brake, 00111100)
button 5/4 toggle : L/R turn blinker - LED blinking toggle(11000000, 00000011)
button 1 toggle : Gear : Forward or Backward
시계방향 : 전진 / 반시계바향 : 후진

VR : handle - step motor : 제대로 동작하지 않을 시, 메인보드 위 가변저항으로 대체.

uWave : AEB - DC motor, 7seg, LED blinking(11111111), buzzer
 */



#include "S32K144.h"
// #include "Startup_S32K144.s"

#define PCC_BASE	(0x40065000)
#define PCC_LPIT	*((volatile unsigned*)(PCC_BASE + 0xDC))
#define PCC_FTM0	*((volatile unsigned*)(PCC_BASE + 0xE0))
#define PCC_FTM2    *((volatile unsigned*)(PCC_BASE + 0xE8))
#define PCC_ADC0	*((volatile unsigned*)(PCC_BASE + 0xEC))

#define PCC_PORTA   *((volatile unsigned*)(PCC_BASE + 0x124))
#define PCC_PORTB   *((volatile unsigned*)(PCC_BASE + 0x128))
#define PCC_PORTC   *((volatile unsigned*)(PCC_BASE + 0x12C))
#define PCC_PORTD   *((volatile unsigned*)(PCC_BASE + 0x130))
#define PCC_PORTE   *((volatile unsigned*)(PCC_BASE + 0x134))

#define PCS_BITS	24
#define CGC_BIT		30


#define LPIT_BASE	(0x40037000)
#define LPIT_MCR	*((volatile unsigned*)(LPIT_BASE + 0x8))
#define LPIT_MSR	*((volatile unsigned*)(LPIT_BASE + 0xC))
#define LPIT_MIER	*((volatile unsigned*)(LPIT_BASE + 0x10))
#define LPIT_TVAL0	*((volatile unsigned*)(LPIT_BASE + 0x20))
#define LPIT_TCTRL0	*((volatile unsigned*)(LPIT_BASE + 0x28))
#define LPIT_TVAL1	*((volatile unsigned*)(LPIT_BASE + 0x30))
#define LPIT_TCTRL1	*((volatile unsigned*)(LPIT_BASE + 0x38))

#define M_CEN_BIT	0
#define TIF0_BIT	0
#define TIF1_BIT	1
#define TIE0_BIT	0
#define TIE1_BIT	1
#define MODE_BITS	2
#define T_EN_BIT	0


#define NVIC_ISER_BASE	(0xE000E100)
#define NVIC_ISER1		*((volatile unsigned*)(NVIC_ISER_BASE + 0x4))

#define NVIC_ICPR_BASE	(0xE000E280)
#define NVIC_ICPR1		*((volatile unsigned*)(NVIC_ICPR_BASE + 0x4))

#define NVIC_IPR_BASE	(0xE000E400)
#define NVIC_IPR48		*((volatile unsigned char*)(NVIC_IPR_BASE + 0x30)) //LPIT0_CH0
#define NVIC_IPR49		*((volatile unsigned char*)(NVIC_IPR_BASE + 0x31)) //LPIT0_CH1
#define NVIC_IPR59		*((volatile unsigned char*)(NVIC_IPR_BASE + 0x3B)) //PORTA
#define NVIC_IPR61		*((volatile unsigned char*)(NVIC_IPR_BASE + 0x3D)) //PORTC


#define FTM0_BASE	(0x40038000)
#define FTM0_SC		*((volatile unsigned*)(FTM0_BASE + 0x0))
#define FTM0_MOD	*((volatile unsigned*)(FTM0_BASE + 0x8))
#define FTM0_C1SC	*((volatile unsigned*)(FTM0_BASE + 0x14))
#define FTM0_C1V	*((volatile unsigned*)(FTM0_BASE + 0x18))
#define FTM0_CNTIN  *((volatile unsigned*)(FTM0_BASE + 0x4C))

#define FTM2_BASE	(0x4003A000)
#define FTM2_SC     *((volatile unsigned*)(FTM2_BASE + 0x0))
#define FTM2_MOD  	*((volatile unsigned*)(FTM2_BASE + 0x8))
#define FTM2_C2SC  	*((volatile unsigned*)(FTM2_BASE + 0x1C))
#define FTM2_C2V   	*((volatile unsigned*)(FTM2_BASE + 0x20))
#define FTM2_C5SC  	*((volatile unsigned*)(FTM2_BASE + 0x34))
#define FTM2_C5V   	*((volatile unsigned*)(FTM2_BASE + 0x38))
#define FTM2_CNTIN 	*((volatile unsigned*)(FTM0_BASE + 0x4C))

#define PWMEN0_BIT	16
#define PWMEN1_BIT	17
#define PWMEN2_BIT	18
#define PWMEN5_BIT  21
#define CLKS_BITS	3
#define PS_BITS		0
#define MSB_BIT     5
#define MSA_BIT     4
#define ELSB_BIT	3
#define ELSA_BIT	2


#define ADC0_BASE   (0x4003B000)
#define ADC0_SC1A   *((volatile unsigned*)(ADC0_BASE + 0x0))
#define ADC0_CFG1   *((volatile unsigned*)(ADC0_BASE + 0x40))
#define ADC0_CFG2   *((volatile unsigned*)(ADC0_BASE + 0x44))
#define ADC0_RA     *((volatile unsigned*)(ADC0_BASE + 0x48))
#define ADC0_SC2	*((volatile unsigned*)(ADC0_BASE + 0x90))
#define ADC0_SC3	*((volatile unsigned*)(ADC0_BASE + 0x94))

#define ADCH_BITS	0
#define COCO_BIT	7
#define MODE_BITS  	2
#define ADIV_BITS   5
#define SMPLTS_BITS 0
#define ADTRG_BIT   6
#define ADC0_SE7    7   // PTB3 : VR&CDS 제대로 동작하지 않을 경우, 주석처리



#define PORTA_BASE	(0x40049000)
#define PORTA_PCR1	*((volatile unsigned*)(PORTA_BASE + 0x4)) // button_1 : Gear(Forward / Backward)
#define PORTA_PCR2	*((volatile unsigned*)(PORTA_BASE + 0x8)) // button_2 : accel
#define PORTA_PCR3  *((volatile unsigned*)(PORTA_BASE + 0xC)) // button_3 : brake
#define PORTA_PCR11 *((volatile unsigned*)(PORTA_BASE + 0x2C)) // button_4 : blinkerR
#define PORTA_PCR12 *((volatile unsigned*)(PORTA_BASE + 0x30)) // button_5 : blinkerL
#define PORTA_PCR7	*((volatile unsigned*)(PORTA_BASE + 0x1C)) //uWave Echo
#define PORTA_PCR8	*((volatile unsigned*)(PORTA_BASE + 0x20)) //uWave Trig

#define PORTB_BASE	(0x4004A000)
#define PORTB_PCR0	*((volatile unsigned*)(PORTB_BASE + 0x0)) // Seg_COM3 : velocity 00x
#define PORTB_PCR1	*((volatile unsigned*)(PORTB_BASE + 0x4)) // Seg_COM4 : velocity 0x0
#define PORTB_PCR2	*((volatile unsigned*)(PORTB_BASE + 0x8)) // Seg_COM5 : velocity x00
#define PORTB_PCR12	*((volatile unsigned*)(PORTB_BASE + 0x30)) //Seg_COM6 : Gear Type
#define PORTB_PCR3	*((volatile unsigned*)(PORTB_BASE + 0xC)) // VR&CDS INPUT
#define PORTB_PCR17	*((volatile unsigned*)(PORTB_BASE + 0x44)) // Buzzer

#define PORTC_BASE  (0x4004B000)
#define PORTC_PCR13 *((volatile unsigned*)(PORTC_BASE + 0x34)) // EVB - Left Button : P GEAR

#define PORTD_BASE  (0x4004C000)
#define PORTD_PCR0  *((volatile unsigned*)(PORTD_BASE + 0x0))  // Seg_A
#define PORTD_PCR7  *((volatile unsigned*)(PORTD_BASE + 0x1C)) // Seg_B
#define PORTD_PCR2  *((volatile unsigned*)(PORTD_BASE + 0x8))  // Seg_C
#define PORTD_PCR3  *((volatile unsigned*)(PORTD_BASE + 0xC))  // Seg_D
#define PORTD_PCR4  *((volatile unsigned*)(PORTD_BASE + 0x10)) // Seg_E
#define PORTD_PCR5  *((volatile unsigned*)(PORTD_BASE + 0x14)) // Seg_F
#define PORTD_PCR6  *((volatile unsigned*)(PORTD_BASE + 0x18)) // Seg_G
#define PORTD_PCR12 *((volatile unsigned*)(PORTD_BASE + 0x30)) // DC Motor Forward PWM
#define PORTD_PCR14 *((volatile unsigned*)(PORTD_BASE + 0x38)) // DC Motor Backward PWM
#define PORTD_PCR16 *((volatile unsigned*)(PORTD_BASE + 0x40)) // Servo Motor PWM


#define PORTE_BASE	(0x4004D000)
#define PORTE_PCR0	*((volatile unsigned*)(PORTE_BASE + 0x0))   // LED_1 : most right one
#define PORTE_PCR1  *((volatile unsigned*)(PORTE_BASE + 0x4))   // LED_2
#define PORTE_PCR2  *((volatile unsigned*)(PORTE_BASE + 0x8))   // LED_3
#define PORTE_PCR3  *((volatile unsigned*)(PORTE_BASE + 0xC))   // LED_4
#define PORTE_PCR8  *((volatile unsigned*)(PORTE_BASE + 0x20))  // LED_5
#define PORTE_PCR5  *((volatile unsigned*)(PORTE_BASE + 0x14))  // LED_6
#define PORTE_PCR6  *((volatile unsigned*)(PORTE_BASE + 0x18))  // LED_7
#define PORTE_PCR7  *((volatile unsigned*)(PORTE_BASE + 0x1C))  // LED_8 : most left one

#define MUX_BITS	8
#define IRQC_BITS	16 //Interrupt Type
#define ISF_BIT		24 //Interrupt set flag


#define GPIOA_BASE	(0x400FF000)
#define GPIOA_PSOR  *((volatile unsigned*)(GPIOA_BASE + 0x4))
#define GPIOA_PCOR  *((volatile unsigned*)(GPIOA_BASE + 0x8))
#define GPIOA_PDIR	*((volatile unsigned*)(GPIOA_BASE + 0x10))
#define GPIOA_PDDR  *((volatile unsigned*)(GPIOA_BASE + 0x14))

#define GPIOB_BASE  (0x400FF040)
#define GPIOB_PSOR  *((volatile unsigned*)(GPIOB_BASE + 0x4))
#define GPIOB_PCOR  *((volatile unsigned*)(GPIOB_BASE + 0x8))
#define GPIOB_PDDR  *((volatile unsigned*)(GPIOB_BASE + 0x14))

#define GPIOC_BASE  (0x400FF080)
#define GPIOC_PSOR  *((volatile unsigned*)(GPIOC_BASE + 0x4))
#define GPIOC_PCOR  *((volatile unsigned*)(GPIOC_BASE + 0x8))
#define GPIOC_PDDR  *((volatile unsigned*)(GPIOC_BASE + 0x14))

#define GPIOD_BASE  (0x400FF0C0)
#define GPIOD_PSOR  *((volatile unsigned*)(GPIOD_BASE + 0x4))
#define GPIOD_PCOR  *((volatile unsigned*)(GPIOD_BASE + 0x8))
#define GPIOD_PDDR  *((volatile unsigned*)(GPIOD_BASE + 0x14))

#define GPIOE_BASE  (0x400FF100)
#define GPIOE_PSOR  *((volatile unsigned*)(GPIOE_BASE + 0x4))
#define GPIOE_PCOR  *((volatile unsigned*)(GPIOE_BASE + 0x8))
#define GPIOE_PDDR  *((volatile unsigned*)(GPIOE_BASE + 0x14))

#define PTA1	1
#define PTA2	2
#define PTA3    3
#define PTA6    6
#define PTA11   11
#define PTA12   12
#define PTA7	7//uWave Echo
#define PTA8	8//uWave trig

#define PTB0    0
#define PTB1    1
#define PTB2    2
#define PTB3    3
#define PTB12	12//Digit4
#define PTB17	17//buzzer

#define PTC13	13//Parking GEAR

#define PTD0    0
#define PTD7    7
#define PTD2    2
#define PTD3    3
#define PTD4    4
#define PTD5    5
#define PTD6    6
#define	PTD12	12//forward FTM2 CH2
#define PTD14	14//backward FTM2 CH5
#define PTD16	16//servo FTM0 CH1

#define PTE0    0
#define PTE1    1
#define PTE2    2
#define PTE3    3
#define PTE8    8
#define PTE5    5
#define PTE6    6
#define PTE7    7


int time = 0; // unit : 0.01s
int lastStepTime = 0; // for checking time difference
float velocity = 0; // 0.00000 ~

int handleAngle; // 0 ~ 2047 ~ 4095
int blinkerSwitch = 1; // 0, 1, 2 : left, idle, right
int blinkerTime; // 0 ~
int toggleBlinker = 0;
int brakeLED = 0; // 0, 1 : no brake, brake

int SelectGearForwardBackward = 1; // 0, 1 : Backward, Forward
int Buzzer_flag = 0;

int uWaveDistance = 0;
int uWaveCount = 0;
int uWaveLevelHigh = 0;

#define P	0
#define R	2
#define	D	3
#define MaxSpd	100 // can be modified
volatile uint8_t GEARSET = P; //default P GEAR

#define MAX_HANDLE_RIGHT	1200 //+90
#define MAX_HANDLE_LEFT		5000  //-90
#define BUZZER_ALERT_SET	(Buzzer_flag = 30)
#define FORWARD_SET				(SelectGearForwardBackward = 1)
#define BACKWARD_SET			(SelectGearForwardBackward = 0)
#define FORWARD_CHECK			(SelectGearForwardBackward == 1)
#define BACKWARD_CHECK			(SelectGearForwardBackward == 0)
#define GEAR_P					(GEARSET == P)
#define GEAR_R					(GEARSET == R)
#define GEAR_D					(GEARSET == D)

void setHandle(int handleAngle);
void setBlinkLed(int blinkerSwitch);
void setBrakeLED(int brakeLED);
void setDCmotorGear(int velocity);
void Buzzer_set();
void uWaveSensor_10us();


/*-----------------------Clock------------------------*/
void SOSC_init_8MHz(void) {
   SCG->SOSCDIV = SCG_SOSCDIV_SOSCDIV1(1)|SCG_SOSCDIV_SOSCDIV2(1);
   SCG->SOSCCFG = SCG_SOSCCFG_RANGE(2)|SCG_SOSCCFG_EREFS_MASK;

   while(SCG->SOSCCSR & SCG_SOSCCSR_LK_MASK);
   SCG->SOSCCSR = SCG_SOSCCSR_SOSCEN_MASK;

   while(!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK));
}

void SPLL_init_160MHz(void) {
   while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK);
   SCG->SPLLCSR &= ~SCG_SPLLCSR_SPLLEN_MASK;

   SCG->SPLLDIV |= SCG_SPLLDIV_SPLLDIV1(2) | SCG_SPLLDIV_SPLLDIV2(3);

   SCG->SPLLCFG = SCG_SPLLCFG_MULT(24);

   while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK);
   SCG->SPLLCSR |= SCG_SPLLCSR_SPLLEN_MASK;

   while(!(SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK));
}

void NormalRUNmode_80MHz(void) {
   SCG->SIRCDIV = SCG_SIRCDIV_SIRCDIV1(1) | SCG_SIRCDIV_SIRCDIV2(1);

   SCG->RCCR = SCG_RCCR_SCS(6) | SCG_RCCR_DIVCORE(0b01) | SCG_RCCR_DIVBUS(0b01) | SCG_RCCR_DIVSLOW(0b10);

   while (((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) != 6) {}
}
/*-----------------------Clock------------------------*/



void PORT_init() {

   PCC_PORTA |= (1 << CGC_BIT);

   PORTA_PCR1 &= ~((0b111) << MUX_BITS);
   PORTA_PCR1 |= (1 << MUX_BITS);

   PORTA_PCR1 &= ~((0b1111)<<IRQC_BITS);
   PORTA_PCR1 |= ((0b1001)<<IRQC_BITS); //D->R , R->D Interrupt

   PORTA_PCR2 &= ~((0b111) << MUX_BITS);
   PORTA_PCR2 |= (1 << MUX_BITS);
   PORTA_PCR3 &= ~((0b111) << MUX_BITS);
   PORTA_PCR3 |= (1 << MUX_BITS);
   PORTA_PCR11 &= ~((0b111) << MUX_BITS);
   PORTA_PCR11 |= (1 << MUX_BITS);
   PORTA_PCR12 &= ~((0b111) << MUX_BITS);
   PORTA_PCR12 |= (1 << MUX_BITS);

   PORTA_PCR7 &= ~((0b111) << MUX_BITS);
   PORTA_PCR7 |= (1 << MUX_BITS);
   PORTA_PCR8 &= ~((0b111) << MUX_BITS);
   PORTA_PCR8 |= (1 << MUX_BITS);

   GPIOA_PDDR |= (1 << PTA8);
   GPIOA_PDDR &= ~(1 << PTA7);

   GPIOA_PDDR &= ~(1 << PTA1) & ~(1 << PTA2) & ~(1 << PTA3) & ~(1 << PTA11) & ~(1 << PTA12); // input


   PCC_PORTB |= (1 << CGC_BIT);

   PORTB_PCR0 &= ~((0b111) << MUX_BITS);
   PORTB_PCR0 |= (1 << MUX_BITS);
   PORTB_PCR1 &= ~((0b111) << MUX_BITS);
   PORTB_PCR1 |= (1 << MUX_BITS);
   PORTB_PCR2 &= ~((0b111) << MUX_BITS);
   PORTB_PCR2 |= (1 << MUX_BITS);
   PORTB_PCR12 &= ((0b111) << MUX_BITS);
   PORTB_PCR12 |= (1 << MUX_BITS);

   PORTB_PCR3 &= ~((0b111)<<MUX_BITS);
   PORTB_PCR3 |= ((0b010)<<MUX_BITS);
   PORTB_PCR17 &= ~((0b111)<<MUX_BITS);
   PORTB_PCR17 |= ((0b001)<<MUX_BITS);

   GPIOB_PDDR |= (1 << PTB0) | (1 << PTB1) | (1 << PTB2) | (1 << PTB12) | (1 << PTB17); // output


   PCC_PORTC |= (1 << CGC_BIT);
   PORTC_PCR13 &= ~((0b111) << MUX_BITS);
   PORTC_PCR13 |= (1 << MUX_BITS);

   GPIOC_PDDR &= ~(1 << PTC13);

   PORTC_PCR13 &= ~((0b1111)<<IRQC_BITS);
   PORTC_PCR13 |= ((0b1001)<<IRQC_BITS);

   PCC_PORTD |= (1 << CGC_BIT);

   PORTD_PCR0 &= ~((0b111) << MUX_BITS);
   PORTD_PCR0 |= (1 << MUX_BITS);
   PORTD_PCR7 &= ~((0b111) << MUX_BITS);
   PORTD_PCR7 |= (1 << MUX_BITS);
   PORTD_PCR2 &= ~((0b111) << MUX_BITS);
   PORTD_PCR2 |= (1 << MUX_BITS);
   PORTD_PCR3 &= ~((0b111) << MUX_BITS);
   PORTD_PCR3 |= (1 << MUX_BITS);
   PORTD_PCR4 &= ~((0b111) << MUX_BITS);
   PORTD_PCR4 |= (1 << MUX_BITS);
   PORTD_PCR5 &= ~((0b111) << MUX_BITS);
   PORTD_PCR5 |= (1 << MUX_BITS);
   PORTD_PCR6 &= ~((0b111) << MUX_BITS);
   PORTD_PCR6 |= (1 << MUX_BITS);

   GPIOD_PDDR |= (1 << PTD0) | (1 << PTD7) | (1 << PTD2) | (1 << PTD3) | (1 << PTD4) | (1 << PTD5) | (1 << PTD6); // output

   PORTD_PCR12 &= ~((0b111)<<MUX_BITS);
   PORTD_PCR12 |= ((0b010)<<MUX_BITS);

   PORTD_PCR14 &= ~((0b111)<<MUX_BITS);
   PORTD_PCR14 |= ((0b010)<<MUX_BITS);

   PORTD_PCR16 &= ~((0b111)<<MUX_BITS);
   PORTD_PCR16 |= ((0b010)<<MUX_BITS);


   PCC_PORTE |= (1 << CGC_BIT);

   PORTE_PCR0 &= ~((0b111) << MUX_BITS);
   PORTE_PCR0 |= (1 << MUX_BITS);
   PORTE_PCR1 &= ~((0b111) << MUX_BITS);
   PORTE_PCR1 |= (1 << MUX_BITS);
   PORTE_PCR2 &= ~((0b111) << MUX_BITS);
   PORTE_PCR2 |= (1 << MUX_BITS);
   PORTE_PCR3 &= ~((0b111) << MUX_BITS);
   PORTE_PCR3 |= (1 << MUX_BITS);
   PORTE_PCR8 &= ~((0b111) << MUX_BITS);
   PORTE_PCR8 |= (1 << MUX_BITS);
   PORTE_PCR5 &= ~((0b111) << MUX_BITS);
   PORTE_PCR5 |= (1 << MUX_BITS);
   PORTE_PCR6 &= ~((0b111) << MUX_BITS);
   PORTE_PCR6 |= (1 << MUX_BITS);
   PORTE_PCR7 &= ~((0b111) << MUX_BITS);
   PORTE_PCR7 |= (1 << MUX_BITS);

   GPIOE_PDDR |= (1 << PTE0) | (1 << PTE1) | (1 << PTE2) | (1 << PTE3) | (1 << PTE8) | (1 << PTE5) | (1 << PTE6) | (1 << PTE7); // output
}


/*-----------------------7Segment------------------------*/
void set7segmentNumClear() {

   GPIOD_PCOR |= (1 << PTD0) | (1 << PTD7) | (1 << PTD2) | (1 << PTD3) | (1 << PTD4) | (1 << PTD5) | (1 << PTD6);
}

void set7segmentNum0() {
   GPIOD_PSOR |= (1 << PTD0) | (1 << PTD7) | (1 << PTD2) | (1 << PTD3) | (1 << PTD4) | (1 << PTD5);
   GPIOD_PCOR |= (1 << PTD6);
}

void set7segmentNum1() {
   GPIOD_PSOR |= (1 << PTD7) | (1 << PTD2);
   GPIOD_PCOR |= (1 << PTD0) | (1 << PTD3) | (1 << PTD4) | (1 << PTD5) | (1 << PTD6);
}

void set7segmentNum2() {
   GPIOD_PSOR |= (1 << PTD0) | (1 << PTD7) | (1 << PTD3) | (1 << PTD4) | (1 << PTD6);
   GPIOD_PCOR |= (1 << PTD2) | (1 << PTD5);
}

void set7segmentNum3() {
   GPIOD_PSOR |= (1 << PTD0) | (1 << PTD7) | (1 << PTD2) | (1 << PTD3) | (1 << PTD6);
   GPIOD_PCOR |= (1 << PTD4) | (1 << PTD5);
}

void set7segmentNum4() {
   GPIOD_PSOR |= (1 << PTD7) | (1 << PTD2) | (1 << PTD5) | (1 << PTD6);
   GPIOD_PCOR |= (1 << PTD0) | (1 << PTD3) | (1 << PTD4);
}

void set7segmentNum5() {
   GPIOD_PSOR |= (1 << PTD0) | (1 << PTD2) | (1 << PTD3) | (1 << PTD5) | (1 << PTD6);
   GPIOD_PCOR |= (1 << PTD7) | (1 << PTD4);
}

void set7segmentNum6() {
   GPIOD_PSOR |= (1 << PTD0) | (1 << PTD2) | (1 << PTD3) | (1 << PTD4) | (1 << PTD5) | (1 << PTD6);
   GPIOD_PCOR |= (1 << PTD7);
}

void set7segmentNum7() {
   GPIOD_PSOR |= (1 << PTD0) | (1 << PTD7) | (1 << PTD2) | (1 << PTD5);
   GPIOD_PCOR |= (1 << PTD3) | (1 << PTD4) | (1 << PTD6);
}

void set7segmentNum8() {
   GPIOD_PSOR |= (1 << PTD0) | (1 << PTD7) | (1 << PTD2) | (1 << PTD3) | (1 << PTD4) | (1 << PTD5) | (1 << PTD6);
}

void set7segmentNum9() {
   GPIOD_PSOR |= (1 << PTD0) | (1 << PTD7) | (1 << PTD2) | (1 << PTD3) | (1 << PTD5) | (1 << PTD6);
   GPIOD_PCOR |= (1 << PTD4);
}

void set7segmentP(){
	GPIOD_PSOR |= (1 << PTD0) | (1 << PTD7) | (1 << PTD4) | (1 << PTD5) | (1 << PTD6);
	GPIOD_PCOR |= (1 << PTD2) | (1 << PTD3);
}

void set7segmentR(){
	GPIOD_PSOR |= (1 << PTD4) | (1 << PTD6);
	GPIOD_PCOR |= (1 << PTD0) | (1 << PTD7) | (1 << PTD2) | (1 << PTD3) | (1 << PTD5);
}

void set7segmentD(){
	GPIOD_PSOR |= (1 << PTD7) | (1 << PTD2) | (1 << PTD3) | (1 << PTD4) | (1 << PTD6);
	GPIOD_PCOR |= (1 << PTD0) | (1 << PTD5);
}

void set7segmentNum(int num) {
   switch(num) {
   case 0:
      set7segmentNum0();
      break;
   case 1:
      set7segmentNum1();
      break;
   case 2:
      set7segmentNum2();
      break;
   case 3:
      set7segmentNum3();
      break;
   case 4:
      set7segmentNum4();
      break;
   case 5:
      set7segmentNum5();
      break;
   case 6:
      set7segmentNum6();
      break;
   case 7:
      set7segmentNum7();
      break;
   case 8:
      set7segmentNum8();
      break;
   case 9:
      set7segmentNum9();
      break;
   default:
      set7segmentNumClear();
   }
}

void set7segmentGear(int GEAR){
	switch(GEAR){
	case 0:
		set7segmentP();
		break;
	case 2:
		set7segmentR();
		break;
	case 3:
		set7segmentD();
		break;
	default:
		set7segmentNumClear();
		break;
	}
}

void displayDigitClear() {
   set7segmentNumClear();

   GPIOB_PCOR |= (1 << PTB0) | (1 << PTB1) | (1 << PTB2) | (1 << PTB12);
}

void displayDigit1(int num) {
   GPIOB_PSOR |= (1 << PTB0);
   set7segmentNum(num);
}

void displayDigit2(int num) {
   GPIOB_PSOR |= (1 << PTB1);
   set7segmentNum(num);
}

void displayDigit3(int num) {
   GPIOB_PSOR |= (1 << PTB2);
   set7segmentNum(num);
}

void displayDigit4(int GEAR){
	GPIOB_PSOR |= (1 << PTB12);
	set7segmentGear(GEAR);

}

void displayDigits(int num, int GEAR) {
   int n1, n2, n3;

   n3 = (num % 1000) / 100;
   n2 = (num % 100) / 10;
   n1 = num % 10;

   displayDigitClear();
   displayDigit4(GEAR);
   for(int i = 0; i < 1000; i++) {}
   displayDigitClear();
   displayDigit3(n3);
   for(int i = 0; i < 1000; i++) {}
   displayDigitClear();
   displayDigit2(n2);
   for(int i = 0; i < 1000; i++) {}
   displayDigitClear();
   displayDigit1(n1);
   for(int i = 0; i < 1000; i++) {}
}
/*-----------------------7Segment------------------------*/

/*-----------------------Interrupt------------------------*/
void NVIC_init_IRQs(void) {
   NVIC_ICPR1 |= (1 << (48 % 32)) | (1 << (49 % 32)) | (1 << 59 % 32) | (1 << (61 % 32));
   NVIC_ISER1 |= (1 << (48 % 32)) | (1 << (49 % 32)) | (1 << 59 % 32) | (1 << (61 % 32));

   NVIC_IPR48 = (10<<4);
   NVIC_IPR49 = (11<<4);
   NVIC_IPR59 = (8<<4);
   NVIC_IPR61 = (8<<4);

}

void PORTA_IRQHandler(void) // PORTA interrupt handler
{
	if((GEARSET == P || GEARSET == R) && velocity == 0)
	{
		FTM2_C2V = 0;
		FTM2_C5V = 0;
		GEARSET = D;
		BUZZER_ALERT_SET;
		FORWARD_SET;
	}
	else if(GEARSET == D && velocity == 0)
	{
		FTM2_C5V = 0;
		FTM2_C5V = 0;
		GEARSET = R;
		BUZZER_ALERT_SET;
		BACKWARD_SET;// // 0, 1 : Backward, Forward
	}
	PORTA_PCR1 |= (1<<ISF_BIT);
}

void PORTC_IRQHandler(void)
{
	if(velocity == 0){
		GEARSET = P;
		BUZZER_ALERT_SET;
	}
	PORTC_PCR13 |= (1<<ISF_BIT);
}
/*-----------------------Interrupt------------------------*/


/*-----------------------VR&CDS------------------------*/
void ADC0_init(void) {
   PCC_ADC0 &= ~(1 << CGC_BIT);
   PCC_ADC0 &= ~((0b111) << PCS_BITS);
   PCC_ADC0 |= ((0b001) << PCS_BITS);
   PCC_ADC0 |= (1 << CGC_BIT);

   ADC0_SC1A |= ((0b111111) << ADCH_BITS);

   ADC0_CFG1 &= ~((0b11) << ADIV_BITS);
   ADC0_CFG1 &= ~((0b11) << MODE_BITS);
   ADC0_CFG1 |= ((0b01) << MODE_BITS);

   ADC0_CFG2 &= ~(255 << SMPLTS_BITS);
   ADC0_CFG2 |= (12 << SMPLTS_BITS);

   ADC0_SC2 &= ~(1 << ADTRG_BIT);
}

void adc_start(void) {
   ADC0_SC1A &= ~((0b111111) << ADCH_BITS);
   ADC0_SC1A |= (ADC0_SE7 << ADCH_BITS);
}


uint32_t read_adc_chx(void) {
   while((ADC0_SC1A & (1 << COCO_BIT)) == 0) {}
   return ADC0_RA;
}

/*-----------------------VR&CDS------------------------*/

/*-----------------------FTM------------------------*/
void FTM0_CH1_PWM(void) {
   PCC_FTM0 &= ~(1 << CGC_BIT);
   PCC_FTM0 &= ~((0b111) << PCS_BITS);
   PCC_FTM0 |= ((0b010) << PCS_BITS);
   PCC_FTM0 |= (1 << CGC_BIT);

   FTM0_SC |= (1 << PWMEN1_BIT);
   FTM0_SC &= ~((0b111) << PS_BITS);
   FTM0_SC |= ((0b010) << PS_BITS);

   FTM0_MOD = 40000;/////////////////////
   FTM0_CNTIN = 0;

   FTM0_C1SC |= (1 << MSB_BIT);
   FTM0_C1SC |= (1 << ELSB_BIT);

   FTM0_C1V = 0;

   FTM0_SC |= ((0b11) << CLKS_BITS);
}

void FTM2_CH2_PWM(void) {
   PCC_FTM2 &= ~(1 << CGC_BIT);
   PCC_FTM2 &= ~((0b111) << PCS_BITS);
   PCC_FTM2 |= ((0b010) << PCS_BITS);
   PCC_FTM2 |= (1 << CGC_BIT);

   FTM2_SC |= (1 << PWMEN2_BIT);
   FTM2_SC &= ~((0b111) << PS_BITS);
   FTM2_SC |= ((0b001) << PS_BITS);

   FTM2_MOD = 16000-1;
   FTM2_CNTIN = 0;

   FTM2_C2SC |= (1 << MSB_BIT);
   FTM2_C2SC |= (1 << ELSB_BIT);

   FTM2_C2V = 0;

   FTM2_SC |= ((0b11) << CLKS_BITS);
}


void FTM2_CH5_PWM(void) {
   PCC_FTM2 &= ~(1 << CGC_BIT);
   PCC_FTM2 &= ~((0b111) << PCS_BITS);
   PCC_FTM2 |= ((0b010) << PCS_BITS);
   PCC_FTM2 |= (1 << CGC_BIT);

   FTM2_SC |= (1 << PWMEN5_BIT);
   FTM2_SC &= ~((0b111) << PS_BITS);
   FTM2_SC |= ((0b001) << PS_BITS);

   FTM2_MOD = 16000-1;
   FTM2_CNTIN = 0;

   FTM2_C5SC |= (1 << MSB_BIT);
   FTM2_C5SC |= (1 << ELSB_BIT);

   FTM2_C5V = 0;

   FTM2_SC |= ((0b11) << CLKS_BITS);
}
/*-----------------------FTM------------------------*/



/*-----------------------LPIT0------------------------*/
void LPIT0_init(void) {
   PCC_LPIT &= ~((0b111) << PCS_BITS);
   PCC_LPIT |= ((0b110) << PCS_BITS);
   PCC_LPIT |= (1 << CGC_BIT);

   LPIT_MCR |= (1 << M_CEN_BIT);

   LPIT_MIER |= (1 << TIE0_BIT);

   LPIT_TVAL0 = 400000; //10ms

   LPIT_TCTRL0 &= ~((0b11) << MODE_BITS);
   LPIT_TCTRL0 |= (1 << T_EN_BIT);

   LPIT_MIER |= (1 << TIE1_BIT);

   LPIT_TVAL1 = 400; // 10us

   LPIT_TCTRL1 &= ~((0b11) << MODE_BITS);
   LPIT_TCTRL1 |= (1 << T_EN_BIT);
}

void LPIT0_Ch0_IRQHandler(void) {
   time += 1;
   LPIT_MSR |= (1 << TIF0_BIT);
}

void LPIT0_Ch1_IRQHandler(void) {
   uWaveSensor_10us();
   LPIT_MSR |= (1 << TIF1_BIT);
}
/*-----------------------LPIT0------------------------*/

int main(void) {

   PORT_init();
   SOSC_init_8MHz();
   SPLL_init_160MHz();
   NormalRUNmode_80MHz();

   NVIC_init_IRQs();
   LPIT0_init();
   ADC0_init();


   FTM0_CH1_PWM();
   FTM2_CH2_PWM();
   FTM2_CH5_PWM();

   for(;;) { // step delay 0.01

      // checking handleAngle
      adc_start();
      handleAngle = read_adc_chx(); // 0 ~ 2047 ~ 4095

      setHandle(handleAngle);
      setDCmotorGear((int)velocity);
      Buzzer_set();


      // checking blinker L/R
      if ((GPIOA_PDIR & (1 << PTA12)) == 0) {
         if (toggleBlinker == 0) {
            if (blinkerSwitch != 0) {
               blinkerSwitch = 0; // left Sig
               blinkerTime = time;
            }

            else {
               blinkerSwitch = 1; // turn off
            }

            toggleBlinker = 1;
         }
      }

      else if ((GPIOA_PDIR & (1 << PTA11)) == 0) {
         if (toggleBlinker == 0) {
            if (blinkerSwitch != 2) {
               blinkerSwitch = 2; // right Sig
               blinkerTime = time;
            }

            else {
               blinkerSwitch = 1; // turn off
            }

            toggleBlinker = 1;
         }
      }

      else {
         toggleBlinker = 0; // turn off
      }

      setBlinkLED(blinkerSwitch);

      // set next velocity (accel, brake, uWave?)
      // weight : uWave > brake > accel > idle
      if (lastStepTime < time) {
         if ((uWaveDistance < 11 && velocity > 0)) { // uWave detect
            brakeLED = 1;
            Buzzer_flag = 10;
            velocity -= 1; // -100 per sec
         }

         else if ((GPIOA_PDIR & (1 << PTA3)) == 0) { // brake
            brakeLED = 1;
            velocity -= 1; // -100 per sec
         }

         else if ((GPIOA_PDIR & (1 << PTA2)) == 0) { // accel
        	 if(GEARSET > P)
        	 {
                 brakeLED = 0;
                 velocity += ((MaxSpd - velocity) * 0.0025); // logScale acceleration
        	 }
        	 else
        	 {
        		 //
        	 }
         }

         else { // idle
            brakeLED = 0;
            velocity -= velocity * 0.0003; // -3% per sec
         }



         if (velocity < 0) { // clamping velocity (0 ~ MaxSpd)
            velocity = 0;
         }

         else if (velocity > MaxSpd) { // clamping velocity (0 ~ MaxSpd)
            velocity = MaxSpd;
         }

         Buzzer_flag -= 1;
         lastStepTime = time; // check time change
      }

      setBrakeLED(brakeLED);


      // show 7Seg velocity
      displayDigits((int)velocity, GEARSET);

   }

}

/*-----------------------FUNCTION------------------------*/
void setHandle(int handleAngle) {
	if((handleAngle > 3072 || handleAngle < 1024) && (velocity > 80))
		{
		BUZZER_ALERT_SET;
		}
	FTM0_C1V = (int)(MAX_HANDLE_LEFT + (MAX_HANDLE_RIGHT - MAX_HANDLE_LEFT)*(handleAngle)/4095);
}

void setBlinkLED(int blinkerSwitch) {
   if (blinkerSwitch == 1) {
      GPIOE_PSOR |= (1 << PTE0) | (1 << PTE1) | (1 << PTE6) | (1 << PTE7);
   }

   else if ((time - blinkerTime) % 50 < 25) { // turn on
      if (blinkerSwitch == 0) {
         GPIOE_PSOR |= (1 << PTE0) | (1 << PTE1);
         GPIOE_PCOR |= (1 << PTE6) | (1 << PTE7);
      }

      else {
         GPIOE_PSOR |= (1 << PTE6) | (1 << PTE7);
         GPIOE_PCOR |= (1 << PTE0) | (1 << PTE1);
      }
   }

   else {
      GPIOE_PSOR |= (1 << PTE0) | (1 << PTE1) | (1 << PTE6) | (1 << PTE7);
   }
}

void setBrakeLED(int brakeLED) {
   if (brakeLED == 0) { // brake initiate
      GPIOE_PSOR |= (1 << PTE2) | (1 << PTE3) | (1 << PTE8) | (1 << PTE5);
   }

   else { // no brake
      GPIOE_PCOR |= (1 << PTE2) | (1 << PTE3) | (1 << PTE8) | (1 << PTE5);
   }
}


void setDCmotorGear(int velocity) {
	if(FORWARD_CHECK) {
		FTM2_C2V = (int)velocity*210;
		FTM2_C5V = 0;
	}
	else if (BACKWARD_CHECK) {
		FTM2_C5V = (int)velocity*210;
		FTM2_C2V = 0;
	}
}

void Buzzer_set() {
	if(Buzzer_flag > 0){
		GPIOB_PSOR |= (1 << PTB17);
	}

	else {
		GPIOB_PCOR |= (1 << PTB17);
	}
}

/*-----------------------uWave_Sensor------------------------*/
void uWaveSensor_10us() // Called 10us
{
    if (GEAR_R || GEAR_D)
    {
        if (uWaveCount == 0)
            GPIOA_PSOR |= (1 << PTA8);
        else if (uWaveCount == 1)
            GPIOA_PCOR |= (1 << PTA8);


        if (GPIOA_PDIR & (1 << PTA7))
            uWaveLevelHigh++;


		uWaveCount++;


        if (uWaveCount >= 10000) //100ms
        {
            uWaveCount = 0;
            uWaveDistance = (int)uWaveLevelHigh*0.17;
            uWaveLevelHigh = 0;
        }
    }
}
/*-----------------------uWave_Sensor------------------------*/

/*-----------------------FUNCTION------------------------*/
