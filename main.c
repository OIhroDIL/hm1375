#include <iom48p.h>

//#define byte unsigned char;
#define SCL PINC5
#define SDA PINC4
#define RESET PINC3
#define LED_ON PINC2
#define IN_CONN PIND3
#define MSB 0
#define LSB 1
#define WR 0x24
#define RD 0x34
#define HM1375_ADDR 0x48


#define nop() asm("nop")
/*  USER CONFIGURATION  */
#define C_STEP        6  // DEFAULT 0x40
#define R_STEP        4  // DEFAULT 4(0x014a) 1STEP:0x10
#define G_STEP        4  // DEFAULT 4(0x00f0) 1STEP:0x10
#define B_STEP        4  // DEFAULT 4(0x01b2) 1STEP:0x10
#define MIRROR_MODE   1  //(0:NORMAL, 1:MIRROR)
#define IO_STEP       1  // DEFAULT 3

#define R_REG_L   0x0a+(R_STEP*0x10)
#define R_REG_H   0x01
#define G_REG_L   G_STEP>=5?(0x00+(G_STEP-5)*0x10):0xB0+(G_STEP*0x10)
#define G_REG_H   G_STEP>=5?0x01:0x00
#define B_REG_L   B_STEP>=9?(0x00+(B_STEP-9)*0x10):0x72+(B_STEP*0x10)
#define B_REG_H   B_STEP>=9?0x02:0x01
#define M_REG     MIRROR_MODE == 0? 0x00:0x0a
#define C_GAIN    0x20+(C_STEP*0x08)

/*              IO CONTROL                        */
/* 40: 3.8mA, 41: 7.7mA, 42: 11.5mA, 43: 15.3mA  */
/* Be careful each SENSOR LOT 1st: 43(Normal) 40(frame down)
                              2nd: 43(horizontal noise), 40(Normal) */
#define IOCNTR    0x40|IO_STEP

unsigned char ucNack=0;
unsigned char state=1;
void init_twi();
void start_twi();
void start_repeated_twi();
void write_1byte_twi(unsigned char data);
void stop_twi();
unsigned char read_1byte_twi();
void write_reg(unsigned int add, unsigned char data);
void sensor_init(void);
void nops(unsigned char num)
{
  unsigned char i;
  for(i=0;i<num;i++) {
    nop();
  }
}

void Delay_us(unsigned char time_us)
{
  unsigned char i;
  for(i=0;i<time_us;i++) {
    asm(" PUSH R0 ");
    asm(" POP R0 ");
    asm(" PUSH R0 ");
    asm(" POP R0 ");
    asm(" PUSH R0 ");
    asm(" POP R0 ");
//    nop();
  }
}

void Delay_ms(unsigned char time_ms)
{
  unsigned char i;
  for(i=0;i<time_ms;i++) {
    Delay_us(250);
    Delay_us(250);
    Delay_us(250);
    Delay_us(250);
    Delay_us(250);
    Delay_us(250);
  }
}

       
         
void Detect_sw()
{
   if(((PIND &= 0x08)!= 0x08)&&(state == 0))//camera on
    {
      write_reg(0x0004, 0x00);
//      write_reg(0x0005, 0x01);
      sensor_init();
      PORTC |= 0x0c;
      state = 0x01; //ative cam
    }
    else if(((PIND &= 0x08)== 0x08)&&(state == 1))//camera off
    {
      write_reg(0x0028, 0x80);
      write_reg(0x0110, 0x80);
      Delay_ms(160);
      write_reg(0x0004, 0x06); 
      PORTC &= 0xf3;
      state = 0x00; //non ative cam
    } 
    else ;
}
void Detect_snap()
{
   if((PIND &= 0x02)!= 0x02)//snap on
    {
      PORTD &= 0xfe;
    }
    else if((PIND &= 0x02)== 0x02)//snap off
    {
      PORTD |= 0x01;
    } 
    else ;
}     

void port_init(void)
{
  DDRB = 0xff;
  DDRC |= 0x0f;  // PC2: LED ON, PC3: Sensor Reset
  DDRD = 0xf5;  // PD3: Magnetic sensor Input
  PORTC = 0x08;
}

/////////////////////////////////
// 1. Set SCL to400KHz for i2c //
/////////////////////////////////
void init_twi(void)
{
  TWBR = 12; //SCL = CPU Clock /(16+2xTWBR*Prescaler value) = 8000000Hz/(16+2x2x1) = 200KHz
  TWSR = (0<<TWPS1)|(0<<TWPS0); //Prescaler =1
}

void start_twi(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while(!(TWCR&(1<<TWINT)));
  while((TWSR&0xF8)!= 0x08);
}

void start_repeated_twi(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while(!(TWCR&(1<<TWINT)));
  while((TWSR&0xF8)!= 0x10);
} 

void write_1byte_twi(unsigned char data)
{
  TWDR = data;
  TWCR=(1<<TWINT)|(1<<TWEN);
  while(!(TWCR&(1<<TWINT)));
  while((TWSR&0xF8) != 0x28);
}
void write_addr_twi(unsigned char data)
{
  TWDR = data;
  TWCR=(1<<TWINT)|(1<<TWEN);
  while(!(TWCR&(1<<TWINT)));
  while((TWSR&0xF8) != 0x18);
}

unsigned char read_1byte_twi(void)
{
  TWCR = (1<<TWINT)|(1<<TWEN);
  while(!(TWCR&(1<<TWINT)));
  while((TWSR &0xF8) != 0x58);
  return TWDR;
}

void stop_twi(void)
{
  TWCR= (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
  while(!(TWCR &(1<<TWSTO)));
}

void write_reg(unsigned int add, unsigned char data)
{
  unsigned char ADDRH, ADDRL;
  ADDRH = (unsigned char)(add>>8);
  ADDRL = (unsigned char)(add&0x00ff);
  start_twi();
  write_addr_twi(HM1375_ADDR);
  write_1byte_twi(ADDRH);
  write_1byte_twi(ADDRL);  
  write_1byte_twi(data);
  stop_twi();
}

void sensor_init()
{
//////////////////////////////
// Sensor Initialization  //
//////////////////////////////
  write_reg(0x0004, 0x00);  // PWDCNTR : Hardware Reset
  write_reg(0x0022, 0x00);  // SFTSRT : Soft Reset
  write_reg(0x000c, 0x04);  // MODE : Reserved bit
  write_reg(0x0006, M_REG);  // UBMT Flip Mirror       0a: Mirror, 00:Normal
  write_reg(0x000a, 0x00);  // CHIPCFG : Full frame, chip default 
  write_reg(0x000f, 0x18);  // IMGCFG : Select fixed frame rate
  write_reg(0x0010, 0x00);  // Reserved
  write_reg(0x0011, 0x18);  // Reserved
  write_reg(0x0012, 0x01);  // Reserved
  write_reg(0x0013, 0x02);  // Reserved
  write_reg(0x0015, 0x01);  // INTG_H : Set up integration
  write_reg(0x0016, 0x00);  // INTG_L : Set up integration
  write_reg(0x0018, 0x00);  // AGAIN : Set up coarse gain
  write_reg(0x001d, 0x40);  // DGGAIN : Set up fine gain
  
  write_reg(0x0020, 0x10);
 // OPRTCFG : AE gain enabled, PCLK transition on falling edge
  write_reg(0x0023, IOCNTR);  // IOCNTR : IO pad drive strength
  write_reg(0x0024, 0x20);  // CKCNTR : Reserved bit
  write_reg(0x0025, 0x00);  // CKCFG : Select PLL clock -> 24MHz to 78MHz
  write_reg(0x0026, 0x6c);  
  write_reg(0x0027, 0x30);  // OPORTCNTR : YUV \output, ITU601 Disable
  write_reg(0x0028, 0x01);  // CKMUL : Reserved bit
  write_reg(0x0030, 0x00);  // EDRCFG : Enable EDR mode
  write_reg(0x0034, 0x0e);  // EDRBLEND : Set EDR blend to 0.875
  write_reg(0x0035, 0x01);  // Reserved
  write_reg(0x0036, 0x00);  // Reserved
  write_reg(0x0038, 0x02);  // Reserved
  write_reg(0x0039, 0x00);  // Reserved
  write_reg(0x003a, 0x00);  // Reserved
  write_reg(0x003b, 0x00);  // Reserved
  write_reg(0x003c, 0x00);  // Reserved
  write_reg(0x003d, 0x40);  // EDR2DGAIN : Set up EDR2 fine gain
  write_reg(0x003f, 0x14);  // Reserved : Less gradient effect
  
/////////////////////////
// BLACK LEVEL CONTROL //  
/////////////////////////
  write_reg(0x0040, 0x10);  // BLCTGT : Black level target
  write_reg(0x0044, 0x07);  // BLCCFG : BLC configuration enable, reserved bit
  
///////////////////////////////////////////////////
// RESERVED REGISTERS FOR SENSOR READOUT TIMING  //
///////////////////////////////////////////////////
  write_reg(0x0045, 0x35);  // Reserved
  write_reg(0x0048, 0x7f);  // Reserved 
  write_reg(0x004e, 0xff);  // Reserved
  write_reg(0x0070, 0x22);  // Reserved : 1024 updated by willie 4 s-2
  write_reg(0x0071, 0x3f);  // Reserved
  write_reg(0x0072, 0x22);  // Reserved
  write_reg(0x0073, 0x30);  // Reserved
  write_reg(0x0074, 0x13);  // Reserved
  write_reg(0x0075, 0x40);  // Reserved
  write_reg(0x0076, 0x24);  // Reserved
  write_reg(0x0078, 0x0f);  // Reserved
  write_reg(0x007a, 0x06);  // Reserved
  write_reg(0x007b, 0x14);  // Reserved
  write_reg(0x007c, 0x10);  // Reserved
  write_reg(0x0080, 0xc9);  // Reserved
  write_reg(0x0081, 0x00);
  write_reg(0x0082, 0x28);
  write_reg(0x0083, 0xb0);
  write_reg(0x0084, 0x60);
  write_reg(0x0086, 0x3e);
  write_reg(0x0087, 0x70);
  write_reg(0x0088, 0x11);
  write_reg(0x0089, 0x3c);
  write_reg(0x008a, 0x87);
  write_reg(0x008d, 0x64);
  write_reg(0x0090, 0x07);
  write_reg(0x0091, 0x09);
  write_reg(0x0092, 0x0c);
  write_reg(0x0093, 0x0c);
  write_reg(0x0094, 0x0c);
  write_reg(0x0095, 0x0c);
  write_reg(0x0096, 0x01);
  write_reg(0x0097, 0x00);
  write_reg(0x0098, 0x04);
  write_reg(0x0099, 0x08);
  write_reg(0x009a, 0x0c);
  
  ////////////////////////////////////////
  //  IMAGE PIPELINE PROCESSING CONTROL //
  ////////////////////////////////////////
  write_reg(0x0120, 0x37);
  write_reg(0x0121, 0x81);
  write_reg(0x0122, 0xeb);
  write_reg(0x0123, 0x29);
//  write_reg(0x0123, 0x39);
  write_reg(0x0124, 0x50);
//  write_reg(0x0125, 0xdf);
  write_reg(0x0125, 0xde);
  write_reg(0x0126, 0xb1);

///////////////////////
// FLARE CORRECTION  //
///////////////////////
  write_reg(0x013d, 0x03);  // HID 
  write_reg(0x013e, 0x0f);
  write_reg(0x013f, 0x0f);
  
///////////////////////
// BAD PIXEL CONTROL //
///////////////////////
  write_reg(0x0140, 0x14);
  write_reg(0x0141, 0x0a);
  write_reg(0x0142, 0x14);
  write_reg(0x0143, 0x0a);
  
//////////////
// RESERVED //
//////////////
  write_reg(0x0144, 0x08);
  write_reg(0x0145, 0x04);
  write_reg(0x0146, 0x28);
  write_reg(0x0147, 0x3c);
  write_reg(0x0148, 0x28);
  write_reg(0x0149, 0x3c);
  write_reg(0x014a, 0x96);
  write_reg(0x014b, 0xc8);

///////////////////////
// SHARPNESS CONTROL //
///////////////////////
  write_reg(0x0150, 0x14);
  write_reg(0x0151, 0x30);
  write_reg(0x0152, 0x54);
  write_reg(0x0153, 0x70);
  write_reg(0x0154, 0x14);
  write_reg(0x0155, 0x30);
  write_reg(0x0156, 0x54);
  write_reg(0x0157, 0x70);
  write_reg(0x0158, 0x14);
  write_reg(0x0159, 0x30);
  write_reg(0x015a, 0x54);
  write_reg(0x015b, 0x70);
  
//////////////////
// NOISE FILTER //
//////////////////
  write_reg(0x01d8, 0x20);
  write_reg(0x01d9, 0x08);
  write_reg(0x01da, 0x20);
  write_reg(0x01db, 0x08);
  write_reg(0x01dc, 0x20);
  write_reg(0x01dd, 0x08);
  write_reg(0x01de, 0x50);
  write_reg(0x01df, 0x14);
  write_reg(0x01e0, 0x50);
  write_reg(0x01e1, 0x14);
  write_reg(0x01e2, 0x50);
  write_reg(0x01e3, 0x14);
  write_reg(0x01e4, 0x10);
  write_reg(0x01e5, 0x10);
  write_reg(0x01e6, 0x02);
  write_reg(0x01e7, 0x10);
  write_reg(0x01e8, 0x10);
  write_reg(0x01e9, 0x10);
  write_reg(0x01ea, 0x28);
  write_reg(0x01eb, 0x28);
  write_reg(0x01ec, 0x28);
  
/////////////////////////////
// LENS SHADING CORRECTION //
/////////////////////////////
  write_reg(0x0220, 0x00);
  write_reg(0x0221, 0x4a);
  write_reg(0x0222, 0x03);
  write_reg(0x0223, 0x80);
  write_reg(0x0224, 0x4a);
  write_reg(0x0225, 0x03);
  write_reg(0x0226, 0x80);
  write_reg(0x0227, 0x4a);
  write_reg(0x0228, 0x03);
  write_reg(0x0229, 0x80);
  write_reg(0x022a, 0x4a);
  write_reg(0x022b, 0x03);
  write_reg(0x022c, 0x80);
  write_reg(0x022d, 0x12);
  write_reg(0x022e, 0x09);
  write_reg(0x022f, 0x12);
  write_reg(0x0230, 0x09);
  write_reg(0x0231, 0x12);
  write_reg(0x0232, 0x09);
  write_reg(0x0233, 0x12);
  write_reg(0x0234, 0x09);
  write_reg(0x0235, 0x89);
  write_reg(0x0236, 0x02);
  write_reg(0x0237, 0x89);
  write_reg(0x0238, 0x02);
  write_reg(0x0239, 0x89);
  write_reg(0x023a, 0x02);
  write_reg(0x023b, 0x89);
  write_reg(0x023c, 0x02);
  write_reg(0x023d, 0x15);
  write_reg(0x023e, 0x02);
  write_reg(0x023f, 0x15);
  write_reg(0x0240, 0x02);
  write_reg(0x0241, 0x15);
  write_reg(0x0242, 0x02);
  write_reg(0x0243, 0x15);
  write_reg(0x0244, 0x02);
  
  write_reg(0x0251, 0x07);
  write_reg(0x0252, 0x03);
  
  ///////////////////
  // GAMMA CONTROL //
  ///////////////////
  write_reg(0x0280, 0x00);
  write_reg(0x0281, 0x47);
  write_reg(0x0282, 0x00);
  write_reg(0x0283, 0x7F);
  write_reg(0x0284, 0x00);
  write_reg(0x0285, 0xd7);
  write_reg(0x0286, 0x01);
  write_reg(0x0287, 0x6a);
  write_reg(0x0288, 0x01);
  write_reg(0x0289, 0x9c);
  write_reg(0x028a, 0x01);
  write_reg(0x028b, 0xca);
  write_reg(0x028c, 0x01);
  write_reg(0x028d, 0xee);
  write_reg(0x028e, 0x02);
  write_reg(0x028f, 0x13);
  
  write_reg(0x0290, 0x02);
  write_reg(0x0291, 0x34);
  write_reg(0x0292, 0x02);
  write_reg(0x0293, 0x52);
  write_reg(0x0294, 0x02);
  write_reg(0x0295, 0x8a);
  write_reg(0x0296, 0x02);
  write_reg(0x0297, 0xbc);
  write_reg(0x0298, 0x03);
  write_reg(0x0299, 0x15);
  write_reg(0x029a, 0x03);
  write_reg(0x029b, 0x61);
  write_reg(0x029c, 0x03);
  write_reg(0x029d, 0xa3);
  write_reg(0x029e, 0x00);
  write_reg(0x029f, 0x7a);
  
  write_reg(0x02a0, 0x04);
  
  /////////////////////////////
  // COLOR CORRECTION MATRIX //
  /////////////////////////////
  
  write_reg(0x02c0, 0x4f);
  write_reg(0x02c1, 0x01);
  write_reg(0x02c2, 0x2c);
  write_reg(0x02c3, 0x04);
  write_reg(0x02c4, 0x22);
  write_reg(0x02c5, 0x04);
  write_reg(0x02c6, 0x27);
  write_reg(0x02c7, 0x04);
  write_reg(0x02c8, 0x3c);
  write_reg(0x02c9, 0x01);
  write_reg(0x02ca, 0x15);
  write_reg(0x02cb, 0x04);
  write_reg(0x02cc, 0x14);
  write_reg(0x02cd, 0x00);
  write_reg(0x02ce, 0xe7);
  write_reg(0x02cf, 0x04);
  
  write_reg(0x02d0, 0xd3);
  write_reg(0x02d1, 0x01);
  
  write_reg(0x02e0, 0x04);
  write_reg(0x02e1, 0xc0);
  write_reg(0x02e2, 0xe0);
  
  write_reg(0x02f0, 0x48);
  write_reg(0x02f1, 0x01);
  write_reg(0x02f2, 0x32);
  write_reg(0x02f3, 0x04);
  write_reg(0x02f4, 0x16);
  write_reg(0x02f5, 0x04);
  write_reg(0x02f6, 0x52);
  write_reg(0x02f7, 0x04);
  write_reg(0x02f8, 0xaa);
  write_reg(0x02f9, 0x01);
  write_reg(0x02fa, 0x58);
  write_reg(0x02fb, 0x04);
  write_reg(0x02fc, 0x56);
  write_reg(0x02fd, 0x04);
  write_reg(0x02fe, 0xdd);
  write_reg(0x02ff, 0x04);
  
  write_reg(0x0300, 0x33);
  write_reg(0x0301, 0x02);
  
  ////////////////////////////////////////////
  // AUTOMATIC WHITE BALANCE WINDOW CONTROL //
  ////////////////////////////////////////////
  write_reg(0x0324, 0x00);
  write_reg(0x0325, 0x01);
  
   //////////////////////////////////////////////////
  // MANUAL WHITE BALANCE GAIN//
  //////////////////////////////////////////////////
  write_reg(0x032d, R_REG_L);  //R gain (4a 01)
  write_reg(0x032e, R_REG_H);  
  write_reg(0x032f, G_REG_L);  //G gain (f0 00)
  write_reg(0x0330, G_REG_H);
  write_reg(0x0331, B_REG_L);  //B gain (b2 01)
  write_reg(0x0332, B_REG_H);
  
  //////////////////////////////////////////////////
  // AUTOMATIC WHITE BALANCE DETECTION AND LIMITS //
  //////////////////////////////////////////////////
  write_reg(0x0333, 0x85);
  write_reg(0x0334, 0x00);
  write_reg(0x0335, 0x85);
  
  write_reg(0x0340, 0x40);
  write_reg(0x0341, 0x44);
  write_reg(0x0342, 0x4a);
  write_reg(0x0343, 0x2b);
  write_reg(0x0344, 0x94);
  write_reg(0x0345, 0x3f);
  write_reg(0x0346, 0x8e);
  write_reg(0x0347, 0x51);
  write_reg(0x0348, 0x75);
  write_reg(0x0349, 0x5c);
  write_reg(0x034a, 0x6a);
  write_reg(0x034b, 0x68);
  write_reg(0x034c, 0x5e);
  
  write_reg(0x0350, 0x7c);
  write_reg(0x0351, 0x78);
  write_reg(0x0352, 0x08);
  write_reg(0x0353, 0x04);
  write_reg(0x0354, 0x80);
  write_reg(0x0355, 0x9a);
  write_reg(0x0356, 0xcc);
  write_reg(0x0357, 0xff);
  write_reg(0x0358, 0xff);
  write_reg(0x035a, 0xff);
  write_reg(0x035b, 0x00);
  write_reg(0x035c, 0x70);
  write_reg(0x035d, 0x80);
  write_reg(0x035f, 0xa0);
  write_reg(0x0488, 0x30);
  write_reg(0x0360, 0xdf);
  write_reg(0x0361, 0x00);
  write_reg(0x0362, 0xff);
  write_reg(0x0363, 0x03);
  write_reg(0x0364, 0xff);
  
  write_reg(0x037b, 0x11);
  write_reg(0x037c, 0x1e);
  
  //////////////////////////////////////
  // AUTOMATIC EXPOSURE CONFIGURATION //
  //////////////////////////////////////
  write_reg(0x0380, 0xfd);
  write_reg(0x0383, 0x50);
  write_reg(0x038a, 0x64);
  write_reg(0x038b, 0x64);
  write_reg(0x038c, 0x03);
  write_reg(0x038e, 0x32);
  
  write_reg(0x0391, 0x2a);
  write_reg(0x0393, 0x1e);
  write_reg(0x0394, 0x64);
  write_reg(0x0395, 0x23);
  write_reg(0x0398, 0x03);
  write_reg(0x0399, 0x45);
  write_reg(0x039a, 0x06);
  write_reg(0x039b, 0x8b);
  write_reg(0x039c, 0x0d);
  write_reg(0x039d, 0x16);
  write_reg(0x039e, 0x0a);
  write_reg(0x039f, 0x12);
  
  write_reg(0x03a0, 0x10);
  write_reg(0x03a1, 0xe5);
  write_reg(0x03a2, 0x06);
  write_reg(0x03a4, 0x11);
  write_reg(0x03a5, 0x4f);
  write_reg(0x03a6, 0x2d);
  write_reg(0x03a7, 0x78);
  write_reg(0x03ac, 0x5a);
  write_reg(0x03ad, 0x0f);
  write_reg(0x03ae, 0x7f);
  write_reg(0x03af, 0x04);
  write_reg(0x03b0, 0x35);
  write_reg(0x03b1, 0x14);
  
  write_reg(0x036f, 0x05);
  write_reg(0x0370, 0x08);
  write_reg(0x0371, 0x03);
  write_reg(0x0372, 0x0a);
  write_reg(0x0373, 0x60);
  write_reg(0x0374, 0x61);
  write_reg(0x0375, 0x04);
  write_reg(0x0376, 0x06);
  write_reg(0x0377, 0x01);
  write_reg(0x0378, 0x08);
  write_reg(0x0379, 0x01);
  write_reg(0x037a, 0x0a);
  
  ////////////////////////////////////////
  // DIGITAL BLACK LEVEL OFFSET CONTROL //
  ////////////////////////////////////////
  write_reg(0x0420, 0x00);
  write_reg(0x0421, 0x00);
  write_reg(0x0422, 0x00);
  write_reg(0x0423, 0x81);
  
  //////////////////////
  // AUTO BLACK LEVEL //
  //////////////////////
  write_reg(0x0430, 0x00);
  write_reg(0x0431, 0x60);
  write_reg(0x0432, 0x10);
  write_reg(0x0433, 0x00);
  write_reg(0x0434, 0x00);
  write_reg(0x0435, 0x30);
  write_reg(0x0436, 0x00);
  
  /////////////////////////////////
  // LOWLIGHT OUTDOOR IPP CONTROL //
  //////////////////////////////////
  write_reg(0x0450, 0xfd);
  write_reg(0x0451, 0xdc);
  write_reg(0x0452, 0xb3);
  write_reg(0x0453, 0x81);
  write_reg(0x0454, 0x43);
  write_reg(0x0455, 0x00);
  write_reg(0x0456, 0x00);
  write_reg(0x0457, 0x00);
  write_reg(0x0459, 0x04);
  write_reg(0x045a, 0x00);
  write_reg(0x045b, 0x30);
  write_reg(0x045c, 0x01);
  write_reg(0x045d, 0x70);
  write_reg(0x0460, 0x00);
  write_reg(0x0461, 0x00);
  write_reg(0x0462, 0x00);
  write_reg(0x0465, 0x83);
  write_reg(0x0466, 0x82);
  write_reg(0x0478, 0x00);
  
  ///////////////////////////////////////////
  // COLOR SPACE CONVERSION SATURATION ADJ //
  ///////////////////////////////////////////
  write_reg(0x0480, C_GAIN);
  write_reg(0x0481, 0x04);
  write_reg(0x0482, 0x0a);
  write_reg(0x0486, 0x00);  
  write_reg(0x0487, 0xff);  
  
  /////////////////////////
  // CONTRAST BRIGHTNESS //
  /////////////////////////
  write_reg(0x04b0, 0x53);
  write_reg(0x04b1, 0x83);
  write_reg(0x04b2, 0x00);
  write_reg(0x04b3, 0x00);
  write_reg(0x04b4, 0x00);
  write_reg(0x04b5, 0x00);
  write_reg(0x04b6, 0x3a);
  write_reg(0x04b7, 0x00);
  write_reg(0x04b8, 0x00);
  write_reg(0x04b9, 0x19);
  write_reg(0x04ba, 0x00);
  write_reg(0x04bb, 0x00);
  write_reg(0x04bc, 0x00); 
  write_reg(0x04bd, 0x6a);
  write_reg(0x04c0, 0x00);
  write_reg(0x04c1, 0x12);
  
  //////////////////
  // EDR CONTRAST //
  //////////////////
  write_reg(0x04d0, 0x56);
  write_reg(0x04d6, 0x30);
  write_reg(0x04dd, 0x10);
  write_reg(0x04d9, 0x16);
  write_reg(0x04d3, 0x18);
  
  //////////////////////////
  // AE FLICKER STEP SIZE //
  //////////////////////////
  write_reg(0x0540, 0x00);
  write_reg(0x0541, 0xd0);
  write_reg(0x0542, 0x00);
  write_reg(0x0543, 0xfa);
  write_reg(0x0580, 0x50);
  write_reg(0x0581, 0x30);
  
  /////////////////////////////
  // Y COLOR NOISE REDUCTION //
  /////////////////////////////
  write_reg(0x0582, 0x2d);
  write_reg(0x0583, 0x16);
  write_reg(0x0584, 0x1e);
  write_reg(0x0585, 0x0f);
  write_reg(0x0586, 0x08);
  write_reg(0x0587, 0x10);
  write_reg(0x0590, 0x10);
  write_reg(0x0591, 0x10);
  write_reg(0x0592, 0x05);
  write_reg(0x0593, 0x05);
  write_reg(0x0594, 0x04);
  write_reg(0x0595, 0x06);
  
  //////////////////////////
  // Y SHARPNESS STRENGTH //
  //////////////////////////
  write_reg(0x05b0, 0x0c);
  write_reg(0x05b1, 0x06);
  write_reg(0x05b2, 0x03);
  write_reg(0x05b3, 0x04);
  write_reg(0x05b4, 0x05);
  write_reg(0x05b5, 0x06);
  write_reg(0x05b6, 0x11);
  write_reg(0x05b7, 0x0a);
  write_reg(0x05b8, 0x19);
  write_reg(0x05b9, 0x29);
  write_reg(0x05ba, 0x38);
  write_reg(0x05bb, 0x19);
  write_reg(0x05bc, 0xa0);
  write_reg(0x05bd, 0xa0);
  write_reg(0x05be, 0x0a);
  write_reg(0x05bf, 0x00);
  write_reg(0x05c0, 0x5a);
  write_reg(0x05c1, 0x17);
  write_reg(0x05c2, 0xf8);
  
  ///////////////////
  // WINDOW SCALER //
  ///////////////////
  write_reg(0x05e4, 0x08);
  write_reg(0x05e5, 0x00);
  write_reg(0x05e6, 0x07);
  write_reg(0x05e7, 0x05);
  write_reg(0x05e8, 0x0a);
  write_reg(0x05e9, 0x00);
  write_reg(0x05ea, 0xd9);
  write_reg(0x05eb, 0x02);
  
  ////////////////////////////////////
  // FLEXI ENGINE AE ADJUST CONTROL //
  ////////////////////////////////////
  write_reg(0x0666, 0x02);
  write_reg(0x0667, 0xe0);
  write_reg(0x067f, 0x19);
  write_reg(0x067c, 0x00);
  write_reg(0x067d, 0x00);
  write_reg(0x0682, 0x00);
  write_reg(0x0683, 0x00);
  write_reg(0x0688, 0x00);
  write_reg(0x0689, 0x00);
  write_reg(0x068e, 0x00);
  write_reg(0x068f, 0x00);
  write_reg(0x0694, 0x00);
  write_reg(0x0695, 0x00);
  write_reg(0x0697, 0x19);
  write_reg(0x069b, 0x00);
  write_reg(0x069c, 0x48);
  
  write_reg(0x0720, 0x00);
  write_reg(0x0725, 0x6a);
  write_reg(0x0726, 0x03);
  write_reg(0x072b, 0x64);
  write_reg(0x072c, 0x64);
  write_reg(0x072d, 0x20);
  write_reg(0x072e, 0x82);
  write_reg(0x072f, 0x08);
  
  write_reg(0x0800, 0x16);
  write_reg(0x0801, 0x30);
  write_reg(0x0802, 0x00);
  write_reg(0x0803, 0x67);
  write_reg(0x0804, 0x01);
  write_reg(0x0805, 0x28);
  write_reg(0x0806, 0x10);
  write_reg(0x0808, 0x1d);
  write_reg(0x0809, 0x18);
  write_reg(0x080a, 0x23);
  write_reg(0x080b, 0x07);
  write_reg(0x080d, 0x0f);
  write_reg(0x080e, 0x0f);
  write_reg(0x0810, 0x00);
  write_reg(0x0811, 0x08);
  write_reg(0x0812, 0x20);
  write_reg(0x0857, 0x1a);
  write_reg(0x0858, 0x04);
  write_reg(0x0859, 0x01);
  write_reg(0x085a, 0x03);
  write_reg(0x085b, 0x40);
  write_reg(0x085c, 0x03);
  write_reg(0x085d, 0x7f);
  write_reg(0x085e, 0x02);
  write_reg(0x085f, 0xd0);
  write_reg(0x0860, 0x03);
  write_reg(0x0861, 0x7f);
  write_reg(0x0862, 0x02);
  write_reg(0x0863, 0xd0);
  write_reg(0x0864, 0x02);
  write_reg(0x0865, 0x7f);
  write_reg(0x0866, 0x05);
  write_reg(0x0867, 0x00);
  write_reg(0x0868, 0x40);
  write_reg(0x0869, 0x01);
  write_reg(0x086a, 0x00);
  write_reg(0x086b, 0x40);
  write_reg(0x086c, 0x01);
  write_reg(0x086d, 0x00);
  write_reg(0x086e, 0x40); 

// AE WINDOW YYKIM  
  write_reg(0x0870, 0x00);
  write_reg(0x0871, 0x00);
  write_reg(0x0872, 0x01);
  write_reg(0x0873, 0x40);
  write_reg(0x0874, 0x00);
  write_reg(0x0875, 0x00);//9.12 52
  write_reg(0x0876, 0x01);
  write_reg(0x0877, 0x00); //9.12 93
  
  ////////////////////////////////////////
  // FLEXI ENGINE GAMMA FOR MAXIMUM EDR //
  ////////////////////////////////////////
  write_reg(0x0815, 0x00);
  write_reg(0x0816, 0x4c);
  write_reg(0x0817, 0x00);
  write_reg(0x0818, 0x7b);
  write_reg(0x0819, 0x00);
  write_reg(0x081a, 0xca);
  write_reg(0x081b, 0x01);
  write_reg(0x081c, 0x3e);
  write_reg(0x081d, 0x01);
  write_reg(0x081e, 0x77);
  write_reg(0x081f, 0x01);
  write_reg(0x0820, 0xaa);
  write_reg(0x0821, 0x01);
  write_reg(0x0822, 0xce);
  write_reg(0x0823, 0x01);
  write_reg(0x0824, 0xee);
  write_reg(0x0825, 0x02);
  write_reg(0x0826, 0x16);
  write_reg(0x0827, 0x02);
  write_reg(0x0828, 0x33);
  write_reg(0x0829, 0x02);
  write_reg(0x082a, 0x65);
  write_reg(0x082b, 0x02);
  write_reg(0x082c, 0x91);
  write_reg(0x082d, 0x02);
  write_reg(0x082e, 0xdc);
  write_reg(0x082f, 0x03);
  write_reg(0x0830, 0x28);
  write_reg(0x0831, 0x03);
  write_reg(0x0832, 0x74);
  write_reg(0x0833, 0x03);
  write_reg(0x0834, 0xff); 
  
  ////////////////////////////////////////
  // FLEXI ENGINE GAMMA FOR MINIMUM EDR //
  ////////////////////////////////////////
  write_reg(0x0882, 0x00);
  write_reg(0x0883, 0x3e);
  write_reg(0x0884, 0x00);
  write_reg(0x0885, 0x70);
  write_reg(0x0886, 0x00);
  write_reg(0x0887, 0xb8);
  write_reg(0x0888, 0x01);
  write_reg(0x0889, 0x28);
  write_reg(0x088a, 0x01);
  write_reg(0x088b, 0x5b);
  write_reg(0x088c, 0x01);
  write_reg(0x088d, 0x8a);
  write_reg(0x088e, 0x01);
  write_reg(0x088f, 0xb1);
  write_reg(0x0890, 0x01);
  write_reg(0x0891, 0xd9);
  write_reg(0x0892, 0x01);
  write_reg(0x0893, 0xee);
  write_reg(0x0894, 0x02);
  write_reg(0x0895, 0x0f);
  write_reg(0x0896, 0x02);
  write_reg(0x0897, 0x4c);
  write_reg(0x0898, 0x02);
  write_reg(0x0899, 0x74);
  write_reg(0x089a, 0x02);
  write_reg(0x089b, 0xc3);
  write_reg(0x089c, 0x03);
  write_reg(0x089d, 0x0f);
  write_reg(0x089e, 0x03);
  write_reg(0x089f, 0x57);
  write_reg(0x08a0, 0x03);
  write_reg(0x08a1, 0xff); 
  
  //////////////////////
  // ENABLE AE ADJUST //
  //////////////////////
  write_reg(0x038a, 0x67);
  write_reg(0x038b, 0x67);
  write_reg(0x0698, 0x00);
  write_reg(0x0699, 0x00);
  write_reg(0x069a, 0x00);
  write_reg(0x069b, 0x47);
  
  ////////////////////////////
  // COMMAND UPDATE TRIGGER //
  ////////////////////////////
  write_reg(0x0100, 0x01);  // CMU AE
  write_reg(0x0101, 0x01);  // CMU AWB
  write_reg(0x0000, 0x01);  // CMU
  write_reg(0x002C, 0x00);  // RESET 8051
  write_reg(0x0005, 0x01);  // Trigger 
  DDRC &= 0xcf;  // Set SDA&SCL to Input
}
 
void main()
{
  Delay_ms(100);
  Delay_ms(100);
  init_twi();
  port_init();
  sensor_init();
  DDRC &= 0xcf;  // Set SDA&SCL to Input
  PORTC |= 0x0c;
  while(1) 
  {
    Detect_sw();
    Detect_snap();
    Delay_ms(24); 
  }
}