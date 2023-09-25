#define A0pin  A0  // Analog output - yellow
#define SIpin  22  // Srart Integation -orange
#define CLKpin 23  //
#define NPIXELS 128 // No,\. of Pisels in array

byte Pixel[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adapation;
int i;

#define FASTADC 1
#define cbi(sfr,bit)(_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr,bit)(_SFR_BYTE(sfr) |= _BV(bit))

void setup() 
{
  
  for(i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i]          = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i]      = 1023;//0
    MIN_LineSensor_Data[i]      = 0;//1023
  }
  pinMode(SIpin,OUTPUT);
  pinMode(CLKpin,OUTPUT);
  pinMode(A0pin,INPUT);

  digitalWrite(SIpin,LOW);
  digitalWrite(CLKpin,LOW);

#if FASTADC
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);
#endif

    flag_line_adapation = 0;
    
    Serial.begin(115200);
    Serial.begin("TSL 1401");
    
}
void read_line_camera(void)
{
  delay (1);

  digitalWrite(CLKpin,    LOW);
  digitalWrite(SIpin,     HIGH);
  digitalWrite(CLKpin,    HIGH);
  digitalWrite(SIpin,     LOW);
  delayMicroseconds(1);

  for( i = 0; i < NPIXELS; i++)
  {
    Pixel[i] = analogRead (A0pin)/4; // 8-bit is enough
    digitalWrite(CLKpin,  LOW);
    delayMicroseconds(1);
    digitalWrite(CLKpin,  HIGH);
  }
  digitalWrite(CLKpin,    LOW);

}

void loop()
{
  read_line_camera();

  for( i = 0; i< NPIXELS; i++)
  {
    Serial.print((byte)Pixel[i] + 1);
    
  }

}
