/*
  The circuit:
 * HEDGEHOG serial data to digital pin 0 (RXD)
 * LCD RS pin to digital pin 8
 * LCD Enable pin to digital pin 9
 * LCD D4 pin to digital pin 4
 * LCD D5 pin to digital pin 5
 * LCD D6 pin to digital pin 6
 * LCD D7 pin to digital pin 7
 * LCD BL pin to digital pin 10
 *Vcc pin to  +5
 */

#include <stdlib.h>
#include <LiquidCrystal.h>

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//  MARVELMIND HEDGEHOG RELATED PART

long hedgehog_x, hedgehog_y;// coordinates of hedgehog (X,Y), mm
long hedgehog_z;// height of hedgehog, mm
int hedgehog_pos_updated;// flag of new data from hedgehog received

bool high_resolution_mode;

// IMU sensors raw data
int16_t imu_acc_x,imu_acc_y,imu_acc_z;
int16_t imu_gyro_x,imu_gyro_y,imu_gyro_z;
int16_t imu_compass_x,imu_compass_y,imu_compass_z;
int imu_updated;

int64_t imu_raw_timestamp;

///

#define HEDGEHOG_BUF_SIZE 80 
#define HEDGEHOG_CM_DATA_SIZE 0x10
#define HEDGEHOG_MM_DATA_SIZE 0x16
#define HEDGEHOG_RAW_IMU_DATA_SIZE 0x20
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;

#define POSITION_DATAGRAM_ID 0x0001
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define RAW_IMU_DATAGRAM_ID 0x0003
#define NT_POSITION_DATAGRAM_HIGHRES_ID 0x0081
#define NT_RAW_IMU_DATAGRAM_ID 0x0083
unsigned int hedgehog_data_id;

typedef union {byte b[2]; unsigned int w;int wi;} uni_8x2_16;
typedef union {byte b[4];float f;unsigned long v32;long vi32;} uni_8x4_32;
typedef union {byte b[8];int64_t vi64;} uni_8x8_64;

//    Marvelmind hedgehog support initialize
void setup_hedgehog() 
{
  Serial.begin(500000); // hedgehog transmits data on 500 kbps  

  hedgehog_serial_buf_ofs= 0;
  hedgehog_pos_updated= 0;

  imu_updated= 0;
}

// Marvelmind hedgehog service loop
void loop_hedgehog()
{int incoming_byte;
 int total_received_in_loop;
 int packet_received;
 bool good_byte;
 byte packet_size;
 uni_8x2_16 un16;
 uni_8x4_32 un32;
 uni_8x8_64 un64;

  total_received_in_loop= 0;
  packet_received= 0;
  
  while(Serial.available() > 0)
    {
      if (hedgehog_serial_buf_ofs>=HEDGEHOG_BUF_SIZE) 
      {
        hedgehog_serial_buf_ofs= 0;// restart bufer fill
        break;// buffer overflow
      }
      total_received_in_loop++;
      if (total_received_in_loop>100) break;// too much data without required header
      
      incoming_byte= Serial.read();
      good_byte= false;
      switch(hedgehog_serial_buf_ofs)
      {
        case 0:
        {
          good_byte= (incoming_byte == 0xff);
          break;
        }
        case 1:
        {
          good_byte= (incoming_byte == 0x47);
          break;
        }
        case 2:
        {
          good_byte= true;
          break;
        }
        case 3:
        {
          hedgehog_data_id= (((unsigned int) incoming_byte)<<8) + hedgehog_serial_buf[2];
          good_byte=   (hedgehog_data_id == POSITION_DATAGRAM_ID) ||
                       (hedgehog_data_id == POSITION_DATAGRAM_HIGHRES_ID) ||
                       (hedgehog_data_id == RAW_IMU_DATAGRAM_ID) ||
                       (hedgehog_data_id == NT_POSITION_DATAGRAM_HIGHRES_ID) ||
                       (hedgehog_data_id == NT_RAW_IMU_DATAGRAM_ID);
          break;
        }
        case 4:
        {
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_CM_DATA_SIZE);
              break;
            }
            case POSITION_DATAGRAM_HIGHRES_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_MM_DATA_SIZE);
              break;
            }
            case RAW_IMU_DATAGRAM_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_RAW_IMU_DATA_SIZE);
              break;
            }

            case NT_POSITION_DATAGRAM_HIGHRES_ID:
            case NT_RAW_IMU_DATAGRAM_ID:
            {
              good_byte= true;
              break;
            }
          }
          break;
        }
        default:
        {
          good_byte= true;
          break;
        }
      }
      
      if (!good_byte)
        {
          hedgehog_serial_buf_ofs= 0;// restart bufer fill         
          continue;
        }     
      hedgehog_serial_buf[hedgehog_serial_buf_ofs++]= incoming_byte; 
      if (hedgehog_serial_buf_ofs>5)
        {
          packet_size=  7 + hedgehog_serial_buf[4];
          if (hedgehog_serial_buf_ofs == packet_size)
            {// received packet with required header
              packet_received= 1;
              hedgehog_serial_buf_ofs= 0;// restart bufer fill
              break; 
            }
        }
    }

  if (packet_received)  
    {
      hedgehog_set_crc16(&hedgehog_serial_buf[0], packet_size);// calculate CRC checksum of packet
      if ((hedgehog_serial_buf[packet_size] == 0)&&(hedgehog_serial_buf[packet_size+1] == 0))
        {// checksum success
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
            {
              // coordinates of hedgehog (X,Y), cm ==> mm
              un16.b[0]= hedgehog_serial_buf[9];
              un16.b[1]= hedgehog_serial_buf[10];
              hedgehog_x= 10*long(un16.wi);

              un16.b[0]= hedgehog_serial_buf[11];
              un16.b[1]= hedgehog_serial_buf[12];
              hedgehog_y= 10*long(un16.wi);
              
              // height of hedgehog, cm==>mm (FW V3.97+)
              un16.b[0]= hedgehog_serial_buf[13];
              un16.b[1]= hedgehog_serial_buf[14];
              hedgehog_z= 10*long(un16.wi);
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= false;
              break;
            }

            case POSITION_DATAGRAM_HIGHRES_ID:
            case NT_POSITION_DATAGRAM_HIGHRES_ID:
            {
              byte ofs= 9;
              if (hedgehog_data_id == NT_POSITION_DATAGRAM_HIGHRES_ID) {
                ofs+= 4;
              }
              
              // coordinates of hedgehog (X,Y), mm
              un32.b[0]= hedgehog_serial_buf[ofs+0];
              un32.b[1]= hedgehog_serial_buf[ofs+1];
              un32.b[2]= hedgehog_serial_buf[ofs+2];
              un32.b[3]= hedgehog_serial_buf[ofs+3];
              hedgehog_x= un32.vi32;

              un32.b[0]= hedgehog_serial_buf[ofs+4];
              un32.b[1]= hedgehog_serial_buf[ofs+5];
              un32.b[2]= hedgehog_serial_buf[ofs+6];
              un32.b[3]= hedgehog_serial_buf[ofs+7];
              hedgehog_y= un32.vi32;
              
              // height of hedgehog, mm 
              un32.b[0]= hedgehog_serial_buf[ofs+8];
              un32.b[1]= hedgehog_serial_buf[ofs+9];
              un32.b[2]= hedgehog_serial_buf[ofs+10];
              un32.b[3]= hedgehog_serial_buf[ofs+11];
              hedgehog_z= un32.vi32;
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= true;
              break;
            }

            case RAW_IMU_DATAGRAM_ID:
            case NT_RAW_IMU_DATAGRAM_ID:
            {
              un16.b[0]= hedgehog_serial_buf[5];
              un16.b[1]= hedgehog_serial_buf[6];
              imu_acc_x= un16.wi;

              un16.b[0]= hedgehog_serial_buf[7];
              un16.b[1]= hedgehog_serial_buf[8];
              imu_acc_y= un16.wi;

              un16.b[0]= hedgehog_serial_buf[9];
              un16.b[1]= hedgehog_serial_buf[10];
              imu_acc_z= un16.wi;

              //

              un16.b[0]= hedgehog_serial_buf[11];
              un16.b[1]= hedgehog_serial_buf[12];
              imu_gyro_x= un16.wi;

              un16.b[0]= hedgehog_serial_buf[13];
              un16.b[1]= hedgehog_serial_buf[14];
              imu_gyro_y= un16.wi;

              un16.b[0]= hedgehog_serial_buf[15];
              un16.b[1]= hedgehog_serial_buf[16];
              imu_gyro_z= un16.wi;

              //

              un16.b[0]= hedgehog_serial_buf[17];
              un16.b[1]= hedgehog_serial_buf[18];
              imu_compass_x= un16.wi;

              un16.b[0]= hedgehog_serial_buf[19];
              un16.b[1]= hedgehog_serial_buf[20];
              imu_compass_y= un16.wi;

              un16.b[0]= hedgehog_serial_buf[21];
              un16.b[1]= hedgehog_serial_buf[22];
              imu_compass_z= un16.wi; 

              if (hedgehog_data_id == RAW_IMU_DATAGRAM_ID) {
                un32.b[0]= hedgehog_serial_buf[29];
                un32.b[1]= hedgehog_serial_buf[30];
                un32.b[2]= hedgehog_serial_buf[31];
                un32.b[3]= hedgehog_serial_buf[32];
                imu_raw_timestamp= un32.vi32;
              } else {
                un64.b[0]= hedgehog_serial_buf[29];
                un64.b[1]= hedgehog_serial_buf[30];
                un64.b[2]= hedgehog_serial_buf[31];
                un64.b[3]= hedgehog_serial_buf[32];
                un64.b[4]= hedgehog_serial_buf[33];
                un64.b[5]= hedgehog_serial_buf[34];
                un64.b[6]= hedgehog_serial_buf[35];
                un64.b[7]= hedgehog_serial_buf[36];
                imu_raw_timestamp= un64.vi64;
              }

              imu_updated= 1;

              break;
            }
          }
        } 
    }
}

// Calculate CRC-16 of hedgehog packet
void hedgehog_set_crc16(byte *buf, byte size)
{uni_8x2_16 sum;
 byte shift_cnt;
 byte byte_cnt;

  sum.w=0xffffU;

  for(byte_cnt=size; byte_cnt>0; byte_cnt--)
   {
   sum.w=(unsigned int) ((sum.w/256U)*256U + ((sum.w%256U)^(buf[size-byte_cnt])));

     for(shift_cnt=0; shift_cnt<8; shift_cnt++)
       {
         if((sum.w&0x1)==1) sum.w=(unsigned int)((sum.w>>1)^0xa001U);
                       else sum.w>>=1;
       }
   }

  buf[size]=sum.b[0];
  buf[size+1]=sum.b[1];// little endian
}// hedgehog_set_crc16

//  END OF MARVELMIND HEDGEHOG RELATED PART
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


#define CM 1      //Centimeter
#define INC 0     //Inch
#define TP 2      //Trig_pin
#define EP 3      //Echo_pin

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);

void setup()
{
  lcd.clear(); 
  lcd.begin(16, 2);

  lcd.setCursor(0,0); 
  lcd.print("Waiting");
  lcd.setCursor(0,1); 
  lcd.print("raw IMU data...");

  setup_hedgehog();//    Marvelmind hedgehog support initialize
}

void loop()
{  byte lcd_coord_precision;
   char lcd_buf[12];

   loop_hedgehog();// Marvelmind hedgehog service loop

   if (imu_updated)
     {// new IMU data from hedgehog available
       imu_updated= 0;// clear new data flag 

       // First line - accelerometer data
       lcd.setCursor(0,0); 
       itoa(imu_acc_x, &lcd_buf[0], 10);
       lcd.print(lcd_buf);
       lcd.print("    ");  
       
       lcd.setCursor(5,0);
       itoa(imu_acc_y, &lcd_buf[0], 10);
       lcd.print(lcd_buf);
       lcd.print("    ");  
       
       lcd.setCursor(10,0);
       itoa(imu_acc_z, &lcd_buf[0], 10);
       lcd.print(lcd_buf);
       lcd.print("    ");  

       
       // Second line - gyro data
       lcd.setCursor(0,1); 
       itoa(imu_gyro_x, &lcd_buf[0], 10);
       lcd.print(lcd_buf);
       lcd.print("    ");  
        
       lcd.setCursor(5,1);
       itoa(imu_gyro_y, &lcd_buf[0], 10);
       lcd.print(lcd_buf);
       lcd.print("    ");  
       
       lcd.setCursor(10,1);
       itoa(imu_gyro_z, &lcd_buf[0], 10);
       lcd.print(lcd_buf);
       lcd.print("    ");
       
       
       /*
       // Second line - compass data
       lcd.setCursor(0,1); 
       itoa(imu_compass_x, &lcd_buf[0], 10);
       lcd.print(lcd_buf);
       lcd.print("    ");  
        
       lcd.setCursor(5,1);
       itoa(imu_compass_y, &lcd_buf[0], 10);
       lcd.print(lcd_buf);
       lcd.print("    ");  
       
       lcd.setCursor(10,1);
       itoa(imu_compass_z, &lcd_buf[0], 10);
       lcd.print(lcd_buf);
       lcd.print("    ");
       */

       //delay(200);
     }
}
