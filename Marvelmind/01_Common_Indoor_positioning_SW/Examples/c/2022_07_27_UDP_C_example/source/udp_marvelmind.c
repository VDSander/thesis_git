#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#ifdef WIN32
#include <windows.h>
#include <process.h>
#include <winsock2.h>
#else
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#endif // WIN32
#include "udp_marvelmind.h"

#ifdef WIN32
#pragma comment(lib,"ws2_32.lib") //Winsock Library
#endif

#define UDP_BUFFER_SIZE 255

#ifndef WIN32
#define SOCKET_ERROR (-1)
#endif

#define FRAME_ID_POSITION_CM 0x0001
#define FRAME_ID_STAT_POSITION_CM 0x0002
#define FRAME_ID_POSITION_MM 0x0011
#define FRAME_ID_STAT_POSITION_MM 0x0012
#define FRAME_ID_IMU_RAW 0x0003
#define FRAME_ID_RAW_DISTANCES 0x0004
#define FRAME_ID_IMU_FUSION 0x0005
#define FRAME_ID_TELEMETRY 0x0006
#define FRAME_ID_QUALITY 0x0007
#define FRAME_ID_NT_POSITION_MM 0x0081
#define FRAME_ID_NT_IMU_RAW 0x0083
#define FRAME_ID_NT_IMU_FUSION 0x0085

static uint8_t prepareUDPRequest(uint8_t beaconAddress, uint16_t dataCode, uint8_t bytesToRead, uint8_t *buf)
{
    buf[0]= beaconAddress;

    buf[1]= 0x47;

    buf[2]= dataCode&0xff;
    buf[3]= (dataCode>>8)&0xff;

    buf[4]= 0x04;

    buf[5]= 0x00;
    buf[6]= 0x00;

    buf[7]= bytesToRead;

    buf[8]= 0;
    buf[9]= 0;// CRC not required

    return 10;
}

//////////////////////////////////////////////////////////////////////////////
// Thread function started by MarvelmindUDP_start
//////////////////////////////////////////////////////////////////////////////

static void processHedgePosData(struct MarvelmindUDP * udp, uint8_t *bufferInput, bool highRes, uint8_t options) {
        uni_8x2_16 v16;
        uni_8x4_32 v32;
        TimestampOpt timestamp;

        bool realtime= ((options & UDP_OPTIONS_REALTIME) != 0);

        uint8_t addressReceived= bufferInput[0];

        uint8_t ofs= 5;

        if (realtime) {
            memcpy(&timestamp.timestamp64, &bufferInput[ofs], sizeof(timestamp.timestamp64));
            ofs+= 8;
        } else {
            v32.b[0]= bufferInput[ofs+0];
            v32.b[1]= bufferInput[ofs+1];
            v32.b[2]= bufferInput[ofs+2];
            v32.b[3]= bufferInput[ofs+3];
            timestamp.timestamp32= v32.dw;

            ofs+= 4;
        }

        int32_t xc,yc,zc;
        uint8_t flags;
        uint16_t angle;
        if (highRes)
        {
           v32.b[0]= bufferInput[ofs++];
           v32.b[1]= bufferInput[ofs++];
           v32.b[2]= bufferInput[ofs++];
           v32.b[3]= bufferInput[ofs++];
           xc= v32.dwi;

           v32.b[0]= bufferInput[ofs++];
           v32.b[1]= bufferInput[ofs++];
           v32.b[2]= bufferInput[ofs++];
           v32.b[3]= bufferInput[ofs++];
           yc= v32.dwi;

           v32.b[0]= bufferInput[ofs++];
           v32.b[1]= bufferInput[ofs++];
           v32.b[2]= bufferInput[ofs++];
           v32.b[3]= bufferInput[ofs++];
           zc= v32.dwi;

           flags= bufferInput[ofs++];

           ofs++;

           v16.b[0]= bufferInput[ofs++];
           v16.b[1]= bufferInput[ofs++];
           angle= v16.w;
        }
        else
        {
           v16.b[0]= bufferInput[ofs++];
           v16.b[1]= bufferInput[ofs++];
           xc= ((int32_t) v16.wi)*10;// millimeters

           v16.b[0]= bufferInput[ofs++];
           v16.b[1]= bufferInput[ofs++];
           yc= ((int32_t) v16.wi)*10;// millimeters

           v16.b[0]= bufferInput[ofs++];
           v16.b[1]= bufferInput[ofs++];
           zc= ((int32_t) v16.wi)*10;// millimeters

           flags= bufferInput[ofs++];

           ofs++;

           v16.b[0]= bufferInput[ofs++];
           v16.b[1]= bufferInput[ofs++];
           angle= v16.w;
        }

        // add to positionbuffer
#ifdef WIN32
        EnterCriticalSection(&udp->lock_);
#else
        pthread_mutex_lock (&udp->lock_);
#endif
        udp->positionBuffer[udp->lastValues_next].address= addressReceived;
        udp->positionBuffer[udp->lastValues_next].timestamp= timestamp;
        udp->positionBuffer[udp->lastValues_next].realTime= realtime;
        udp->positionBuffer[udp->lastValues_next].x= xc;
        udp->positionBuffer[udp->lastValues_next].y= yc;
        udp->positionBuffer[udp->lastValues_next].z= zc;
        udp->positionBuffer[udp->lastValues_next].flags= flags;
        udp->positionBuffer[udp->lastValues_next].angle= angle;
        udp->positionBuffer[udp->lastValues_next].updated= true;

        udp->lastValues_next++;
        if (udp->lastValues_next>=udp->maxBufferedPositions)
            udp->lastValues_next=0;
        if (udp->lastValuesCount_<udp->maxBufferedPositions)
            udp->lastValuesCount_++;
        udp->haveNewValues_=true;
#ifdef WIN32
        LeaveCriticalSection(&udp->lock_);
#else
        pthread_mutex_unlock (&udp->lock_);
#endif

        if (udp->receiveHedgePosCallback)
        {
            HedgePositionValue position=
            {
                addressReceived,
                timestamp,
                realtime,
                xc,
                yc,
                zc,
                flags,
                angle
            };
            udp->receiveHedgePosCallback (position);
        }
}

static void processStatPosData(struct MarvelmindUDP * udp, uint8_t *bufferInput, bool highRes) {
        uni_8x2_16 v16;
        uni_8x4_32 v32;

        //uint8_t addressReceived= bufferInput[0];
        uint8_t n= bufferInput[5];
        uint8_t i;
        uint8_t ofs= 5+1;

        int32_t xc,yc,zc;
        uint8_t addr;

         #ifdef WIN32
        EnterCriticalSection(&udp->lock_);
        #else
        pthread_mutex_lock (&udp->lock_);
        #endif

        for(i=0;i<n;i++) {
            addr= bufferInput[ofs];
            if (highRes)
                {
                    v32.b[0]= bufferInput[ofs+1];
                    v32.b[1]= bufferInput[ofs+2];
                    v32.b[2]= bufferInput[ofs+3];
                    v32.b[3]= bufferInput[ofs+4];
                    xc= v32.dwi;

                    v32.b[0]= bufferInput[ofs+5];
                    v32.b[1]= bufferInput[ofs+6];
                    v32.b[2]= bufferInput[ofs+7];
                    v32.b[3]= bufferInput[ofs+8];
                    yc= v32.dwi;

                    v32.b[0]= bufferInput[ofs+9];
                    v32.b[1]= bufferInput[ofs+10];
                    v32.b[2]= bufferInput[ofs+11];
                    v32.b[3]= bufferInput[ofs+12];
                    zc= v32.dwi;

                    ofs+= 14;
                }
            else
                {
                    v16.b[0]= bufferInput[ofs+1];
                    v16.b[1]= bufferInput[ofs+2];
                    xc= ((int32_t) v16.wi)*10;// millimeters

                    v16.b[0]= bufferInput[ofs+3];
                    v16.b[1]= bufferInput[ofs+4];
                    yc= ((int32_t) v16.wi)*10;// millimeters

                    v16.b[0]= bufferInput[ofs+5];
                    v16.b[1]= bufferInput[ofs+6];
                    zc= ((int32_t) v16.wi)*10;// millimeters

                    ofs+= 8;
                }

            udp->stationaryPos.items[i].address= addr;
            udp->stationaryPos.items[i].x= xc;
            udp->stationaryPos.items[i].y= yc;
            udp->stationaryPos.items[i].z= zc;
        }//for i

    udp->stationaryPos.n= n;
    udp->stationaryPos.updated= true;

    #ifdef WIN32
    LeaveCriticalSection(&udp->lock_);
    #else
    pthread_mutex_unlock (&udp->lock_);
    #endif
}

static void processImuRawData(struct MarvelmindUDP * udp, uint8_t *bufferInput, uint8_t options) {
    uni_8x2_16 v16;
    uni_8x4_32 v32;

    #ifdef WIN32
        EnterCriticalSection(&udp->lock_);
    #else
        pthread_mutex_lock (&udp->lock_);
    #endif

    bool realtime= ((options & UDP_OPTIONS_REALTIME) != 0);

     v16.b[0]= bufferInput[5];
     v16.b[1]= bufferInput[6];
     udp->imuRawData.accel_x= v16.wi;

     v16.b[0]= bufferInput[7];
     v16.b[1]= bufferInput[8];
     udp->imuRawData.accel_y= v16.wi;

     v16.b[0]= bufferInput[9];
     v16.b[1]= bufferInput[10];
     udp->imuRawData.accel_z= v16.wi;

     //

     v16.b[0]= bufferInput[11];
     v16.b[1]= bufferInput[12];
     udp->imuRawData.gyro_x= v16.wi;

     v16.b[0]= bufferInput[13];
     v16.b[1]= bufferInput[14];
     udp->imuRawData.gyro_y= v16.wi;

     v16.b[0]= bufferInput[15];
     v16.b[1]= bufferInput[16];
     udp->imuRawData.gyro_z= v16.wi;

     //

     v16.b[0]= bufferInput[17];
     v16.b[1]= bufferInput[18];
     udp->imuRawData.compass_x= v16.wi;

     v16.b[0]= bufferInput[19];
     v16.b[1]= bufferInput[20];
     udp->imuRawData.compass_y= v16.wi;

     v16.b[0]= bufferInput[21];
     v16.b[1]= bufferInput[22];
     udp->imuRawData.compass_z= v16.wi;

     udp->imuRawData.address= bufferInput[23];

     if (realtime) {
        memcpy(&udp->imuRawData.timestamp.timestamp64, &bufferInput[29], sizeof(udp->imuRawData.timestamp.timestamp64));
        udp->imuRawData.realTime= true;
     } else {
        v32.b[0]= bufferInput[29];
        v32.b[1]= bufferInput[30];
        v32.b[2]= bufferInput[31];
        v32.b[3]= bufferInput[32];
        udp->imuRawData.timestamp.timestamp32= v32.dw;
        udp->imuRawData.realTime= false;
     }

    udp->imuRawData.updated= true;

    #ifdef WIN32
        LeaveCriticalSection(&udp->lock_);
    #else
        pthread_mutex_unlock (&udp->lock_);
    #endif
}

static void processImuFusionData(struct MarvelmindUDP * udp, uint8_t *bufferInput, uint8_t options) {
    uni_8x2_16 v16;
    uni_8x4_32 v32;

    #ifdef WIN32
        EnterCriticalSection(&udp->lock_);
    #else
        pthread_mutex_lock (&udp->lock_);
    #endif

    bool realtime= ((options & UDP_OPTIONS_REALTIME) != 0);

    udp->imuFusionData.address= bufferInput[0];

    v32.b[0]= bufferInput[5];
    v32.b[1]= bufferInput[6];
    v32.b[2]= bufferInput[7];
    v32.b[3]= bufferInput[8];
    udp->imuFusionData.pos_x= v32.dwi;

    v32.b[0]= bufferInput[9];
    v32.b[1]= bufferInput[10];
    v32.b[2]= bufferInput[11];
    v32.b[3]= bufferInput[12];
    udp->imuFusionData.pos_y= v32.dwi;

    v32.b[0]= bufferInput[13];
    v32.b[1]= bufferInput[14];
    v32.b[2]= bufferInput[15];
    v32.b[3]= bufferInput[16];
    udp->imuFusionData.pos_z= v32.dwi;

    //

    v16.b[0]= bufferInput[17];
    v16.b[1]= bufferInput[18];
    udp->imuFusionData.quaternion_w= v16.wi;

    v16.b[0]= bufferInput[19];
    v16.b[1]= bufferInput[20];
    udp->imuFusionData.quaternion_x= v16.wi;

    v16.b[0]= bufferInput[21];
    v16.b[1]= bufferInput[22];
    udp->imuFusionData.quaternion_y= v16.wi;

    v16.b[0]= bufferInput[23];
    v16.b[1]= bufferInput[24];
    udp->imuFusionData.quaternion_z= v16.wi;

    //

    v16.b[0]= bufferInput[25];
    v16.b[1]= bufferInput[26];
    udp->imuFusionData.velocity_x= v16.wi;

    v16.b[0]= bufferInput[27];
    v16.b[1]= bufferInput[28];
    udp->imuFusionData.velocity_y= v16.wi;

    v16.b[0]= bufferInput[29];
    v16.b[1]= bufferInput[30];
    udp->imuFusionData.velocity_z= v16.wi;

    //

    v16.b[0]= bufferInput[31];
    v16.b[1]= bufferInput[32];
    udp->imuFusionData.accel_x= v16.wi;

    v16.b[0]= bufferInput[33];
    v16.b[1]= bufferInput[34];
    udp->imuFusionData.accel_y= v16.wi;

    v16.b[0]= bufferInput[35];
    v16.b[1]= bufferInput[36];
    udp->imuFusionData.accel_z= v16.wi;

    udp->imuRawData.address= bufferInput[37];

    //

    if (realtime) {
        memcpy(&udp->imuFusionData.timestamp.timestamp64, &bufferInput[39], sizeof(udp->imuFusionData.timestamp.timestamp64));
        udp->imuFusionData.realTime= true;
     } else {
        v32.b[0]= bufferInput[39];
        v32.b[1]= bufferInput[40];
        v32.b[2]= bufferInput[41];
        v32.b[3]= bufferInput[42];
        udp->imuFusionData.timestamp.timestamp32= v32.dw;
        udp->imuFusionData.realTime= false;
     }

    udp->imuFusionData.updated= true;

    #ifdef WIN32
        LeaveCriticalSection(&udp->lock_);
    #else
        pthread_mutex_unlock (&udp->lock_);
    #endif
}

void processRawDistancesData(struct MarvelmindUDP * udp, uint8_t *bufferInput) {
    uni_8x4_32 v32;

    #ifdef WIN32
        EnterCriticalSection(&udp->lock_);
    #else
        pthread_mutex_lock (&udp->lock_);
    #endif

    udp->rawDistances.hedgeAddress= bufferInput[5];

    uint8_t ofs= 6;

    uint8_t i;
    for(i=0;i<4;i++) {
        udp->rawDistances.items[i].address= bufferInput[ofs+0];

        v32.b[0]= bufferInput[ofs+1];
        v32.b[1]= bufferInput[ofs+2];
        v32.b[2]= bufferInput[ofs+3];
        v32.b[3]= bufferInput[ofs+4];
        udp->rawDistances.items[i].distance_mm= v32.dw;

        ofs+= 6;
    }

    udp->rawDistances.updated= true;

    #ifdef WIN32
        LeaveCriticalSection(&udp->lock_);
    #else
        pthread_mutex_unlock (&udp->lock_);
    #endif
}

static void processTelemetryData(struct MarvelmindUDP * udp, uint8_t *bufferInput) {
    uni_8x2_16 v16;

    #ifdef WIN32
        EnterCriticalSection(&udp->lock_);
    #else
        pthread_mutex_lock (&udp->lock_);
    #endif

    udp->telemetryPacket.address= bufferInput[0];

    v16.b[0]= bufferInput[5];
    v16.b[1]= bufferInput[6];
    udp->telemetryPacket.vbat_mv= v16.w;

    udp->telemetryPacket.rssi_dbm= (int8_t) bufferInput[7];

    udp->telemetryPacket.updated= true;

    #ifdef WIN32
        LeaveCriticalSection(&udp->lock_);
    #else
        pthread_mutex_unlock (&udp->lock_);
    #endif
}

static void processQualityData(struct MarvelmindUDP * udp, uint8_t *bufferInput) {
    #ifdef WIN32
        EnterCriticalSection(&udp->lock_);
    #else
        pthread_mutex_lock (&udp->lock_);
    #endif

    udp->qualityPacket.address= bufferInput[5];
    udp->qualityPacket.quality= bufferInput[6];

    udp->qualityPacket.updated= true;

    #ifdef WIN32
        LeaveCriticalSection(&udp->lock_);
    #else
        pthread_mutex_unlock (&udp->lock_);
    #endif
}


void
#ifndef WIN32
*
#endif // WIN32
Marvelmind_Thread_ (void* param)
{
    struct MarvelmindUDP * udp=(struct MarvelmindUDP*) param;

    int s;
    struct sockaddr_in si_other, si_me;
    int slen = sizeof(si_other) ;
    uint8_t bufferInput[UDP_BUFFER_SIZE];
    uint8_t bufferOutput[UDP_BUFFER_SIZE];
    uint8_t dataSize;
    bool highRes;

    udp->lastValues_next=0;

#ifdef WIN32
    WSADATA wsa;
    //Initialise winsock
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        printf("Winsock initialization failed. Error Code : %d\n",WSAGetLastError());
        return;
    }
#endif

    //create socket
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
    {
#ifdef WIN32
        printf("socket() failed with error code: %d \n" , WSAGetLastError());
        return;
#else
        printf("socket() failed\n");
        return NULL;
#endif // WIN32
    }
    printf("Socket opened\n");
    //setup address structure
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(udp->serverPort);
#ifdef WIN32
	si_other.sin_addr.S_un.S_addr = inet_addr(udp->serverAddress);
#else
	si_other.sin_addr.s_addr = inet_addr(udp->serverAddress);
#endif
    if (udp->beaconRequestAddress == 0)
    {// want to listen for stream
        si_me.sin_family = AF_INET;
        si_me.sin_port = htons(udp->serverPort);
#ifdef WIN32
        si_me.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
#else
        si_me.sin_addr.s_addr = htonl(INADDR_ANY);
#endif
        if (bind(s, (struct sockaddr *)&si_me, sizeof(si_me)) == SOCKET_ERROR)
        {
#ifdef WIN32
            printf("bind() failed: %d.\n", WSAGetLastError());
            closesocket(s);
            return;
#else
            return NULL;
#endif
        }
    }

    printf("UDP opened: address= %s   port= %d \n" , udp->serverAddress, udp->serverPort);

    int failCount= 0;
    while (udp->terminationRequired==false)
    {
#ifdef WIN32
        EnterCriticalSection(&udp->lock_);
#else
        pthread_mutex_lock (&udp->lock_);
#endif
        int sleepTimeMs= 0;;
        /*
        if (udp->requestRateHz == 0)
        {
            sleepTimeMs= 1000;
        }
        else
        {
            sleepTimeMs= (1000/udp->requestRateHz);
        }
        */

        uint8_t rqAddress= udp->beaconRequestAddress;
#ifdef WIN32
        LeaveCriticalSection(&udp->lock_);
#else
        pthread_mutex_unlock (&udp->lock_);
#endif
        //int sleepTimeFail= (failCount+1)*10;
        //if (sleepTimeFail>sleepTimeMs)
       //     sleepTimeMs= sleepTimeFail;

        if (rqAddress != 0) {
            if (udp->requestRateHz != 0)
            {

                if (sleepTimeMs != 0) {
                    #ifdef WIN32
                    Sleep(sleepTimeMs);
                    #else
                    usleep(sleepTimeMs*1000);
                    #endif // WIN32
                }
            }

            int sendSize= 0;

            if (udp->request_mode == REQUEST_MODE_POS) {
                sendSize= prepareUDPRequest(rqAddress, FRAME_ID_POSITION_MM, 0x16, bufferOutput);
            } else if (udp->request_mode == REQUEST_MODE_IMU_RAW) {
                sendSize= prepareUDPRequest(rqAddress, FRAME_ID_IMU_RAW, 0x20, bufferOutput);
            } else if (udp->request_mode == REQUEST_MODE_IMU_FUSION) {
                sendSize= prepareUDPRequest(rqAddress, FRAME_ID_IMU_FUSION, 0x2a, bufferOutput);
            }
            //send the message
            if (sendSize != 0)
              if (sendto(s, (const char *) bufferOutput, sendSize , 0 , (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
            {
#ifdef WIN32
                printf("sendto() failed with error code : %d\n" , WSAGetLastError());
#endif // WIN32
                if (failCount<100)
                    failCount++;
                continue;
            }
        }

        int recvSize= 64;
        //try to receive some data, this is a blocking call
        if (recvfrom(s, (char *) bufferInput, recvSize, 0, (struct sockaddr *) &si_other, &slen) == SOCKET_ERROR)
        {
            if (failCount<100)
                failCount++;
            continue;
        }
        failCount= 0;

        if (bufferInput[1] != 0x47)
            continue;
        uint16_t dataCode= bufferInput[2] + (((int) bufferInput[3])<<8);
        switch(dataCode)
        {
          case FRAME_ID_POSITION_MM:
            {
               dataSize= 0x16;
               highRes= true;
               break;
            }

          case FRAME_ID_POSITION_CM:
            {
               dataSize= 0x10;
               highRes= false;
               break;
            }

          case FRAME_ID_NT_POSITION_MM:
            {
              dataSize= 0;
              highRes= true;
              break;
            }

          case FRAME_ID_STAT_POSITION_MM:
            {
               dataSize= bufferInput[5]*14+1;
               highRes= true;
               break;
            }

          case FRAME_ID_STAT_POSITION_CM:
            {
               dataSize= bufferInput[5]*8+1;
               highRes= false;
               break;
            }

          case FRAME_ID_IMU_RAW:
            {
               dataSize= 0x20;
               break;
            }

          case FRAME_ID_NT_IMU_RAW:
            {
               dataSize= 0;
               break;
            }

          case FRAME_ID_RAW_DISTANCES:
            {
                dataSize= 0x20;
                break;
            }

          case FRAME_ID_IMU_FUSION:
            {
               dataSize= 0x2a;
               break;
            }

          case FRAME_ID_NT_IMU_FUSION:
            {
               dataSize= 0;
               break;
            }

          case FRAME_ID_TELEMETRY:
            {
                dataSize= 0x10;
                break;
            }

          case FRAME_ID_QUALITY:
            {
                dataSize= 0x10;
                break;
            }

          default:
            {
              continue;
            }
        }
        if ((bufferInput[4] != dataSize)&&(dataSize != 0))
            continue;

        switch(dataCode) {
            case FRAME_ID_POSITION_MM:
            case FRAME_ID_POSITION_CM:
              {
                 processHedgePosData(udp, &bufferInput[0], highRes,0);
                 break;
              }

            case FRAME_ID_NT_POSITION_MM:
              {
                 processHedgePosData(udp, &bufferInput[0], true, UDP_OPTIONS_REALTIME);
                 break;
              }

            case FRAME_ID_STAT_POSITION_MM:
            case FRAME_ID_STAT_POSITION_CM:
              {
                 processStatPosData(udp, &bufferInput[0], highRes);
                 break;
              }

            case FRAME_ID_IMU_RAW:
              {
                 processImuRawData(udp, &bufferInput[0], 0);
                 break;
              }

            case FRAME_ID_NT_IMU_RAW:
              {
                 processImuRawData(udp, &bufferInput[0], UDP_OPTIONS_REALTIME);
                 break;
              }

            case FRAME_ID_RAW_DISTANCES:
              {
                 processRawDistancesData(udp, &bufferInput[0]);
                 break;
              }

            case FRAME_ID_IMU_FUSION:
              {
                 processImuFusionData(udp, &bufferInput[0], 0);
                 break;
              }

            case FRAME_ID_NT_IMU_FUSION:
              {
                 processImuFusionData(udp, &bufferInput[0], UDP_OPTIONS_REALTIME);
                 break;
              }

            case FRAME_ID_TELEMETRY:
              {
                 processTelemetryData(udp, &bufferInput[0]);
                 break;
              }

            case FRAME_ID_QUALITY:
              {
                 processQualityData(udp, &bufferInput[0]);
                 break;
              }
        }

        // callback
        if (udp->anyInputPacketCallback) {
            udp->anyInputPacketCallback();
        }
    }
#ifndef WIN32
    return NULL;
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Create an initialize MarvelmindUDP structure
// returncode: pointer to structure on success or NULL on error
//////////////////////////////////////////////////////////////////////////////
struct MarvelmindUDP * createMarvelmindUDP ()
{
    struct MarvelmindUDP * udp=malloc (sizeof (struct MarvelmindUDP));
    if (udp)
    {
        udp->serverAddress=DEFAULT_UDP_SERVER_ADDRESS;
        udp->serverPort=DEFAULT_UDP_SERVER_PORT;
        udp->beaconRequestAddress= 0;
        udp->requestRateHz= 16;
        udp->maxBufferedPositions=3;
        udp->positionBuffer=NULL;
        udp->verbose=false;
        udp->receiveHedgePosCallback=NULL;
        udp->anyInputPacketCallback= NULL;
        udp->lastValuesCount_=0;
        udp->haveNewValues_=false;
        udp->terminationRequired= false;

        udp->request_mode= REQUEST_MODE_POS;

        udp->imuRawData.updated= false;
        udp->imuFusionData.updated= false;
#ifdef WIN32
        InitializeCriticalSection(&udp->lock_);
#else
        pthread_mutex_init (&udp->lock_, NULL);
#endif
    }
    else puts ("Not enough memory");
    return udp;
}

//////////////////////////////////////////////////////////////////////////////
// Initialize and start work thread
//////////////////////////////////////////////////////////////////////////////


int timezone_offset() {
    time_t zero = 0;
    struct tm* lt = localtime( &zero );
    if (lt == NULL) return 0;
    //return 0;
    //int unaligned = lt->tm_sec + ( lt->tm_min +  ( lt->tm_hour * 6 ) ) * 6;
    int unaligned = lt->tm_sec + ( lt->tm_min + ( lt->tm_hour * 60 ) ) * 60;
    return lt->tm_mon ? unaligned - 24*60*60 : unaligned;
}

void startMarvelmindUDP (struct MarvelmindUDP * udp)
{uint8_t i;
    udp->positionBuffer=
        malloc(sizeof (HedgePositionValue)*udp->maxBufferedPositions);
    if (udp->positionBuffer==NULL)
    {
        if (udp->verbose) puts ("Not enough memory");
        udp->terminationRequired=true;
        return;
    }

    for(i=0;i<udp->maxBufferedPositions;i++)
        udp->positionBuffer[i].updated= false;

    udp->stationaryPos.updated= false;
    udp->imuRawData.updated= false;
    udp->imuFusionData.updated= false;
    udp->rawDistances.updated= false;
    udp->telemetryPacket.updated= false;
    udp->qualityPacket.updated= false;

    udp->timeOffset= timezone_offset();

#ifdef WIN32
    _beginthread (Marvelmind_Thread_, 0, udp);
#else
    pthread_create (&udp->thread_, NULL, Marvelmind_Thread_, udp);
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Stop work thread
//////////////////////////////////////////////////////////////////////////////
void stopMarvelmindUDP (struct MarvelmindUDP * udp)
{
    udp->terminationRequired=true;
    if (udp->verbose) puts ("stopping");
#ifdef WIN32
    WaitForSingleObject (udp->thread_, INFINITE);
#else
    pthread_join (udp->thread_, NULL);
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Destroy structures to free memory (You must call stopMarvelmindUDP
// first)
//////////////////////////////////////////////////////////////////////////////
void destroyMarvelmindUDP (struct MarvelmindUDP * udp)
{
    if (udp->positionBuffer) free (udp->positionBuffer);
    free (udp);
}

//////////////////////////////////////////////////////////////////////////////
// Write average position coordinates
// udp:        MarvelmindUDP structure
// address:    address of hedge to get position
// position:   pointer to HedgePositionValue for write coordinates
// returncode: true if position is valid
//////////////////////////////////////////////////////////////////////////////
bool getHedgePositionFromMarvelmindUDP (struct MarvelmindUDP * udp,
                                   uint8_t address,
                                   HedgePositionValue * position)
{
    uint8_t i;
    int32_t avg_x=0, avg_y=0, avg_z=0;
    int64_t max_timestamp=0;
    TimestampOpt max_timestamp_opt;
    int64_t curT;
    uint8_t max_flags= 0;
    uint16_t max_angle= 0;
    bool position_valid;
    bool realTime= false;
#ifdef WIN32
    EnterCriticalSection(&udp->lock_);
#else
    pthread_mutex_lock (&udp->lock_);
#endif
    if (udp->lastValuesCount_)
    {
        uint8_t real_values_count=udp->maxBufferedPositions;
        uint8_t nFound= 0;
        if (udp->lastValuesCount_<real_values_count)
            real_values_count=udp->lastValuesCount_;
        for (i=0; i<real_values_count; i++)
        {
            if (udp->positionBuffer[i].address != address)
                continue;
            if (!udp->positionBuffer[i].updated)
                continue;
            nFound++;
            avg_x+=udp->positionBuffer[i].x;
            avg_y+=udp->positionBuffer[i].y;
            avg_z+=udp->positionBuffer[i].z;

            if (udp->positionBuffer[i].realTime) {
                curT= udp->positionBuffer[i].timestamp.timestamp64;
                realTime= true;
            } else {
                curT= udp->positionBuffer[i].timestamp.timestamp32;
                realTime= false;
            }

            if (curT>max_timestamp) {
                max_timestamp=curT;
                max_timestamp_opt= udp->positionBuffer[i].timestamp;
                max_flags= udp->positionBuffer[i].flags;
                max_angle= udp->positionBuffer[i].angle;
            }
        }
        if (nFound != 0)
        {
          avg_x/=nFound;
          avg_y/=nFound;
          avg_z/=nFound;
          position_valid=true;
        } else
        {
          position_valid=false;
        }
    }
    else position_valid=false;
#ifdef WIN32
    LeaveCriticalSection(&udp->lock_);
#else
    pthread_mutex_unlock (&udp->lock_);
#endif
    position->address= address;
    position->x=avg_x;
    position->y=avg_y;
    position->z=avg_z;
    position->timestamp=max_timestamp_opt;
    position->realTime= realTime;
    position->flags= max_flags;
    position->angle= max_angle;
    position->updated= position_valid;
    return position_valid;
}

//////////////////////////////////////////////////////////////////////////////


void printRealtimeStamp(struct MarvelmindUDP * udp, char *s, TimestampOpt timestamp, bool realTime) {
    if (!realTime) {
        sprintf(s, "%d", timestamp.timestamp32);
    } else {
        time_t time_sec= (timestamp.timestamp64 / 1000);
        if (time_sec>udp->timeOffset)
            time_sec= time_sec - udp->timeOffset;
        int time_ms= timestamp.timestamp64 % 1000;

        struct tm ts;
        struct tm *tsptr;
        tsptr= localtime(&time_sec);
        if (tsptr == NULL) return;
        ts= *tsptr;

        sprintf(s,"%04d_%02d_%02d__%02d%02d%02d_%03d",
                (int) ts.tm_year+1900, (int) ts.tm_mon+1, (int) ts.tm_mday, (int) ts.tm_hour, (int) ts.tm_min, (int) ts.tm_sec, (int) time_ms);
    }
}


//////////////////////////////////////////////////////////////////////////////
// Print average position coordinates
// onlyNew: print only new positions
//////////////////////////////////////////////////////////////////////////////
void printHedgePositionFromMarvelmindUDP (struct MarvelmindUDP * udp, bool onlyNew)
{uint8_t i,j;

    if (udp->haveNewValues_ || (!onlyNew))
    {
        HedgePositionValue position;
        uint8_t real_values_count=udp->maxBufferedPositions;
        uint8_t addresses[real_values_count];
        uint8_t addressesNum= 0;

        for(i=0;i<real_values_count;i++)
        {
           uint8_t address= udp->positionBuffer[i].address;
           bool alreadyProcessed= false;
           if (addressesNum != 0)
             for(j=0;j<addressesNum;j++)
               {
                  if (address == addresses[j])
                  {
                      alreadyProcessed= true;
                      break;
                  }
               }
            if (alreadyProcessed)
                continue;
            addresses[addressesNum++]= address;

            getHedgePositionFromMarvelmindUDP (udp, address, &position);

            char times[128];
            printRealtimeStamp(udp, times, position.timestamp, position.realTime);

            if (position.updated)
                printf ("Address: %d, X: %d, Y: %d, Z: %d  Flags: %d,  Angle: %.1f   at time T: %s\n", (int) address, (int) position.x,
                        (int) position.y, (int) position.z, (int) position.flags, (float) ((position.angle&0xfff)/10.0f),
                        times);
        }
        udp->haveNewValues_=false;
    }
}

//////////////////////////////////////////////////////////////////////////////
// Print received stationary beacons positions
//////////////////////////////////////////////////////////////////////////////
void printStationaryPositionsFromMarvelmindUDP (struct MarvelmindUDP * udp) {
    StationaryPositionValues spd= udp->stationaryPos;
    if (spd.updated) {
        uint8_t i;
        for(i=0;i<spd.n;i++) {
            printf ("Stationary beacon: Address: %d, X: %d, Y: %d, Z: %d  \n",
                (int) spd.items[i].address, (int) spd.items[i].x, (int) spd.items[i].y, (int) spd.items[i].z);
        }

        udp->stationaryPos.updated= false;
    }
}

//////////////////////////////////////////////////////////////////////////////
// Print received IMU data
//////////////////////////////////////////////////////////////////////////////
void printIMUFromMarvelmindUDP(struct MarvelmindUDP * udp) {
    char times[128];

    IMURawData ird= udp->imuRawData;
    if (ird.updated) {
        printRealtimeStamp(udp, times, ird.timestamp, ird.realTime);

        printf ("Raw IMU: Address: %d, AX: %d, AY: %d, AZ: %d     GX: %d, GY: %d, GZ: %d    CX: %d, CY: %d, CZ: %d   at time T: %s\n",
                (int) ird.address, (int) ird.accel_x, (int) ird.accel_y, (int) ird.accel_z,
                (int) ird.gyro_x, (int) ird.gyro_y, (int) ird.gyro_z,
                (int) ird.compass_x, (int) ird.compass_y, (int) ird.compass_z, times);

        udp->imuRawData.updated= false;
    }

    IMUFusionData ifd= udp->imuFusionData;
    if (ifd.updated) {
        printRealtimeStamp(udp, times, ifd.timestamp, ifd.realTime);

        printf ("IMU fusion: Address: %d, X: %d, Y: %d, Z: %d     QW: %d, QX: %d, QY: %d, QZ: %d    VX: %d, VY: %d, VZ: %d   AX: %d, AY: %d, AZ: %d   at time T: %s\n",
                (int) ifd.address, (int) ifd.pos_x, (int) ifd.pos_y, (int) ifd.pos_z,
                (int) ifd.quaternion_w, (int) ifd.quaternion_x, (int) ifd.quaternion_y, (int) ifd.quaternion_z,
                (int) ifd.velocity_x, (int) ifd.velocity_y, (int) ifd.velocity_z,
                (int) ifd.accel_x, (int) ifd.accel_y, (int) ifd.accel_z, times);

        udp->imuFusionData.updated= false;
    }
}

//////////////////////////////////////////////////////////////////////////////
// Print received raw distances
//////////////////////////////////////////////////////////////////////////////
void printRawDistancesFromMarvelmindUDP(struct MarvelmindUDP * udp) {
    RawDistances rdd= udp->rawDistances;
    if (rdd.updated) {
        uint8_t i;
        for(i=0;i<4;i++) {
            if (rdd.items[i].address == 0)
                continue;

            printf ("Raw distance: Hedge: %d, Beacon: %d,  Distance: %d mm  \n",
                (int) rdd.hedgeAddress, (int) rdd.items[i].address, (int) rdd.items[i].distance_mm);
        }

        udp->rawDistances.updated= false;
    }
}

//////////////////////////////////////////////////////////////////////////////
// Print received telemetry
//////////////////////////////////////////////////////////////////////////////
void printTelemetryFromMarvelmindUDP(struct MarvelmindUDP * udp) {
    TelemetryPacket tpd= udp->telemetryPacket;
    if (tpd.updated) {
        printf ("Telemetry: Address: %d, Vbat: %.3f V,   RSSI: %d dBm  \n",
                (int) tpd.address, ((float) tpd.vbat_mv)/1000.0f, (int) tpd.rssi_dbm);

        udp->telemetryPacket.updated= false;
    }
}

//////////////////////////////////////////////////////////////////////////////
// Print received quality
//////////////////////////////////////////////////////////////////////////////
void printQualityFromMarvelmindUDP(struct MarvelmindUDP * udp) {
    QualityPacket qpd= udp->qualityPacket;
    if (qpd.updated) {
        printf ("Quality: Address: %d, quality %d %%  \n",
                (int) qpd.address, (int) qpd.quality);

        udp->qualityPacket.updated= false;
    }
}


