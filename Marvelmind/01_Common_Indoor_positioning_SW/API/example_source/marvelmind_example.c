#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef WIN32
#include <windows.h>
#endif // WIN32
#include "marvelmind_example.h"
#include "marvelmind_devices.h"
#include "marvelmind_utils.h"
#include "marvelmind_pos.h"

typedef enum {
    waitPort, waitDevice, connected
} ConState;
ConState conState= waitPort;

MMDeviceType deviceTypeUSB= unknown;

MarvelmindDeviceVersion usbDevVersion;

/////////////////////////////////////////////////////////////////////

static void switchToConState(ConState newConState);

// Reopen port
static void marvelmindReopenPort() {
    mmClosePort();
    switchToConState(waitPort);
}

// Reads Marvelmind API version
bool marvelmindCheckVersionCommand(char *token1) {
    if (strcmp(token1,"version") != 0)
       return false;

    uint32_t version= 0;
    if (mmAPIVersion(&version)) {
        printf("Marvelmind API version: %d\r\n", (int) version);
    } else {
        printf("Marvelmind API version read failed\r\n");
    }

    return true;
}

// Prints last error
void marvelmindPrintLastError(void) {
    uint32_t error;

    if (mmGetLastError(&error)) {
        printf("Last error: %d\r\n", (int) error);
    } else {
        printf("Get last error failed\r\n");
    }
}

// Check and execute wake command
bool marvelmindCheckWakeCommand(char *token1, char *token2) {
    if (strcmp(token1,"wake") != 0)
       return false;

    if (token2 == NULL)
        return true;

    uint8_t address= atoi(token2);
    if (mmWakeDevice(address)) {
        printf("Wake command was sent\r\n");
    } else {
        printf("Wake command failed\r\n");
        marvelmindPrintLastError();
    }

    return true;
}

// Check and execute sleep command
bool marvelmindCheckSleepCommand(char *token1, char *token2) {
    if (strcmp(token1,"sleep") != 0)
       return false;

    if (token2 == NULL)
        return true;

    uint8_t address= atoi(token2);
    if (mmSendToSleepDevice(address)) {
        printf("Sleep command was sent\r\n");
    } else {
        printf("Sleep command failed\r\n");
        marvelmindPrintLastError();
    }

    return true;
}

// Check and execute 'default' command - set default setings
bool marvelmindCheckDefaultCommand(char *token1, char *token2) {
    if (strcmp(token1,"default") != 0)
       return false;

    if (token2 == NULL)
        return true;

    uint8_t address= atoi(token2);
    if (mmSetDefaultSettings(address)) {
        printf("Default settings command was sent\r\n");
    } else {
        printf("Default setings command failed\r\n");
        marvelmindPrintLastError();
    }

    return true;
}

// Check and executer read beacon telemetry command
bool marvelmindCheckTelemetryCommand(char *token1, char *token2) {
    if (strcmp(token1,"tele") != 0)
       return false;

    if (token2 == NULL)
        return true;

    uint8_t address= atoi(token2);

    MarvelmindBeaconTelemetry tele;

    if (mmGetBeaconTelemetry(address, &tele)) {
        printf("Beacon %d telemetry:\r\n", (int) address);
        printf("  Working time: %d sec\r\n", (int) tele.worktimeSec);
        printf("  RSSI: %d dBm\r\n", (int) tele.rssi);
        printf("  Voltage: %.3f V\r\n", (float) tele.voltageMv/1000.0);
        printf("  Temperature: %d C\r\n", (int) tele.temperature);
    }

    return true;
}

// Read and show submap settings
bool marvelmindShowSubmapSettings(uint8_t submapId) {
    MarvelmindSubmapSettings sm;
    if (!mmGetSubmapSettings(submapId, &sm)) {
        return false;
    }

    uint8_t i;

    printf("Submap %d settings:\r\n", (int) submapId);

    if (sm.frozen) {
        printf("  Submap is FROZEN\r\n");
    } else {
        printf("  Submap is not frozen\r\n");
    }
    if (sm.locked) {
        printf("  Submap is locked\r\n");
    } else {
        printf("  Submap is not locked\r\n");
    }
    if (sm.beaconsHigher) {
        printf("  Stationary beacons higher than mobile\r\n");
    } else {
        printf("  Stationary beacons lower than mobile\r\n");
    }
    if (sm.mirrored) {
        printf("  Submap is mirrored\r\n");
    } else {
        printf("  Submap is not mirrored\r\n");
    }

    printf("  Starting beacon trilateration: %d\r\n", (int) sm.startingBeacon);
    printf("  Starting set: %d;  %d;%d;%d\r\n",
           (int) sm.startingSet_1, (int) sm.startingSet_2, (int) sm.startingSet_3,(int) sm.startingSet_4);
    printBoolEnabled("  3D navigation", sm.enabled3d);
    printBoolEnabled("  Only for Z coordinate", sm.onlyForZ);
    if (sm.limitationDistanceIsManual) {
        printf("  Limitation distances: manual\r\n");
        printf("  Maximum distance, m: %d\r\n", sm.maximumDistanceManual_m);
    } else {
        printf("  Limitation distances: auto\r\n");
    }

    printf("  Submap X shift, m: %.2f\r\n", (float) sm.submapShiftX_cm/100.0);
    printf("  Submap Y shift, m: %.2f\r\n", (float) sm.submapShiftY_cm/100.0);
    printf("  Submap Z shift, m: %.2f\r\n", (float) sm.submapShiftZ_cm/100.0);
    printf("  Submap rotation, degrees: %.2f\r\n", (float) sm.submapRotation_cdeg/100.0);

    printf("  Plane rotation quaternion: W=%d, X=%d, Y=%d, Z=%d\r\n",
           (int) sm.planeQw, (int) sm.planeQx, (int) sm.planeQy, (int) sm.planeQz);

    printf("  Service zone thickness, m: %.2f\r\n", (float) sm.serviceZoneThickness_cm/100.0);
    printf("  Hedges height in 2D mode, m: %.2f\r\n", (float) sm.hedgesHeightFor2D_cm/100.0);

    printf("  Beacons in submap: ");
    for(i=0;i<MM_SUBMAP_BEACONS_MAX_NUM;i++) {
        if (sm.beacons[i]!=0) {
            printf("%d ", (int) sm.beacons[i]);
        }
    }
    printf("\r\n");

    printf("  Nearby submaps: ");
    for(i=0;i<MM_NEARBY_SUBMAPS_MAX_NUM;i++) {
        if (sm.nearbySubmaps[i]!=255) {
            printf("%d ", (int) sm.nearbySubmaps[i]);
        }
    }
    printf("\r\n");

    printf("  Service zone: ");
    if (sm.serviceZonePointsNum == 0) {
        printf("none \r\n");
    } else {
        for(i=0;i<sm.serviceZonePointsNum;i++) {
           ServiceZonePoint p= sm.serviceZonePolygon[i];
           printf("X= %.2f, Y= %.2f     ", (float) p.x/100.0, (float) p.y/100.0);
        }
        printf("\r\n");
    }

    return true;
}

// Test function for writing submap settings
bool marvelmindTestSetSubmapSettings(uint8_t submapId) {
    MarvelmindSubmapSettings sm;

    uint8_t i;

    sm.frozen= true;
    sm.locked= true;
    sm.beaconsHigher= false;
    sm.mirrored= false;

    sm.startingBeacon= 9;

    sm.enabled3d= true;
    sm.onlyForZ= true;
    sm.limitationDistanceIsManual= true;
    sm.maximumDistanceManual_m= 19;

    sm.submapShiftX_cm= 987;
    sm.submapShiftY_cm= -654;
    sm.submapShiftZ_cm= 321;

    sm.submapRotation_cdeg= 10423;

    sm.planeQw= 10000;
    sm.planeQx= 0;
    sm.planeQy= 0;
    sm.planeQz= 0;

    sm.serviceZoneThickness_cm= -500;

    sm.hedgesHeightFor2D_cm= 350;

    for(i=0;i<MM_SUBMAP_BEACONS_MAX_NUM;i++) {
       sm.beacons[i]= 0;
    }
    sm.beacons[0]= 9;
    sm.beacons[1]= 10;

    for(i=0;i<MM_NEARBY_SUBMAPS_MAX_NUM;i++) {
        sm.nearbySubmaps[i]= 255;
    }
    sm.nearbySubmaps[0]= 2;

    sm.serviceZonePointsNum= 4;
    sm.serviceZonePolygon[0].x= 100;
    sm.serviceZonePolygon[0].y= 150;
    sm.serviceZonePolygon[1].x= -220;
    sm.serviceZonePolygon[1].y= 130;
    sm.serviceZonePolygon[2].x= -250;
    sm.serviceZonePolygon[2].y= -80;
    sm.serviceZonePolygon[3].x= 154;
    sm.serviceZonePolygon[3].y= -120;

    if (!mmSetSubmapSettings(submapId, &sm)) {
        printf("Submap %d settings sending error\r\n", (int) submapId);
        marvelmindPrintLastError();
        return false;
    }

    printf("Submap %d settings sending success\r\n", (int) submapId);

    return true;
}

// Check and executer submap command (add submap, delete submap etc)
bool marvelmindCheckSubmapCommand(char *token1, char *token2, char *token3) {
    if (strcmp(token1,"submap") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (token3 == NULL)
        return true;

    uint8_t submapId= atoi(token3);

    if (strcmp(token2,"add") == 0) {
        if (mmAddSubmap(submapId)) {
            printf("Submap %d added\r\n", (int) submapId);
        } else {
            printf("Submap %d add failed\r\n", (int) submapId);
            marvelmindPrintLastError();
        }
    }
    else if (strcmp(token2,"delete") == 0) {
        if (mmDeleteSubmap(submapId)) {
            printf("Submap %d deleted\r\n", (int) submapId);
        } else {
            printf("Submap %d delete failed\r\n", (int) submapId);
            marvelmindPrintLastError();
        }
    }
    else if (strcmp(token2,"freeze") == 0) {
        if (mmFreezeSubmap(submapId)) {
            printf("Submap %d freeze success\r\n", (int) submapId);
        } else {
            printf("Submap %d freeze failed\r\n", (int) submapId);
            marvelmindPrintLastError();
        }
    }
    else if (strcmp(token2,"unfreeze") == 0) {
        if (mmUnfreezeSubmap(submapId)) {
            printf("Submap %d unfreeze success\r\n", (int) submapId);
        } else {
            printf("Submap %d unfreeze failed\r\n", (int) submapId);
            marvelmindPrintLastError();
        }
    }
    else if (strcmp(token2,"get") == 0) {
        marvelmindShowSubmapSettings((int) submapId);
    }
    else if (strcmp(token2,"testset") == 0) {
        marvelmindTestSetSubmapSettings((int) submapId);
    }

    return true;
}

// Check and execute map command
bool marvelmindCheckMapCommand(char *token1, char *token2) {
    if (strcmp(token1,"map") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (strcmp(token2,"erase") == 0) {
        if (mmEraseMap()) {
            printf("Erase map success\r\n");
        } else {
            printf("Erase map failed\r\n");
            marvelmindPrintLastError();
        }
        return true;
    } else if (strcmp(token2,"freeze") == 0) {
        if (mmFreezeMap()) {
            printf("Freeze map success\r\n");
        } else {
            printf("Freeze map failed\r\n");
            marvelmindPrintLastError();
        }
        return true;
    } else if (strcmp(token2,"unfreeze") == 0) {
        if (mmUnfreezeMap()) {
            printf("Unfreeze map success\r\n");
        } else {
            printf("Unfreeze map failed\r\n");
            marvelmindPrintLastError();
        }
        return true;
    }

    return true;
}

// Command to get/set update rate setting
bool marvelmindCheckRateCommand(char *token1, char *token2, char *token3) {
    if (strcmp(token1,"rate") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (strcmp(token2,"get") == 0) {
        float updRate;
        if (mmGetUpdateRateSetting(&updRate)) {
            printf("Update rate setting: %.2f Hz\r\n", updRate);
        } else {
            printf("Update rate setting read failed\r\n");
            marvelmindPrintLastError();
        }
        return true;
    }

    if (strcmp(token2,"set") == 0) {
        if (token3 == NULL)
            return true;

        float updRate= atof(token3);
        if (mmSetUpdateRateSetting(&updRate)) {
            printf("Update rate setting write success\r\n");
        } else {
            printf("Update rate setting write failed\r\n");
            marvelmindPrintLastError();
        }
        return true;
    }

    return true;
}

static char *marvelmindDSPFilterString(uint8_t filterIndex) {
    switch(filterIndex) {
        case MM_US_FILTER_19KHZ: return "19 kHz";
        case MM_US_FILTER_25KHZ: return "25 kHz";
        case MM_US_FILTER_31KHZ: return "31 kHz";
        case MM_US_FILTER_37KHZ: return "37 kHz";
        case MM_US_FILTER_45KHZ: return "45 kHz";
        default: return "unknown";
    }
}

// Command to get/set ultrasound settings
bool marvelmindCheckUltrasoundCommand(char *token1, char *token2, char *token3) {
    if (strcmp(token1,"usound") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (token3 == NULL)
        return true;

    uint8_t address= atoi(token3);

    if (strcmp(token2,"get") == 0) {
        MarvelmindUltrasoundSettings us;
        if (mmGetUltrasoundSettings(address, &us)) {
            printf("Ultrasound settings for beacon %d:\r\n", address);
            printf("  Tx frequency, Hz: %d\r\n", (int) us.txFrequency_hz);
            printf("  Tx number of periods: %d\r\n", (int) us.txPeriodsNumber);
            if (us.rxAmplifierAGC) {
               printf("  Amplification: AGC\r\n");
            } else {
                printf("  Amplification: manual\r\n");
            }
            if (!us.rxAmplifierAGC) {
                printf("  Amplification: %d\r\n", (int) us.rxAmplificationManual);
            }
            printf("  Sensors normal: %d %d %d %d %d\r\n",
                   boolAsInt(us.sensorsNormal[0]),
                   boolAsInt(us.sensorsNormal[1]),
                   boolAsInt(us.sensorsNormal[2]),
                   boolAsInt(us.sensorsNormal[3]),
                   boolAsInt(us.sensorsNormal[4])
                   );
            printf("  Sensors frozen: %d %d %d %d %d\r\n",
                   boolAsInt(us.sensorsFrozen[0]),
                   boolAsInt(us.sensorsFrozen[1]),
                   boolAsInt(us.sensorsFrozen[2]),
                   boolAsInt(us.sensorsFrozen[3]),
                   boolAsInt(us.sensorsFrozen[4])
                   );

            printf("  Rx DSP filter: %s\r\n", marvelmindDSPFilterString(us.rxDSPFilterIndex));
        } else {
            printf("Ultrasound settings read failed\r\n");
            marvelmindPrintLastError();
        }
        return true;
    }

    if (strcmp(token2,"testset") == 0) {
        MarvelmindUltrasoundSettings us;
        printf("Test writing ultrasound settings to beacon %d\r\n", (int) address);

        us.txFrequency_hz= 31234;
        us.txPeriodsNumber= 34;
        us.rxAmplifierAGC= false;
        us.rxAmplificationManual= 2345;

        us.sensorsNormal[MM_SENSOR_RX1]= true;
        us.sensorsNormal[MM_SENSOR_RX2]= false;
        us.sensorsNormal[MM_SENSOR_RX3]= false;
        us.sensorsNormal[MM_SENSOR_RX4]= true;
        us.sensorsNormal[MM_SENSOR_RX5]= false;

        us.sensorsFrozen[MM_SENSOR_RX1]= true;
        us.sensorsFrozen[MM_SENSOR_RX2]= false;
        us.sensorsFrozen[MM_SENSOR_RX3]= false;
        us.sensorsFrozen[MM_SENSOR_RX4]= true;
        us.sensorsFrozen[MM_SENSOR_RX5]= true;

        us.rxDSPFilterIndex= MM_US_FILTER_45KHZ;

        if (mmSetUltrasoundSettings(address, &us)) {
            printf("Ultrasound settings write success\r\n");
        } else {
            printf("Ultrasound settings write failed\r\n");
            marvelmindPrintLastError();
        }

        return true;
    }

    return true;
}

bool marvelmindCheckAxesCommand(char *token1, char *token2, char *token3, char *token4) {
    if (strcmp(token1,"axes") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (token3 == NULL)
        return true;

    if (token4 == NULL)
        return true;

    uint8_t address_0= atoi(token2);
    uint8_t address_x= atoi(token3);
    uint8_t address_y= atoi(token4);

     if (mmBeaconsToAxes(address_0, address_x, address_y)) {
            printf("Beacons to axes success\r\n");
        } else {
            printf("Beacons to axes failed\r\n");
            marvelmindPrintLastError();
        }

    return true;
}

static uint8_t dump_buffer[65536];
bool marvelmindCheckReadDumpCommand(char *token1, char *token2, char *token3) {
    if (strcmp(token1,"read_dump") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (token3 == NULL)
        return true;


    uint32_t offset= atoi(token2);
    uint32_t size= atoi(token3);
    if (size == 0)
        return true;
    if (size>65536UL)
        size= 65536UL;

    uint32_t i;

    //uint8_t *dump_buffer= malloc(size);

    if (mmReadFlashDump(offset, size, &dump_buffer[0])) {
        printf("Read flash dump success\r\n");
        for(i=0;i<size;i++) {
            printf(" %02x", dump_buffer[i]);
        }
        printf("\r\n");
    } else {
        printf("Read flash dump failed\r\n");
        marvelmindPrintLastError();
    }

    //free(dump_buffer);

    return true;
}

bool marvelmindCheckWriteDumpCommand(char *token1, char *token2, char *token3) {
    if (strcmp(token1,"write_dump_test") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (token3 == NULL)
        return true;

    uint32_t offset= atoi(token2);
    uint32_t size= atoi(token3);
    if (size == 0)
        return true;
    if (size>65536UL)
        size= 65536UL;

    uint32_t i;

    //uint8_t *dump_buffer= malloc(size);
    for(i=0;i<size;i++) {
        dump_buffer[i]= i+1;
    }

    if (mmWriteFlashDump(offset, size, &dump_buffer[0])) {
        printf("Write flash dump success\r\n");
    } else {
        printf("Write flash dump failed\r\n");
        marvelmindPrintLastError();
    }

    //free(dump_buffer);

    return true;
}

bool marvelmindCheckResetCommand(char *token1, char *token2) {
    if (strcmp(token1,"reset") != 0)
       return false;

    if (token2 == NULL)
        return true;

    uint32_t address= atoi(token2);

    if (mmResetDevice(address)) {
        printf("Reset device success\r\n");
    } else {
        printf("Reset device failed\r\n");
        marvelmindPrintLastError();
    }

    return true;
}

bool marvelmindCheckTemperatureCommand(char *token1, char *token2, char *token3) {
    if (strcmp(token1,"temperature") != 0)
       return false;

    if (token2 == NULL)
       return true;

    if (strcmp(token2,"get") == 0) {
        int8_t temperature;
        if (mmGetAirTemperature(&temperature)) {
            printf("Temperature %d celsius \r\n", (int) temperature);
        } else {
            printf("Temperature read failed\r\n");
            marvelmindPrintLastError();
        }
    } else if (strcmp(token2,"set") == 0) {
        if (token3 == NULL)
            return true;

        int8_t temperature= atoi(token3);
        if (mmSetAirTemperature(temperature)) {
            printf("Temperature write success \r\n");
        } else {
            printf("Temperature write failed\r\n");
            marvelmindPrintLastError();
        }
    }

    return true;
}


// Change connection state
static void switchToConState(ConState newConState) {
    switch(newConState) {
        case waitPort: {
            printf("Waiting for port...\r\n");
            break;
        }
        case waitDevice: {
            printf("Trying connect to device...\r\n");
            break;
        }
        case connected: {
            printf("Device is connected via USB.\r\n");
            printMMDeviceVersionAndId(&usbDevVersion);

            deviceTypeUSB= getMMDeviceType(usbDevVersion.fwVerDeviceType);
            printMMDeviceType(&deviceTypeUSB);
            break;
        }
    }
    conState= newConState;
}

bool marvelmindCheckSetLocCommand(char *token1, char *token2, char *token3, char *token4, char *token5) {
    if (strcmp(token1,"setloc") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (token3 == NULL)
        return true;

    if (token4 == NULL)
        return true;

    if (token5 == NULL)
        return true;

    uint8_t address= atoi(token2);
    float pos_x_m= atof(token3);
    float pos_y_m= atof(token4);
    float pos_z_m= atof(token5);

    if (mmSetBeaconLocation(address, pos_x_m*1000.0f, pos_y_m*1000.0f, pos_z_m*1000.0f)) {
        printf("Location setup success\r\n");
    } else {
        printf("Location setup failed\r\n");
        marvelmindPrintLastError();
    }

    return true;
}

bool marvelmindCheckSetDistanceCommand(char *token1, char *token2, char *token3, char *token4) {
    if (strcmp(token1,"setdist") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (token3 == NULL)
        return true;

    if (token4 == NULL)
        return true;

    uint8_t address_1= atoi(token2);
    uint8_t address_2= atoi(token3);
    float distance_m= atof(token4);

    if (mmSetBeaconsDistance(address_1, address_2, distance_m*1000.0f)) {
        printf("Distance setup success\r\n");
    } else {
        printf("Distance setup failed\r\n");
        marvelmindPrintLastError();
    }

    return true;
}

bool marvelmindCheckHedgeHeightCommand(char *token1, char *token2, char *token3, char *token4) {
    if (token2 == NULL)
       return true;

    if (strcmp(token2,"get") == 0) {
        int32_t height;

        if (token3 == NULL)
           return true;
        uint8_t address= atoi(token3);

        if (mmGetHedgeHeight(address, &height)) {
            printf("Height is %.3f meters \r\n", ((float) height)/1000.0f);
        } else {
            printf("Height read failed\r\n");
            marvelmindPrintLastError();
        }
    } else if (strcmp(token2,"set") == 0) {
        if (token3 == NULL)
            return true;

        if (token4 == NULL)
           return true;

        uint8_t address= atoi(token3);
        int32_t height = atof(token4)*1000.0f;

        if (mmSetHedgeHeight(address, height)) {
            printf("Height write success \r\n");
        } else {
            printf("Height write failed\r\n");
            marvelmindPrintLastError();
        }
    }

    return true;
}

bool marvelmindCheckBeaconHeightCommand(char *token1, char *token2, char *token3, char *token4, char *token5) {
    if (token2 == NULL)
       return true;

    if (strcmp(token2,"get") == 0) {
        int32_t height;

        if (token3 == NULL)
           return true;
        uint8_t address= atoi(token3);

        uint8_t submapId= atoi(token4);

        if (mmGetBeaconHeight(address, submapId, &height)) {
            printf("Height in submap %d is %.3f meters \r\n", (int) submapId, ((float) height)/1000.0f);
        } else {
            printf("Height read failed\r\n");
            marvelmindPrintLastError();
        }
    } else if (strcmp(token2,"set") == 0) {
        if (token3 == NULL)
            return true;

        if (token4 == NULL)
           return true;

        uint8_t address= atoi(token3);
        uint8_t submapId= atoi(token4);
        int32_t height = atof(token5)*1000.0f;

        if (mmSetBeaconHeight(address, submapId, height)) {
            printf("Height write success \r\n");
        } else {
            printf("Height write failed\r\n");
            marvelmindPrintLastError();
        }
    }

    return true;
}

bool marvelmindCheckHeightCommand(char *token1, char *token2, char *token3, char *token4, char *token5) {
    if (strcmp(token1,"height_h") == 0) {
        return marvelmindCheckHedgeHeightCommand(token1, token2, token3, token4);
    }
    if (strcmp(token1,"height_b") == 0) {
        return marvelmindCheckBeaconHeightCommand(token1, token2, token3, token4, token5);
    }

    return false;
}

bool marvelmindCheckRealtimePlayerCommand(char *token1, char *token2, char *token3) {
    if (strcmp(token1,"rtp") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (token3 == NULL)
        return true;

    uint8_t address= atoi(token3);

    if (strcmp(token2,"get") == 0) {
        MarvelmindRealtimePlayerSettings rtp;
        if (mmGetRealtimePlayerSettings(address, &rtp)) {
            printf("Real-time player settings for beacon %d:\r\n", address);
            printf("  Enabled: %d\r\n", (int) rtp.rtpEnabled);
            printf("  Forward: %d\r\n", (int) rtp.rtpForward);
            printf("  Backward: %d\r\n", (int) rtp.rtpBackward);
        } else {
            printf("Real-time player settings for beacon %d read failed:\r\n", address);
            marvelmindPrintLastError();
        }
    } else if (strcmp(token2,"testset") == 0) {
        MarvelmindRealtimePlayerSettings rtp;
        printf("Test writing real-time settings to beacon %d\r\n", (int) address);

        rtp.rtpEnabled= true;
        rtp.rtpForward= 2;
        rtp.rtpBackward= 7;
        rtp.reserved0= 0;
        rtp.reserved1= 0;

        if (mmSetRealtimePlayerSettings(address, &rtp)) {
            printf("Real-time player settings write success\r\n");
        } else {
            printf("Real-time player settings write failed\r\n");
            marvelmindPrintLastError();
        }

        return true;
    }

    return false;
}

bool marvelmindCheckGeoreferencingCommand(char *token1, char *token2, char *token3, char *token4) {
    if (strcmp(token1,"georef") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (strcmp(token2,"get") == 0) {
        MarvelmindGeoreferencingSettings gr;
        if (mmGetGeoreferencingSettings(&gr)) {
            printf("Georeferencing:\r\n");
            printf("  Latitude: %.7f\r\n", ((double) gr.latitude_x100ndeg)/10000000.0);
            printf("  Longitude: %.7f\r\n", ((double) gr.longitude_x100ndeg)/10000000.0);
        } else {
            printf("Georeferencing settings read failed:\r\n");
            marvelmindPrintLastError();
        }

        return true;
    } else if (strcmp(token2,"set") == 0) {
        if (token3 == NULL)
            return true;

        if (token4 == NULL)
            return true;

        double latitude= atof(token3);
        double longitude= atof(token4);

        MarvelmindGeoreferencingSettings gr;
        gr.latitude_x100ndeg= latitude*10000000.0;
        gr.longitude_x100ndeg= longitude*10000000.0;

        if (mmSetGeoreferencingSettings(&gr)) {
            printf("Georeferencing settings write success\r\n");
        } else {
            printf("Georeferencing settings write failed\r\n");
            marvelmindPrintLastError();
        }
    }

    return false;
}

bool marvelmindCheckUpdateLocationModeCommand(char *token1, char *token2, char *token3) {
    if (strcmp(token1,"update_mode") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (strcmp(token2,"get") == 0) {
        MarvelmindUpdatePositionsMode upm;
        if (mmGetUpdatePositionsMode(&upm)) {
            printf("Update locations mode: %d\r\n", (int) upm.mode);
        } else {
            printf("Update locations mode read failed:\r\n");
            marvelmindPrintLastError();
        }

        return true;
    } else if (strcmp(token2,"set") == 0) {
        if (token3 == NULL)
            return true;

        uint8_t mode= atoi(token3);

        MarvelmindUpdatePositionsMode upm;
        uint8_t i;
        upm.mode= mode;
        for(i=0;i<7;i++)
            upm.reserved[i]= 0;

        if (mmSetUpdatePositionsMode(&upm)) {
            printf("Update locations mode write success\r\n");
        } else {
            printf("Update locations mode write failed\r\n");
            marvelmindPrintLastError();
        }
    }

    return true;
}

bool marvelmindCheckSendUpdateLocationCommand(char *token1) {
    if (strcmp(token1,"update") != 0)
       return false;

    MarvelmindUpdatePositionsCommand upc;
    uint8_t i;
    for(i=0;i<8;i++)
        upc.reserved[i]= 0;

    if (mmSendUpdatePositionsCommand(&upc)) {
        printf("Update locations command success\r\n");
    } else {
        printf("Update locations command failed\r\n");
        marvelmindPrintLastError();
    }

    return true;
}

bool marvelmindCheckSetMotorsCommand(char *token1, char *token2, char *token3, char *token4, char *token5) {
    if (strcmp(token1,"motors") != 0)
       return false;

    if (token2 == NULL)
        return true;

    if (token3 == NULL)
        return true;

    uint8_t address= atoi(token2);
    MarvelmindMotorsSettings ms;
    ms.mode= atoi(token3);

    if (ms.mode == 0) {
        ms.moveType= 0;
        ms.levelPercents= 0;
    } else {
        if (token4 == NULL)
          return true;

        if (token5 == NULL)
          return true;

        ms.moveType= atoi(token4);
        ms.levelPercents= atoi(token5);
    }

    if (mmSetMotorsControl(address, &ms)) {
        printf("Motors setup success\r\n");
    } else {
        printf("Motors setup failed\r\n");
        marvelmindPrintLastError();
    }

    return true;
}

// Working cycle if modem is connected via USB
void marvelmindModemCycle() {
    static uint8_t failCounter= 0;

    marvelmindDevicesReadIfNeeded();

    switch(marvelmindLocationsReadIfNeeded()) {
        case readSuccess: {
            failCounter= 0;
            break;
        }
        case readFail: {
            failCounter++;
            if (failCounter>10) {
                marvelmindReopenPort();
                break;
            }
            break;
        }
        case notRead: {
            break;
        }
    }
}

// Working cycle if beacon is connected via USB
void marvelmindBeaconCycle() {
    //TODO
}

// Working cycle if robot is connected via USB
void marvelmindRobotCycle() {
    //TODO
}

// Marvelmind communication state machine
void marvelmindCycle() {
        switch(conState) {
        case waitPort: {
            //if (mmOpenPortByName("com10")) {
            if (mmOpenPort()) {
                switchToConState(waitDevice);
                break;
            }
            sleep_ms(1);
            break;
        }
        case waitDevice: {
            if (mmGetVersionAndId(MM_USB_DEVICE_ADDRESS, &usbDevVersion)) {
                switchToConState(connected);
            }
            sleep_ms(1);
            break;
        }
        case connected: {
            switch(deviceTypeUSB) {
                case modem: {
                    marvelmindModemCycle();
                    break;
                }
                case beacon:
                case hedgehog: {
                    marvelmindBeaconCycle();
                    break;
                }

                case robot: {
                    marvelmindRobotCycle();
                    break;
                }

                case unknown: {
                    break;
                }
            }
            break;
        }
    }
}

void marvelmindStart() {
    marvelmindAPILoad();// Load Marvelmind API library

    initMarvelmindDevicesList();
    initMarvelmindPos();
    switchToConState(waitPort);// Start waiting port connection
}

void marvelmindFinish() {
    mmClosePort();// Close port (if was opened)

    marvelmindAPIFree();// Free Marvelmind API library memory
}
