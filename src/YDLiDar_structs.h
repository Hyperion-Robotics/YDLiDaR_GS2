#ifndef YDLIDAR_STRUCTS_C
#define YDLIDAR_STRUCTS_C

typedef struct{
    double env;
    uint16_t quality[SCANS_PER_CYCLE] = {(uint16_t)-1};
    double angle[SCANS_PER_CYCLE] = {(uint16_t)-1};
    uint16_t distance[SCANS_PER_CYCLE] = {(uint16_t)-1};
    bool valid[SCANS_PER_CYCLE] = {false};
} iter_Scan;

typedef struct{
    bool valid1[SCANS_PER_CYCLE] = {false};
    bool valid2[SCANS_PER_CYCLE] = {false};
    bool valid3[SCANS_PER_CYCLE] = {false};

    double env1;
    double env2;
    double env3;
    
    uint16_t quality1[SCANS_PER_CYCLE] = {(uint16_t)-1};
    uint16_t quality2[SCANS_PER_CYCLE] = {(uint16_t)-1};
    uint16_t quality3[SCANS_PER_CYCLE] = {(uint16_t)-1};
    
    uint16_t distance1[SCANS_PER_CYCLE] = {(uint16_t)-1};
    uint16_t distance2[SCANS_PER_CYCLE] = {(uint16_t)-1};
    uint16_t distance3[SCANS_PER_CYCLE] = {(uint16_t)-1};
    
    double angle1[SCANS_PER_CYCLE] = {(uint16_t)-1};
    double angle2[SCANS_PER_CYCLE] = {(uint16_t)-1};
    double angle3[SCANS_PER_CYCLE] = {(uint16_t)-1};
} iter_multi_Scans;


#endif