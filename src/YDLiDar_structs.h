#ifndef YDLIDAR_STRUCTS_C
#define YDLIDAR_STRUCTS_C

typedef struct{
    uint16_t env = -1;
    uint16_t quality = -1;
    double angle = -1;
    uint16_t distance = -1;
    bool valid = false;
} iter_Measurement;

typedef struct{
    double env;
    uint16_t quality[SCANS_PER_CYCLE] = {(uint16_t)-1};
    double angle[SCANS_PER_CYCLE] = {(uint16_t)-1};
    uint16_t distance[SCANS_PER_CYCLE] {(uint16_t)-1};
    bool valid[SCANS_PER_CYCLE] = {false};
} iter_Scan;
#endif