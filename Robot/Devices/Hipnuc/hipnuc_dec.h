#ifndef HIPNUC_DEC_H
#define HIPNUC_DEC_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "hipnuc_def.h"

typedef struct 
{
    /* data */
    uint8_t tag;               //0x91数据包
    uint16_t main_status;      //状态字
    int8_t temperature;        //温度
    float pressure;            //气压
    uint32_t timestamp;        //时间戳--ms

    float acc[3];              //加速度--x,y,z--g
    float gyro[3];             //角速度--x,y,z--deg/s
    float mag[3];              //磁场--x,y,z

    float roll;
    float pitch;
    float yaw;
    float quaternion[4];       //四元数--q0,q1,q2,q3
    
} imu_data_t;

/* HI83 bitmap masks */
#define HI83_BMAP_ACC_B              (1u << 0)
#define HI83_BMAP_GYR_B              (1u << 1)
#define HI83_BMAP_MAG_B              (1u << 2)
#define HI83_BMAP_RPY                (1u << 3)
#define HI83_BMAP_QUAT               (1u << 4)
#define HI83_BMAP_SYSTEM_TIME        (1u << 5)
#define HI83_BMAP_UTC                (1u << 6)
#define HI83_BMAP_AIR_PRESSURE       (1u << 7)
#define HI83_BMAP_TEMPERATURE        (1u << 8)
#define HI83_BMAP_INCLINATION        (1u << 9)
#define HI83_BMAP_HSS                (1u << 10)
#define HI83_BMAP_HSS_FRQ            (1u << 11)
#define HI83_BMAP_VEL_ENU            (1u << 12)
#define HI83_BMAP_ACC_ENU            (1u << 13)
#define HI83_BMAP_INS_LON_LAT_MSL    (1u << 14)
#define HI83_BMAP_GNSS_QUALITY_NV    (1u << 15)
#define HI83_BMAP_OD_SPEED           (1u << 16)
#define HI83_BMAP_UNDULATION         (1u << 17)
#define HI83_BMAP_DIFF_AGE           (1u << 18)
#define HI83_BMAP_NODE_ID            (1u << 19)
#define HI83_BMAP_GNSS_LON_LAT_MSL   (1u << 30)
#define HI83_BMAP_GNSS_VEL           (1u << 31)

typedef struct __attribute__((__packed__))
{
    uint8_t  tag;
    uint16_t main_status;
    uint8_t  ins_status;
    uint32_t data_bitmap;

    float    acc_b[3];
    float    gyr_b[3];
    float    mag_b[3];
    float    rpy[3];
    float    quat[4];
    uint32_t system_time;
    struct __attribute__((__packed__)) {
        uint8_t  year;
        uint8_t  month;
        uint8_t  day;
        uint8_t  hour;
        uint8_t  min;
        uint16_t sec_ms;
        uint8_t  rev;
    } utc;
    float    air_pressure;
    float    temperature;
    float    inclination[3];
    float    hss[3];
    float    hss_frq[3];
    float    vel_enu[3];
    float    acc_enu[3];
    double   ins_lon_lat_msl[3];
    uint8_t  solq_pos;
    uint8_t  nv_pos;
    uint8_t  solq_heading;
    uint8_t  nv_heading;
    float    od_speed;
    float    undulation;
    float    diff_age;
    struct __attribute__((__packed__)) {
        uint8_t node_id;
        uint8_t reserved[3];
    } node;
    double   gnss_lon_lat_msl[3];
    float    gnss_vel[3];
} hi83_t;

typedef struct
{
    uint16_t len;
    int nbyte;
    uint8_t buf[HIPNUC_MAX_RAW_SIZE];       /* Message raw buffer */
    imu_data_t hi91;                        /* Decoded 0x91 packet data */
    hi83_t hi83;
} hipnuc_raw_t;

// int ParseData(hipnuc_raw_t *raw);
int HipnucInput(uint8_t data, hipnuc_raw_t *raw);
int HipnucDumpPacket(hipnuc_raw_t *raw, char *buf, size_t buf_size);

#endif // HIPNUC_DEC_H