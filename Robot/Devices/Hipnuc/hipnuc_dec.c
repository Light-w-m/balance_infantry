/**
 * @file hipnuc_dec.c
 * @author Light
 * @brief 超核电子hi05解析
 * @version 0.1
 * @date 2025-12-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "hipnuc_dec.h"

/* Common type conversion */
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t *)(p)))
#define I2(p) (*((int16_t *)(p)))

static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}
// static int32_t  I4(uint8_t *p) {int32_t u; memcpy(&u,p,4); return u;}
static float    R4(uint8_t *p) {float r; memcpy(&r,p,4); return r;}
static double D8(uint8_t *p) {double d; memcpy(&d, p, 8); return d;}

/**
 * @brief CRC计算
 * 
 * @param currectCrc 
 * @param src 
 * @param lengthInBytes 
 */
static void Crc16Update(
    uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j = 0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currectCrc = crc;
}

// 解析数据
int ParseData(hipnuc_raw_t *raw)
{
    int offset = 0; /* Payload start at buf[6] */
    uint8_t *p = &raw->buf[CH_HDR_SIZE];

    raw->hi91.tag = 0;
    raw->hi83.tag = 0;

    while (offset < raw->len)
    {
        /* code */
        switch (p[offset])
        {
        case HIPNUC_ID_HI91:
        {
            /* code */
            const int size = 76;
            if (offset + size > raw->len)
            {
                /* code */
                return -1;
            }
            raw->hi91.tag =           HIPNUC_ID_HI91;
            raw->hi91.main_status =   U2(p+offset+1);
            raw->hi91.temperature =   I1(p+offset+3);
            raw->hi91.pressure =      R4(p+offset+4);
            raw->hi91.timestamp =     U4(p+offset+8);

            raw->hi91.acc[0] =        R4(p+offset+12)*GRAVITY;
            raw->hi91.acc[1] =        R4(p+offset+16)*GRAVITY;
            raw->hi91.acc[2] =        R4(p+offset+20)*GRAVITY;

            raw->hi91.gyro[0] =       R4(p+offset+24);
            raw->hi91.gyro[1] =       R4(p+offset+28);
            raw->hi91.gyro[2] =       R4(p+offset+32);

            raw->hi91.mag[0] =        R4(p+offset+36);
            raw->hi91.mag[1] =        R4(p+offset+40);
            raw->hi91.mag[2] =        R4(p+offset+44);

            raw->hi91.roll =          R4(p+offset+48);
            raw->hi91.pitch =         R4(p+offset+52);
            raw->hi91.yaw =           R4(p+offset+56);

            raw->hi91.quaternion[0] = R4(p+offset+60);
            raw->hi91.quaternion[1] = R4(p+offset+64);
            raw->hi91.quaternion[2] = R4(p+offset+68);
            raw->hi91.quaternion[3] = R4(p+offset+72);

            offset += size;
        }
            break;
        case HIPNUC_ID_HI83:
        {
            raw->hi83.tag = HIPNUC_ID_HI83;
            raw->hi83.main_status = U2(p + offset + 1);
            raw->hi83.ins_status = p[offset + 3];
            raw->hi83.data_bitmap = U4(p + offset + 4);
            int idx = offset + 8;
            uint32_t bm = raw->hi83.data_bitmap;

            if (bm & HI83_BMAP_ACC_B) { raw->hi83.acc_b[0] = R4(p + idx + 0); raw->hi83.acc_b[1] = R4(p + idx + 4); raw->hi83.acc_b[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_GYR_B) { raw->hi83.gyr_b[0] = R4(p + idx + 0); raw->hi83.gyr_b[1] = R4(p + idx + 4); raw->hi83.gyr_b[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_MAG_B) { raw->hi83.mag_b[0] = R4(p + idx + 0); raw->hi83.mag_b[1] = R4(p + idx + 4); raw->hi83.mag_b[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_RPY) { raw->hi83.rpy[0] = R4(p + idx + 0); raw->hi83.rpy[1] = R4(p + idx + 4); raw->hi83.rpy[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_QUAT) { raw->hi83.quat[0] = R4(p + idx + 0); raw->hi83.quat[1] = R4(p + idx + 4); raw->hi83.quat[2] = R4(p + idx + 8); raw->hi83.quat[3] = R4(p + idx + 12); idx += 16; }
            if (bm & HI83_BMAP_SYSTEM_TIME) { raw->hi83.system_time = U4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_UTC) { raw->hi83.utc.year = p[idx+0]; raw->hi83.utc.month = p[idx+1]; raw->hi83.utc.day = p[idx+2]; raw->hi83.utc.hour = p[idx+3]; raw->hi83.utc.min = p[idx+4]; raw->hi83.utc.sec_ms = U2(p + idx + 5); raw->hi83.utc.rev = p[idx+7]; idx += 8; }
            if (bm & HI83_BMAP_AIR_PRESSURE) { raw->hi83.air_pressure = R4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_TEMPERATURE) { raw->hi83.temperature = R4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_INCLINATION) { raw->hi83.inclination[0] = R4(p + idx + 0); raw->hi83.inclination[1] = R4(p + idx + 4); raw->hi83.inclination[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_HSS) { raw->hi83.hss[0] = R4(p + idx + 0); raw->hi83.hss[1] = R4(p + idx + 4); raw->hi83.hss[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_HSS_FRQ) { raw->hi83.hss_frq[0] = R4(p + idx + 0); raw->hi83.hss_frq[1] = R4(p + idx + 4); raw->hi83.hss_frq[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_VEL_ENU) { raw->hi83.vel_enu[0] = R4(p + idx + 0); raw->hi83.vel_enu[1] = R4(p + idx + 4); raw->hi83.vel_enu[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_ACC_ENU) { raw->hi83.acc_enu[0] = R4(p + idx + 0); raw->hi83.acc_enu[1] = R4(p + idx + 4); raw->hi83.acc_enu[2] = R4(p + idx + 8); idx += 12; }
            if (bm & HI83_BMAP_INS_LON_LAT_MSL) { raw->hi83.ins_lon_lat_msl[0] = D8(p + idx + 0); raw->hi83.ins_lon_lat_msl[1] = D8(p + idx + 8); raw->hi83.ins_lon_lat_msl[2] = D8(p + idx + 16); idx += 24; }
            if (bm & HI83_BMAP_GNSS_QUALITY_NV) { raw->hi83.solq_pos = p[idx+0]; raw->hi83.nv_pos = p[idx+1]; raw->hi83.solq_heading = p[idx+2]; raw->hi83.nv_heading = p[idx+3]; idx += 4; }
            if (bm & HI83_BMAP_OD_SPEED) { raw->hi83.od_speed = R4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_UNDULATION) { raw->hi83.undulation = R4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_DIFF_AGE) { raw->hi83.diff_age = R4(p + idx); idx += 4; }
            if (bm & HI83_BMAP_NODE_ID) { raw->hi83.node.node_id = p[idx+0]; raw->hi83.node.reserved[0] = p[idx+1]; raw->hi83.node.reserved[1] = p[idx+2]; raw->hi83.node.reserved[2] = p[idx+3]; idx += 4; }
            if (bm & HI83_BMAP_GNSS_LON_LAT_MSL) { raw->hi83.gnss_lon_lat_msl[0] = D8(p + idx + 0); raw->hi83.gnss_lon_lat_msl[1] = D8(p + idx + 8); raw->hi83.gnss_lon_lat_msl[2] = D8(p + idx + 16); idx += 24; }
            if (bm & HI83_BMAP_GNSS_VEL) { raw->hi83.gnss_vel[0] = R4(p + idx + 0); raw->hi83.gnss_vel[1] = R4(p + idx + 4); raw->hi83.gnss_vel[2] = R4(p + idx + 8); idx += 12; }

            offset = idx;
        }
            break;
        default:
            offset++;
            break;
        }
    }
    return 1;
}
//crc
static int DecodeHipnucData(hipnuc_raw_t *raw)
{
    uint16_t crc = 0;
    Crc16Update(&crc, raw->buf, (CH_HDR_SIZE - 2));
    Crc16Update(&crc, raw->buf + CH_HDR_SIZE, raw->len);
    if (crc != U2(raw->buf + CH_HDR_SIZE - 2))
    {
        return -1;
    }
    return ParseData(raw);
}

//帧头
static int SyncHipnucData(uint8_t* buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == CHSYNC1 && buf[1] == CHSYNC2;
}

//输入--一次一个字节
int HipnucInput(uint8_t data, hipnuc_raw_t *raw)
{
    if (raw->nbyte == 0)
    {
        /* code */
        if (!SyncHipnucData(raw->buf, data))
        {
            /* code */
            return 0;
        }
        raw->nbyte = 2;
        return 0;        
    }
    raw->buf[raw->nbyte++] = data;
    if (raw->nbyte == CH_HDR_SIZE)
    {
        /* code */
        if ((raw->len = U2(raw->buf + 2)) > (HIPNUC_MAX_RAW_SIZE - CH_HDR_SIZE))
        {
            /* code */
            raw->nbyte = 0;
            return -1;
        }
        
    }
    if (raw->nbyte < CH_HDR_SIZE || raw->nbyte < (raw->len + CH_HDR_SIZE))
    {
        /* code */
        return 0;
    }
    raw->nbyte = 0;
    return DecodeHipnucData(raw);    
}

//将数据包转换为字符串，仅提取部分数据进行输出
int HipnucDumpPacket(hipnuc_raw_t *raw, char *buf, size_t buf_size)
{
    int written = 0;
    int ret = 0;
    if (raw->hi91.tag == HIPNUC_ID_HI91)
    {
        /* code */
        ret = snprintf(buf + written, buf_size - written,
            "{\n"
            "  \"type\": \"HI91\",\n"
            "  \"main_status\": [0x%X],\n"
            "  \"timestamp\": %ld,\n"
            "  \"acc\": [%.3f, %.3f, %.3f],\n"
            "  \"gyr\": [%.3f, %.3f, %.3f],\n"
            "  \"mag\": [%.3f, %.3f, %.3f],\n"
            "  \"pitch\": %.2f,\n"
            "  \"roll\": %.2f,\n"
            "  \"yaw\": %.2f,\n"
            "  \"quat\": [%.3f, %.3f, %.3f, %.3f],\n"
            "  \"pressure\": %.1f\n"
            "}\n",
            raw->hi91.main_status,
            raw->hi91.timestamp,
            raw->hi91.acc[0]*GRAVITY, raw->hi91.acc[1]*GRAVITY, raw->hi91.acc[2]*GRAVITY,
            raw->hi91.gyro[0], raw->hi91.gyro[1], raw->hi91.gyro[2],
            raw->hi91.mag[0], raw->hi91.mag[1], raw->hi91.mag[2],
            raw->hi91.pitch, raw->hi91.roll, raw->hi91.yaw,
            raw->hi91.quaternion[0], raw->hi91.quaternion[1], raw->hi91.quaternion[2], raw->hi91.quaternion[3],
            raw->hi91.pressure);
    }
    else if (raw->hi83.tag == HIPNUC_ID_HI83)
    {
        ret = snprintf(buf + written, buf_size - written,
            "{\n"
            "  \"type\": \"HI83\",\n"
            "  \"main_status\": %d,\n"
            "  \"ins_status\": %u,\n"
            "  \"data_bitmap\": %u\n",
            raw->hi83.main_status,
            (unsigned)raw->hi83.ins_status,
            (unsigned)raw->hi83.data_bitmap);
        if (ret > 0) written += ret;

        if (raw->hi83.data_bitmap & HI83_BMAP_ACC_B) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"acc\": [%.3f, %.3f, %.3f]\n", raw->hi83.acc_b[0], raw->hi83.acc_b[1], raw->hi83.acc_b[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_GYR_B) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"gyr\": [%.3f, %.3f, %.3f]\n", raw->hi83.gyr_b[0], raw->hi83.gyr_b[1], raw->hi83.gyr_b[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_MAG_B) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"mag\": [%.3f, %.3f, %.3f]\n", raw->hi83.mag_b[0], raw->hi83.mag_b[1], raw->hi83.mag_b[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_RPY) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"pitch\": %.2f\n  ,\"roll\": %.2f\n  ,\"yaw\": %.2f\n", raw->hi83.rpy[1], raw->hi83.rpy[0], raw->hi83.rpy[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_QUAT) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"quat\": [%.3f, %.3f, %.3f, %.3f]\n", raw->hi83.quat[0], raw->hi83.quat[1], raw->hi83.quat[2], raw->hi83.quat[3]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_SYSTEM_TIME) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"system_time\": %u\n", (unsigned)raw->hi83.system_time);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_UTC) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"utc\": \"20%02u-%02u-%02u %02u:%02u:%02u.%03u\"\n", (unsigned)raw->hi83.utc.year, (unsigned)raw->hi83.utc.month, (unsigned)raw->hi83.utc.day, (unsigned)raw->hi83.utc.hour, (unsigned)raw->hi83.utc.min, (unsigned)(raw->hi83.utc.sec_ms/1000), (unsigned)(raw->hi83.utc.sec_ms%1000));
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_AIR_PRESSURE) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"air_pressure\": %.1f\n", raw->hi83.air_pressure);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_TEMPERATURE) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"temperature\": %.2f\n", raw->hi83.temperature);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_INCLINATION) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"inclination\": [%.2f, %.2f, %.2f]\n", raw->hi83.inclination[0], raw->hi83.inclination[1], raw->hi83.inclination[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_HSS) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"hss\": [%.3f, %.3f, %.3f]\n", raw->hi83.hss[0], raw->hi83.hss[1], raw->hi83.hss[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_HSS_FRQ) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"hss_frq\": [%.3f, %.3f, %.3f]\n", raw->hi83.hss_frq[0], raw->hi83.hss_frq[1], raw->hi83.hss_frq[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_VEL_ENU) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"vel_enu\": [%.3f, %.3f, %.3f]\n", raw->hi83.vel_enu[0], raw->hi83.vel_enu[1], raw->hi83.vel_enu[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_ACC_ENU) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"acc_enu\": [%.3f, %.3f, %.3f]\n", raw->hi83.acc_enu[0], raw->hi83.acc_enu[1], raw->hi83.acc_enu[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_INS_LON_LAT_MSL) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"ins_lon_lat_msl\": [%.7f, %.7f, %.3f]\n", raw->hi83.ins_lon_lat_msl[0], raw->hi83.ins_lon_lat_msl[1], raw->hi83.ins_lon_lat_msl[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_GNSS_QUALITY_NV) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"solq_pos\": %u\n  ,\"nv_pos\": %u\n  ,\"solq_heading\": %u\n  ,\"nv_heading\": %u\n", (unsigned)raw->hi83.solq_pos, (unsigned)raw->hi83.nv_pos, (unsigned)raw->hi83.solq_heading, (unsigned)raw->hi83.nv_heading);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_OD_SPEED) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"od_speed\": %.3f\n", raw->hi83.od_speed);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_UNDULATION) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"undulation\": %.3f\n", raw->hi83.undulation);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_DIFF_AGE) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"diff_age\": %.3f\n", raw->hi83.diff_age);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_NODE_ID) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"node_id\": %u\n", (unsigned)raw->hi83.node.node_id);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_GNSS_LON_LAT_MSL) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"gnss_lon_lat_msl\": [%.7f, %.7f, %.3f]\n", raw->hi83.gnss_lon_lat_msl[0], raw->hi83.gnss_lon_lat_msl[1], raw->hi83.gnss_lon_lat_msl[2]);
            if (ret > 0) written += ret;
        }
        if (raw->hi83.data_bitmap & HI83_BMAP_GNSS_VEL) {
            ret = snprintf(buf + written, buf_size - written, "  ,\"gnss_vel\": [%.3f, %.3f, %.3f]\n", raw->hi83.gnss_vel[0], raw->hi83.gnss_vel[1], raw->hi83.gnss_vel[2]);
            if (ret > 0) written += ret;
        }

        ret = snprintf(buf + written, buf_size - written, "}\n");
        if (ret > 0) written += ret;
        ret = 0;
    }
    if (ret > 0) written += ret;
    return written;
}