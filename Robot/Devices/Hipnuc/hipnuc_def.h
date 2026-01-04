#ifndef HIPNUC_DEF_H
#define HIPNUC_DEF_H

#include <stdint.h>

#ifndef GRAVITY
#define GRAVITY (9.80665f)
#endif

#define HIPNUC_MIN_LENGTH       (82)   
#define HIPNUC_MAX_RAW_SIZE     (512)
#define HIPNUC_ID_HI91          (0x91)
#define HIPNUC_ID_HI83          (0x83)

/* HiPNUC protocol constants */
#define CHSYNC1                 (0x5A)              /* CHAOHE message sync code 1 */
#define CHSYNC2                 (0xA5)              /* CHAOHE message sync code 2 */
#define CH_HDR_SIZE             (0x06)              /* CHAOHE protocol header size */

/* main status 状态字说明 */
#define WB_CONV             (1U << 3)    // 零偏收敛状态--0：收敛良好 1：收敛精度差
#define MAG_DISTURB         (1U << 4)    // 磁场异常检测--0：环境良好或处于6轴模式 1：有磁干扰
#define MAG_AIDING          (1U << 10)   // 磁参与姿态解算状态--0：未参与 1：参与
#define UTC_TIME            (1U << 11)   // UTC时间同步状态--0：同步成功 1：未同步
#define SOUT_PULSE          (1U << 12)   // SOUT输出脉冲状态--0：没有输出 1：输出

#endif // HIPNUC_DEF_H