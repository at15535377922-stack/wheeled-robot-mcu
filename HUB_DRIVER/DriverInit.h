#ifndef __DRIVERINIT_H
#define __DRIVERINIT_H
#include "system.h"

typedef struct
{
	float L_motorTemperature;
	float R_motorTemperature;
	float L_motorCurrent;
	float R_motorCurrent;
	u16 SoftwareVersion;
}HubDrive_parameter;
extern HubDrive_parameter Back_Drive;
extern HubDrive_parameter Front_Drive;

extern u32 FeedBackID;

#define canX 2 //Doc:1銆�2 | 閫夋嫨CAN1鎴栬�匔AN2鍙戦��

//椹卞姩鍣�1鐨勫彂閫佸拰鎺ユ敹鍦板潃
#define COBID_Send_1 0x601
#define COBID_Recv_1 0x581

//椹卞姩鍣�2鐨勫彂閫佸拰鎺ユ敹鍦板潃
#define COBID_Send_2 0x602
#define COBID_Recv_2 0x582

//椹卞姩鍣ㄧ殑璇诲啓灞炴��
#define write_hub 0x01 //鍐欐暟鎹� rw=0x01
#define read_hub  0x10 //璇绘暟鎹� rw=0x10

//椹卞姩鍣ㄥ湴鍧�鍐呴儴鐨勭储寮曞��
#define Index_None 0x00 //鏃犵储寮曞��
#define Index_L    0x01 //宸︾數鏈虹储寮�
#define Index_R    0x02 //鍙崇數鏈虹储寮�

//澶栭儴鎬ュ仠瑙ｈ酱銆侀攣杞磋�剧疆
#define IO_LOCK   0 //閿佽酱
#define IO_UNLOCK 1 //瑙ｈ酱

//鎺у埗妯″紡
#define POS_REL_CONTROL 1 //鐩稿�逛綅缃�鎺у埗
#define POS_ABS_CONTROL 2 //缁濆�逛綅缃�鎺у埗
#define VEL_CONTROL 3 //閫熷害鎺у埗
#define TOR_CONTROL 4 //杞�鐭╂帶鍒�

/* 椹卞姩鍣ㄥ唴閮ㄨ�剧疆鍦板潃 */
#define ENCODER_ACCURACY_ADDR      0x200E //缂栫爜鍣ㄧ嚎鏁拌�剧疆鍦板潃
#define HALL_OFFSET_ADDR           0x2011 //鐢垫満涓嶩ALL鍋忕Щ瑙掑害鍦板潃
#define MOTOR_POLES_ADDR           0x200C //鐢垫満鏋佸�规暟璁剧疆鍦板潃
#define MOTOR_RATEDCUR_ADDR        0x2014 //鐢垫満棰濆畾鐢垫祦璁剧疆鍦板潃
#define MOTOR_MAXCUR_ADDR          0x2015 //鐢垫満鏈�澶х數娴佽�剧疆鍦板潃
#define MOTOR_MAXRPM_ADDR          0x2008 //鐢垫満鏈�澶ц浆閫熻�剧疆鍦板潃
#define MOTOR_OVERLOAD_FACTOR_ADDR 0x2012 //鐢垫満杩囪浇淇濇姢绯绘暟璁剧疆鍦板潃
#define MOTOR_OVERLOAD_TIME_ADDR   0x2016 //鐢垫満杩囪浇淇濇姢鏃堕棿璁剧疆鍦板潃
#define ENCODER_ERRORALARM_ADDR    0x2017 //缂栫爜鍣ㄨ秴宸�鎶ヨ�﹂槇鍊艰�剧疆鍦板潃
#define MAX_TEMPERATURE_ADDR       0x2013 //鐢垫満娓╁害淇濇姢闃堝�艰�剧疆鍦板潃
#define SPEED_KP_ADDR              0x201D //閫熷害鐜疜P璁剧疆鍦板潃
#define SPEED_KI_ADDR              0x201E //閫熷害鐜疜i璁剧疆鍦板潃
#define SPEED_KF_ADDR              0x201F //閫熷害鍓嶉�圞F璁剧疆鍦板潃
#define SPEED_SMOOTH_ADDR          0x2018 //閫熷害骞虫粦绯绘暟璁剧疆鍦板潃
#define SMOOTH_KF_ADDR             0x201B //鍓嶉�堝钩婊戠郴鏁拌�剧疆鍦板潃
#define POS_KP_ADDR                0x2020 //浣嶇疆鐜痥p璁剧疆鍦板潃
#define POS_KF_ADDR                0x2021 //浣嶇疆鍓嶉�圞f璁剧疆鍦板潃
#define TOR_SMOOTH_ADDR            0x201C //杞�鐭╁钩婊戠郴鏁拌�剧疆鍦板潃
#define TOR_KP_ADDR                0x2019 //鐢垫祦鐜痥p璁剧疆鍦板潃
#define TOR_KI_ADDR                0x201A //鐢垫祦鐜痥i璁剧疆鍦板潃
#define MOTOR_SYNCON_ADDR          0x200F //鐢垫満鍚屾��/寮傛�ユ帶鍒惰�剧疆鍦板潃
#define MOTOR_ACCTIME_ADDR         0x6083 //鐢垫満鍔犻�熸椂闂磋�剧疆
#define MOTOR_DECTIME_ADDR         0x6084 //鐢垫満鍑忛�熸椂闂磋�剧疆
#define MOTOR_STOPTIME_ADDR        0x6085 //鐢垫満鎬ュ仠鏃堕棿璁剧疆
#define MOTOR_IO_STOP_ADDR         0x2026 //澶栭儴IO鎬ュ仠妯″紡璁剧疆
#define MOTOR_CONTROLMODE_ADDR     0x6060 //鐢垫満鎺у埗妯″紡璁剧疆
#define POSCon_MAX_SPEED_ADDR      0x6081 //浣嶇疆妯″紡涓嬫渶澶ч�熷害璁剧疆
#define TOR_SLOPE_ADDR             0x6087 //杞�鐭╂枩鐜囪�剧疆
#define TARGET_POS_ADDR            0x607A //鐢垫満鐩�鏍囦綅缃�璁剧疆鍦板潃
#define TARGET_RPM_ADDR            0x60FF //鐢垫満鐩�鏍囪浆閫熻�剧疆鍦板潃
#define TARGET_TOR_ADDR            0x6071 //鐢垫満鐩�鏍囪浆鐭╄�剧疆鍦板潃
#define STATUS_WORD_ADDR           0x6041 //鐘舵�佸瓧
#define HUB_ERROR_ADDR             0x603F //椹卞姩鎶ラ敊鐘舵�佸湴鍧�

//TPDO鏄犲皠鍦板潃
#define TPDO_0_ADDR 0x1A00 //TPDO 0鏄犲皠鍦板潃
#define TPDO_1_ADDR 0x1A01 //TPDO 1鏄犲皠鍦板潃
#define TPDO_2_ADDR 0x1A02 //TPDO 2鏄犲皠鍦板潃
#define TPDO_3_ADDR 0x1A03 //TPDO 3鏄犲皠鍦板潃

//TPDO閫氫俊鍦板潃
#define TPDO_0_COMMADDR 0x1800 //TPDO 0閫氫俊鍦板潃
#define TPDO_1_COMMADDR 0x1801 //TPDO 1閫氫俊鍦板潃
#define TPDO_2_COMMADDR 0x1802 //TPDO 2閫氫俊鍦板潃
#define TPDO_3_COMMADDR 0x1803 //TPDO 3閫氫俊鍦板潃
#define SET_ID          0x01   //璁剧疆CAN涓婃姤鍦板潃
#define SET_TYPE        0x02   //璁剧疆瑙﹀彂涓婃姤鐨勪簨浠�
#define SET_FORB_TIME   0x03   //璁剧疆绂佹�㈡椂闂�
#define SET_EVENT_TIME  0x05   //璁剧疆浜嬩欢瀹氭椂鍣ㄧ殑鏃堕棿


/* 椹卞姩鍣ㄥ弬鏁拌�剧疆 */
#define ENCODER_ACCURACY       0
#define HALL_OFFSET            0
#define MOTOR_POLES            0
#define MOTOR_RATEDCUR         0
#define MOTOR_MAXCUR           0
#define MOTOR_MAXRPM           0
#define MOTOR_OVERLOAD_FACTOR  0
#define MOTOR_OVERLOAD_TIME    0
#define ENCODER_ERRORALARM     0
#define MAX_TEMPERATURE        0
#define SPEED_KP               0
#define SPEED_KI               0
#define SPEED_KF               0
#define SPEED_SMOOTH           0


//void set_encoder_accuracy(u32 hub,u16 data);
//void set_HallOffset(u32 hub,short data);
//void set_poles(u32 hub,u16 data);
//void set_rated_current(u32 hub,u16 data);
//void set_max_current(u32 hub,u16 data);
//void set_max_rpm(u32 hub,u16 data);
//void set_overload_factor(u32 hub,u16 data);
//void set_overload_time(u32 hub,u16 data);
//void set_encoder_ErrorAlarm(u32 hub,u16 data);
//void set_maxTemp(u32 hub,u16 data);
//void set_speed_kp(u32 hub,u16 data);
//void set_speed_ki(u32 hub,u16 data);
//void set_speed_kf(u32 hub,u16 data);
//void set_speed_smooth(u32 hub,u16 data);
//void set_smooth_kf(u32 hub,u16 data);
//void set_pos_kp(u32 hub,u16 data);
//void set_pos_kf(u32 hub,u16 data);
//void set_tor_smooth(u32 hub,u16 data);
//void set_tor_kp(u32 hub,u16 data);
//void set_tor_ki(u32 hub,u16 data);
//void set_SynCon_state(u32 hub,u8 data);
//void set_acctime(u32 hub,u32 data);
//void set_dectime(u32 hub,u32 data);
//void set_stoptime(u32 hub,u32 data);
//void set_IO_StopMode(u32 hub,u8 data);
//void EEPROM_Save(u32 hub);
//void set_ControlMode(u32 hub,u8 data);

void CAN_Stop(u32 hub);
void CAN_Enable(u32 hub);
void CAN_ClearError(u32 hub);
void Check_MotorError(u32 hub);
void set_MotorRPM(u32 hub,short L,short R);
void robot_ParkMode(u32 hub,u8 state);
void set_error_state(u32 hub,u8 state);
void HUB_Init(u32 hub);

#endif
