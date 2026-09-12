#include "DriverInit.h"

HubDrive_parameter Back_Drive;
HubDrive_parameter Front_Drive;

//鐢ㄤ簬鎺ユ敹鍙嶉�堢殑ID淇″彿
u32 FeedBackID=0;

//鑾峰彇璁剧疆鍙傛暟鏁版嵁鍑芥暟
uint8_t* get_setparam(uint8_t rw, uint8_t bytes,uint16_t addr,uint8_t index,uint32_t writedata)
{
	static uint8_t user_needdata[8];
    uint8_t i=0;

    if(bytes>4) return user_needdata;//鏈�澶у彧鑳藉啓鍏�4瀛楄妭鏁版嵁

	if(rw==0x01) // r=0,w=1,鍐欐暟鎹�
	{
			 if (bytes==1) user_needdata[0]=0x2f;
		else if (bytes==2) user_needdata[0]=0x2B;
		else if (bytes==3) user_needdata[0]=0x27;
		else if (bytes==4) user_needdata[0]=0x23;
	}
	else if(rw==0x10) // r=1,w=0,璇绘暟鎹�
	{
			 if (bytes==1) user_needdata[0]=0x4f;
		else if (bytes==2) user_needdata[0]=0x4B;
		else if (bytes==3) user_needdata[0]=0x47;
		else if (bytes==4) user_needdata[0]=0x43;
        bytes=0;//璧嬪�煎畬姣曞悗娓呯┖,璁╂暟鎹�鍖轰负鍏�0,鍥犱负璇绘暟鎹�鏃朵笉闇�瑕佹暟鎹�鍖烘湁鏁版嵁
	}

	user_needdata[1] = addr&0xff;      //鍙栧嚭鍦板潃浣�8浣�
	user_needdata[2] = (addr>>8)&0xff; //鍙栧嚭鍦板潃楂�8浣�
	user_needdata[3] = index;          //鍙栧嚭绱㈠紩鍊�

    //瑕佸啓鍏ョ殑鏁版嵁浠庝綆浣嶅悜楂樹綅鎺掑簭鍙栧嚭
    for(i=0;i<4;i++)
    {
        user_needdata[4+i] = ( writedata>>(8*i) )&0xff;
    }

	return user_needdata;
}

//閫夋嫨浣跨敤鍝�涓�璺疌AN鍙戦��
void use_canX_send(u32 id,u8 *data)
{
	#if canX==1
	CAN1_Send_Num(id,data);//閫夋嫨CAN1
	#elif canX==2
	CAN2_Send_Num(id,data);//閫夋嫨CAN2
	#endif
	delay_ms(1); //鏁版嵁鍙戦�侀�熷害涓嶅疁杩囧揩,椹卞姩鎺ユ敹閫熷害鏈夐檺
}

//璁剧疆缂栫爜鍣ㄧ嚎鏁�
void set_encoder_accuracy(u32 hub,u16 data)
{
	u8 *p;

	//宸﹁疆
	p = get_setparam(write_hub,2,ENCODER_ACCURACY_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,ENCODER_ACCURACY_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//璁剧疆鐢垫満HALL鍋忕Щ瑙掑害
void set_HallOffset(u32 hub,short data)
{
	u8 *p;

	//闄愬箙
	if(data<-360) data=-360;
	if(data> 360) data=360;

	//宸﹁疆
	p = get_setparam(write_hub,2,HALL_OFFSET_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,HALL_OFFSET_ADDR,Index_R,data);
	use_canX_send(hub,p);
	
}

//璁剧疆鐢垫満鏋佸�规暟
void set_poles(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data<4) data=4;
	if(data>64) data=64;

	//宸﹁疆
	p = get_setparam(write_hub,2,MOTOR_POLES_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,MOTOR_POLES_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//璁剧疆鐢垫満棰濆畾鐢垫祦
void set_rated_current(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>150) data=150;

	//宸﹁疆
	p = get_setparam(write_hub,2,MOTOR_RATEDCUR_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,MOTOR_RATEDCUR_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//璁剧疆鐢垫満鏈�澶х數娴�
void set_max_current(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>300) data=300;

	//宸﹁疆
	p = get_setparam(write_hub,2,MOTOR_MAXCUR_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,MOTOR_MAXCUR_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//璁剧疆鐢垫満鏈�澶ц浆閫�
void set_max_rpm(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data<1)    data=1;
	if(data>1000) data=1000;

	p = get_setparam(write_hub,2,MOTOR_MAXRPM_ADDR,Index_None,data);
	use_canX_send(hub,p);
}

//璁剧疆杩囪浇淇濇姢绯绘暟
void set_overload_factor(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>300) data=300;

	//宸﹁疆
	p = get_setparam(write_hub,2,MOTOR_OVERLOAD_FACTOR_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,MOTOR_OVERLOAD_FACTOR_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//璁剧疆杩囪浇淇濇姢鏃堕棿
void set_overload_time(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>6553) data=6553;

	//宸﹁疆
	p = get_setparam(write_hub,2,MOTOR_OVERLOAD_TIME_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,MOTOR_OVERLOAD_TIME_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//璁剧疆缂栫爜鍣ㄨ秴宸�鎶ヨ�﹂槇鍊�
void set_encoder_ErrorAlarm(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>6553) data=6553;

	//宸﹁疆
	p = get_setparam(write_hub,2,ENCODER_ERRORALARM_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,ENCODER_ERRORALARM_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//璁剧疆娓╁害淇濇姢闃堝��
void set_maxTemp(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>1200) data=1200;

	//宸﹁疆
	p = get_setparam(write_hub,2,MAX_TEMPERATURE_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,MAX_TEMPERATURE_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//閫熷害鐜痥p
void set_speed_kp(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>30000) data=30000;

	//宸﹁疆
	p = get_setparam(write_hub,2,SPEED_KP_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,SPEED_KP_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//閫熷害鐜痥i
void set_speed_ki(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>30000) data=30000;

	//宸﹁疆
	p = get_setparam(write_hub,2,SPEED_KI_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,SPEED_KI_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//閫熷害鍓嶉�坘f
void set_speed_kf(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>30000) data=30000;

	//宸﹁疆
	p = get_setparam(write_hub,2,SPEED_KF_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,SPEED_KF_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//閫熷害骞虫粦绯绘暟
void set_speed_smooth(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>30000) data=30000;

	//宸﹁疆
	p = get_setparam(write_hub,2,SPEED_SMOOTH_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,SPEED_SMOOTH_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//鍓嶉�堝钩婊戠郴鏁�
void set_smooth_kf(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>30000) data=30000;

	//宸﹁疆
	p = get_setparam(write_hub,2,SMOOTH_KF_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,SMOOTH_KF_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//浣嶇疆鐜痥p
void set_pos_kp(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>30000) data=30000;

	//宸﹁疆
	p = get_setparam(write_hub,2,POS_KP_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,POS_KP_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//浣嶇疆鍓嶉�坘f
void set_pos_kf(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>30000) data=30000;

	//宸﹁疆
	p = get_setparam(write_hub,2,POS_KF_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,POS_KF_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//杞�鐭╁钩婊戠郴鏁�
void set_tor_smooth(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>30000) data=30000;

	//宸﹁疆
	p = get_setparam(write_hub,2,TOR_SMOOTH_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,TOR_SMOOTH_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//鐢垫祦鐜痥p
void set_tor_kp(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>30000) data=30000;

	//宸﹁疆
	p = get_setparam(write_hub,2,TOR_KP_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,TOR_KP_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//鐢垫祦鐜痥i
void set_tor_ki(u32 hub,u16 data)
{
	u8 *p;

	//闄愬箙
	if(data>30000) data=30000;

	//宸﹁疆
	p = get_setparam(write_hub,2,TOR_KI_ADDR,Index_L,data);
	use_canX_send(hub,p);

	//鍙宠疆
	p = get_setparam(write_hub,2,TOR_KI_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//璁剧疆鐢垫満鎺у埗鍚屾��/寮傛�ユ帶鍒�(0:寮傛�ユ帶鍒� 1:鍚屾�ユ帶鍒�)
void set_SynCon_state(u32 hub,u8 data)
{
	u8 *p;

	if(data!=0) data=1;

	p = get_setparam(write_hub,2,MOTOR_SYNCON_ADDR,Index_None,data);
	use_canX_send(hub,p);
}

//璁剧疆鐢垫満鍔犻�熸椂闂�
void set_acctime(u32 hub,u32 data)
{
	u8 *p;
	if(data>32767) data=32767;
	p = get_setparam(write_hub,4,MOTOR_ACCTIME_ADDR,Index_L,data);
	use_canX_send(hub,p);
	p = get_setparam(write_hub,4,MOTOR_ACCTIME_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//璁剧疆鐢垫満鍑忛�熸椂闂�
void set_dectime(u32 hub,u32 data)
{
	u8 *p;
	if(data>32767) data=32767;
	p = get_setparam(write_hub,4,MOTOR_DECTIME_ADDR,Index_L,data);
	use_canX_send(hub,p);
	p = get_setparam(write_hub,4,MOTOR_DECTIME_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//璁剧疆鐢垫満鎬ュ仠鏃堕棿
void set_stoptime(u32 hub,u32 data)
{
	u8 *p;
	if(data>32767) data=32767;
	p = get_setparam(write_hub,4,MOTOR_STOPTIME_ADDR,Index_L,data);
	use_canX_send(hub,p);
	p = get_setparam(write_hub,4,MOTOR_STOPTIME_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//浣嶇疆鎺у埗妯″紡涓嬫渶澶ч�熷害璁剧疆
void set_PosControl_MaxSpeed(u32 hub,u32 data)
{
	u8 *p;
	if(data>1000) data=1000;
	p = get_setparam(write_hub,4,POSCon_MAX_SPEED_ADDR,Index_L,data);
	use_canX_send(hub,p);
	p = get_setparam(write_hub,4,POSCon_MAX_SPEED_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//杞�鐭╂枩鐜囪�剧疆
void set_Tor_Slope(u32 hub,u32 data)
{
	u8 *p;
	p = get_setparam(write_hub,4,TOR_SLOPE_ADDR,Index_L,data);
	use_canX_send(hub,p);
	p = get_setparam(write_hub,4,TOR_SLOPE_ADDR,Index_R,data);
	use_canX_send(hub,p);
}

//璁剧疆鐢垫満澶栭儴鎬ュ仠鐨勬柟寮�(0:鎬ュ仠鍚庨攣浣忚疆瀛� 1:鎬ュ仠鍚庨噴鏀捐疆瀛�)
void set_IO_StopMode(u32 hub,u8 data)
{
	u8 *p;
	if(data!=0) data=1;
	p = get_setparam(write_hub,2,MOTOR_IO_STOP_ADDR,0x03,data);
	use_canX_send(hub,p);
}

//灏嗗弬鏁颁繚瀛樺埌EEPROM
void EEPROM_Save(u32 hub)
{
	u8 SaveEEPROM[8]={0x2B,0x10,0x20,0x00,0x01,0x00,0x00,0x00};
	use_canX_send(hub,SaveEEPROM);
}

//璁剧疆鎺у埗妯″紡
//POS_REL_CONTROL 鐩稿�逛綅缃�鎺у埗
//POS_ABS_CONTROL 缁濆�逛綅缃�鎺у埗
//VEL_CONTROL     閫熷害鎺у埗
//TOR_CONTROL     鍔涚煩鎺у埗
void set_ControlMode(u32 hub,u8 data)
{
	u8 *p;
	if(data>4) data=0;
	p = get_setparam(write_hub,1,MOTOR_CONTROLMODE_ADDR,Index_None,data);
	use_canX_send(hub,p);
}

//鐢垫満澶辫兘
void CAN_Stop(u32 hub)
{
	u8 Stop[8]= {0x2b, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
	use_canX_send(hub,Stop);
}

//鐢垫満浣胯兘
void CAN_Enable(u32 hub)
{
	u8 enable1[8]= {0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
	u8 enable2[8]= {0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};
	u8 enable3[8]= {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};
	use_canX_send(hub,enable1);
	use_canX_send(hub,enable2);
	use_canX_send(hub,enable3);
}

//浣嶇疆妯″紡涓嬪惎鍔ㄧ浉瀵硅繍鍔�
void Enable_POS_ABS(u32 hub)
{
	u8 enable1[8]= {0x2b, 0x40, 0x60, 0x00, 0x4F, 0x00, 0x00, 0x00};
	u8 enable2[8]= {0x2b, 0x40, 0x60, 0x00, 0x5F, 0x00, 0x00, 0x00};
	use_canX_send(hub,enable1);
	use_canX_send(hub,enable2);
}

//浣嶇疆妯″紡涓嬪惎鍔ㄧ粷瀵硅繍鍔�
void Enable_POS_REL(u32 hub)
{
	u8 enable1[8]= {0x2b, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
	u8 enable2[8]= {0x2b, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00};
	use_canX_send(hub,enable1);
	use_canX_send(hub,enable2);
}

//鐢垫満娓呴櫎鎶ラ敊
void CAN_ClearError(u32 hub)
{
	u8 Clear_Error[8]= {0x2b, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
	use_canX_send(hub,Clear_Error);
}

//璁剧疆鐢垫満杞�閫�
void set_MotorRPM(u32 hub,short L,short R)
{
	#if 1
	u8 set_rpm[8]= {0x23, 0xFF, 0x60, 0x03, 0x00, 0x00, 0x00, 0x00};
	set_rpm[4] = L;
	set_rpm[5] = L>>8;
	set_rpm[6] = R;
	set_rpm[7] = R>>8;
	
	#else  //娉�锛歶se_canX_send()鍐呯疆delay_ms(1)锛屽湪楂橀�戜娇鐢ㄧ殑鐜�澧冧笅涓嶅疁浣跨敤锛屽�规槗閫犳垚rtos姝绘満
	
	uint8_t* p;
	uint32_t rpm;
	rpm = (uint32_t)R<<16|(uint16_t)L;
	p = get_setparam(write_hub,4,TARGET_RPM_ADDR,0x03,rpm);
	use_canX_send(hub,p);
	
	#endif
	
	#if canX==1
	CAN1_Send_Num(id,data);//閫夋嫨CAN1
	#elif canX==2
	CAN2_Send_Num(hub,set_rpm);//閫夋嫨CAN2
	#endif
//	delay_ms(5);

}

//璁剧疆鐢垫満鐩�鏍囦綅缃�
void set_MotorPOS(u32 hub,int L,int R)
{
	u8* p;
	p = get_setparam(write_hub,4,TARGET_POS_ADDR,Index_L,L);//璁剧疆宸︾數鏈虹洰鏍囦綅缃�
	use_canX_send(hub,p);
	
	p = get_setparam(write_hub,4,TARGET_POS_ADDR,Index_R,R);//璁剧疆鍙崇數鏈虹洰鏍囦綅缃�
	use_canX_send(hub,p);
}

//璁剧疆鐢垫満鐩�鏍囪浆鐭�
void set_MotorTOR(u32 hub,short L,short R)
{
	u8* p;
	p = get_setparam(write_hub,2,TARGET_TOR_ADDR,Index_L,L);//璁剧疆宸︾數鏈虹洰鏍囦綅缃�
	use_canX_send(hub,p);
	
	p = get_setparam(write_hub,2,TARGET_TOR_ADDR,Index_R,R);//璁剧疆鍙崇數鏈虹洰鏍囦綅缃�
	use_canX_send(hub,p);
}

//璁块棶鐘舵�佸瓧
void Read_StatusWord(u32 hub)
{
	u8* p;
	p = get_setparam(read_hub,4,STATUS_WORD_ADDR,Index_None,0);//璁块棶鐘舵�佸瓧
	use_canX_send(hub,p);
	
	//鏍规嵁鍙戦�佺殑ID涓嶅悓鏉ュ垽鏂�鏄�鍝�涓�涓�椹卞姩鍣ㄨ繑鍥炴暟鎹�
	     if(hub==COBID_Send_1)   FeedBackID=COBID_Recv_1;
	else if(hub==COBID_Send_2)   FeedBackID=COBID_Recv_2;
	
	//杩斿洖鏁版嵁鍐呭��:
	//     id          0       1    2   3      4           5         6           7
	// COBID_Recv_x    0x43  0x41 0x60 0x00 宸︾數鏈轰綆8浣� 宸︾數鏈洪珮8浣� 鍙崇數鏈轰綆8浣� 鍙崇數鏈洪珮8浣�
	
	/* 鐘舵�佸瓧璁块棶demo锛�
	if(id==FeedBackID) //璁块棶ID姝ｇ‘
	{
		if( ( (u16)msg[2]<<8|msg[1] )==STATUS_WORD_ADDR  ) //浼犺緭鍐呭�规�ｇ‘
		{
			u16 L_State = msg[5]<<8 | msg[4]; //鑾峰彇宸︾數鏈虹殑鐘舵�佸瓧
			u16 R_State = msg[7]<<8 | msg[6]; //鑾峰彇鍙崇數鏈虹殑鐘舵�佸瓧
			if( (L_State>>10)&0x01 == 0 )
			{
			  printf("宸︾數鏈虹洰鏍囬�熷害宸插埌杈撅紒");
			}
		}
	}
	*/
}

//璇诲彇椹卞姩鍣ㄨ蒋浠剁増鏈�鍙�
void read_SoftwareVersion(u32 hub)
{
	u8* p;
	p = get_setparam(read_hub,2,0x2031,Index_None,0);//璇昏蒋浠剁増鏈�鍙�
	use_canX_send(hub,p);
	//杩斿洖鍊硷細
	// 4b 0A 10 00 浣�8 楂�8 
}

//椹昏溅妯″紡
void robot_ParkMode(u32 hub,u8 state)
{
	u8* p;
	p = get_setparam(write_hub,2,0x2026,0x04,state);
	use_canX_send(hub,p);
}

void set_error_state(u32 hub,u8 state)
{
	u8* p;
	p = get_setparam(write_hub,2,0x2026,0x01,state);
	use_canX_send(hub,p);
}

//妫�鏌ョ數鏈烘姤閿欐儏鍐�
void Check_MotorError(u32 hub)
{
	u8 data[8]={0x43,0x3F,0x60,0x00,0,0,0,0};
	
	#if canX==1
	CAN1_Send_Num(hub,data);//閫夋嫨CAN1
	#elif canX==2
	CAN2_Send_Num(hub,data);//閫夋嫨CAN2
	#endif
	//鏍规嵁鍙戦�佺殑ID涓嶅悓鏉ュ垽鏂�鏄�鍝�涓�涓�椹卞姩鍣ㄨ繑鍥炴暟鎹�
	     if(hub==COBID_Send_1)   FeedBackID=COBID_Recv_1;
	else if(hub==COBID_Send_2)   FeedBackID=COBID_Recv_2;
}

//鍚�鍔ㄧ紪鐮佸櫒鏁版嵁涓诲姩涓婃姤鍔熻兘
void start_encoder_tpdo(u32 hub,u32 recvid)
{
	u8* p;
	p = get_setparam(write_hub,1,TPDO_0_ADDR,Index_None,0);//娓呯┖TPDO 0涓婃墍鏈夋槧灏�
	use_canX_send(hub,p);
	
	//鏄犲皠鍦板潃0x606C,绱㈠紩0x01,鏁版嵁闀垮害涓�0x20鐨勬暟鎹�,鏄犲皠鍒癟PDO0鐨�0x01涓�
	p = get_setparam(write_hub,4,TPDO_0_ADDR,0x01,0x606C0120);
	use_canX_send(hub,p);
	
	//鏄犲皠鍦板潃0x606C,绱㈠紩0x02,鏁版嵁闀垮害涓�0x20鐨勬暟鎹�,鏄犲皠鍒癟PD0鐨�0x02涓�
	p = get_setparam(write_hub,4,TPDO_0_ADDR,0x02,0x606C0220);
	use_canX_send(hub,p);
	
	//璁剧疆鏁版嵁涓婃姤鏃剁殑甯�ID
	p = get_setparam(write_hub,4,TPDO_0_COMMADDR,SET_ID,recvid);
	use_canX_send(hub,p);
	
	//璁剧疆涓婃姤绫诲瀷涓�254浜嬩欢
	//254浜嬩欢:琚�鏄犲皠鐨勬暟鎹�鑻ュ彂鐢熸敼鍙樺垯绔嬪嵆涓婃姤
	//255浜嬩欢:浜嬩欢瀹氭椂鍣ㄧ殑璁℃椂鍒拌揪鍚庝笂鎶�
	p = get_setparam(write_hub,1,TPDO_0_COMMADDR,SET_TYPE,0xFE);
	use_canX_send(hub,p);
	
	//璁剧疆绂佹�㈡椂闂�(鏁版嵁涓婃姤鐨勬渶灏忔椂闂撮棿闅�,濡傛灉鍒拌揪浜嗚繖涓�鏃堕棿,鏁版嵁浠嶆湭鍙戠敓鏀瑰彉,浼氫富鍔ㄤ笂鎶ユ暟鎹�)
	//鍗曚綅 0.1ms --> 渚嬭�剧疆 200鍒欐椂闂翠负 200*0.1 = 20ms
	p = get_setparam(write_hub,2,TPDO_0_COMMADDR,SET_FORB_TIME,200);
	use_canX_send(hub,p);
	
	//璁剧疆浜嬩欢瀹氭椂鍣�(褰撹�℃椂鍒拌揪鏃�,浼氳Е鍙�1娆′笂鎶�)
	
	//[鏍规嵁鎵嬪唽渚嬪瓙,璁剧疆254浜嬩欢鏃朵笉璁剧疆浜嬩欢瀹氭椂鍣�,鍙�璁剧疆绂佹�㈡椂闂达紱璁剧疆255浜嬩欢鏃惰�剧疆瀹氭椂鍣ㄤ簨浠�,涓嶈�剧疆绂佹�㈡椂闂碷
	
	//寮�鍚�2涓猅PDO0鏄犲皠
	p = get_setparam(write_hub,1,TPDO_0_ADDR,Index_None,2);
	use_canX_send(hub,p);
	
	//淇濆瓨鍙傛暟鍒癊EPROM
	EEPROM_Save(hub);
	
	//鍚�鍔ㄦ槧灏�
	u8 start[8]= {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//鍚�鍔ㄦ槧灏勫伐浣滐紝鏁版嵁寮�濮嬭緭鍑�
	use_canX_send(0x00,start);
	// 0        1       2        3       4      5        6      7
	//宸︾數鏈�8 宸︾數鏈�16 宸︾數鏈�24 宸︾數鏈�32 鍙崇數鏈�8 鍙崇數鏈�16 鍙崇數鏈�24 鍙崇數鏈�32
}


//鍚�鍔ㄧ數鏈虹數娴併�佹俯搴︿富鍔ㄤ笂鎶�
void start_motor_state_tpdo(u32 hub,u32 recvid)
{
	u8* p;
	p = get_setparam(write_hub,1,TPDO_1_ADDR,Index_None,0);//娓呯┖TPDO 1涓婃墍鏈夋槧灏�
	use_canX_send(hub,p);
	
	//鏄犲皠鍦板潃0x6077,绱㈠紩0x01,鏁版嵁闀垮害涓�0x10鐨勬暟鎹�,鏄犲皠鍒癟PDO1鐨�0x01涓�
	p = get_setparam(write_hub,4,TPDO_1_ADDR,0x01,0x60770110);//宸︾數鏈虹數娴�
	use_canX_send(hub,p);
	
	//鏄犲皠鍦板潃0x2032,绱㈠紩0x01,鏁版嵁闀垮害涓�0x10鐨勬暟鎹�,鏄犲皠鍒癟PDO1鐨�0x02涓�
	p = get_setparam(write_hub,4,TPDO_1_ADDR,0x02,0x20320110);//宸︾數鏈烘俯搴�
	use_canX_send(hub,p);
	
	//鏄犲皠鍦板潃0x6077,绱㈠紩0x02,鏁版嵁闀垮害涓�0x10鐨勬暟鎹�,鏄犲皠鍒癟PDO1鐨�0x03涓�
	p = get_setparam(write_hub,4,TPDO_1_ADDR,0x03,0x60770210);//鍙崇數鏈虹數娴�
	use_canX_send(hub,p);
	
	//鏄犲皠鍦板潃0x2032,绱㈠紩0x01,鏁版嵁闀垮害涓�0x10鐨勬暟鎹�,鏄犲皠鍒癟PDO1鐨�0x04涓�
	p = get_setparam(write_hub,4,TPDO_1_ADDR,0x04,0x20320210);//鍙崇數鏈烘俯搴�
	use_canX_send(hub,p);
	
	//璁剧疆涓婃姤CAN ID
	p = get_setparam(write_hub,4,TPDO_1_COMMADDR,SET_ID,recvid);
	use_canX_send(hub,p);
	
	//璁剧疆涓婃姤绫诲瀷涓�255浜嬩欢
	//254浜嬩欢:琚�鏄犲皠鐨勬暟鎹�鑻ュ彂鐢熸敼鍙樺垯绔嬪嵆涓婃姤
	//255浜嬩欢:浜嬩欢瀹氭椂鍣ㄧ殑璁℃椂鍒拌揪鍚庝笂鎶�
	p = get_setparam(write_hub,1,TPDO_1_COMMADDR,SET_TYPE,0xFF);
	use_canX_send(hub,p);
	
	//璁剧疆浜嬩欢瀹氭椂鍣�(褰撹�℃椂鍒拌揪鏃�,浼氳Е鍙�1娆′笂鎶�)
	//鍗曚綅 0.1ms --> 渚嬭�剧疆 400鍒欐椂闂翠负 400*0.1 = 40ms
	p = get_setparam(write_hub,2,TPDO_1_COMMADDR,SET_EVENT_TIME,400);
	use_canX_send(hub,p);
	
	//[鏍规嵁鎵嬪唽渚嬪瓙,璁剧疆254浜嬩欢鏃朵笉璁剧疆浜嬩欢瀹氭椂鍣�,鍙�璁剧疆绂佹�㈡椂闂达紱璁剧疆255浜嬩欢鏃惰�剧疆瀹氭椂鍣ㄤ簨浠�,涓嶈�剧疆绂佹�㈡椂闂碷
	
	//寮�鍚�4涓猅PDO0鏄犲皠
	p = get_setparam(write_hub,1,TPDO_1_ADDR,Index_None,4);
	use_canX_send(hub,p);
	
	//淇濆瓨鍙傛暟鍒癊EPROM
	EEPROM_Save(hub);
	
	//鍚�鍔ㄦ槧灏�
	u8 start[8]= {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//鍚�鍔ㄦ槧灏勫伐浣滐紝鏁版嵁寮�濮嬭緭鍑�
	use_canX_send(0x00,start);
	//  0         1         2        3       4         5        6        7
	//宸︾數娴佷綆8 宸︾數娴侀珮8 宸︽俯搴︿綆8 宸︽俯搴﹂珮8 鍙崇數娴佷綆8 鍙崇數娴侀珮8 鍙虫俯搴︿綆8 鍙虫俯搴﹂珮8 
}

//鍚�鍔ㄨ嚜妫�鏁版嵁涓诲姩涓婃姤
void start_selfcheck_tpdo(u32 hub,u32 recvid)
{
	u8* p;
	p = get_setparam(write_hub,1,TPDO_2_ADDR,Index_None,0);//娓呯┖TPDO 2涓婃墍鏈夋槧灏�
	use_canX_send(hub,p);
	
	//鏄犲皠鍦板潃0x603F,绱㈠紩0x00,鏁版嵁闀垮害涓�0x20鐨勬暟鎹�,鏄犲皠鍒癟PDO2鐨�0x01涓�
	p = get_setparam(write_hub,4,TPDO_2_ADDR,0x01,0x603F0020);//鑷�妫�鏁版嵁
	use_canX_send(hub,p);
	
	//鏄犲皠鐘舵�佸瓧鍒癟PDO2鐨�0x02涓�
	p = get_setparam(write_hub,4,TPDO_2_ADDR,0x02,0x60410020);//鑷�妫�鏁版嵁
	use_canX_send(hub,p);
	
	//璁剧疆涓婃姤CAN ID
	p = get_setparam(write_hub,4,TPDO_2_COMMADDR,SET_ID,recvid);
	use_canX_send(hub,p);
	
	//璁剧疆涓婃姤绫诲瀷涓�255浜嬩欢
	//254浜嬩欢:琚�鏄犲皠鐨勬暟鎹�鑻ュ彂鐢熸敼鍙樺垯绔嬪嵆涓婃姤
	//255浜嬩欢:浜嬩欢瀹氭椂鍣ㄧ殑璁℃椂鍒拌揪鍚庝笂鎶�
	p = get_setparam(write_hub,1,TPDO_2_COMMADDR,SET_TYPE,0xFF);
	use_canX_send(hub,p);
	
	//璁剧疆浜嬩欢瀹氭椂鍣�(褰撹�℃椂鍒拌揪鏃�,浼氳Е鍙�1娆′笂鎶�)
	//鍗曚綅 0.1ms --> 渚嬭�剧疆 2000鍒欐椂闂翠负 2000*0.1 = 200ms
	p = get_setparam(write_hub,2,TPDO_2_COMMADDR,SET_EVENT_TIME,2000);
	use_canX_send(hub,p);
	
	//[鏍规嵁鎵嬪唽渚嬪瓙,璁剧疆254浜嬩欢鏃朵笉璁剧疆浜嬩欢瀹氭椂鍣�,鍙�璁剧疆绂佹�㈡椂闂达紱璁剧疆255浜嬩欢鏃惰�剧疆瀹氭椂鍣ㄤ簨浠�,涓嶈�剧疆绂佹�㈡椂闂碷
	
	//寮�鍚�2涓猅PDO0鏄犲皠
	p = get_setparam(write_hub,1,TPDO_2_ADDR,Index_None,2);
	use_canX_send(hub,p);
	
	//淇濆瓨鍙傛暟鍒癊EPROM
	EEPROM_Save(hub);
	
	//鍚�鍔ㄦ槧灏�
	u8 start[8]= {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//鍚�鍔ㄦ槧灏勫伐浣滐紝鏁版嵁寮�濮嬭緭鍑�
	use_canX_send(0x00,start);
	//  0         1         2        3       4             5           6             7
	//宸︾數鏈轰綆8 宸︾數鏈洪珮8 鍙崇數鏈轰綆8  鍙崇數鏈洪珮8  鐘舵�佸瓧宸︿綆8  鐘舵�佸瓧宸﹂珮8  鐘舵�佸瓧鍙充綆8     鐘舵�佸瓧鍙抽珮8 
}


//浣嶇疆鎺у埗Demo
void PosControl_Demo(u32 hub)
{
	//鍒濆�嬪寲:
	set_ControlMode(hub,POS_REL_CONTROL);//璁剧疆鐢垫満宸ヤ綔鍦ㄤ綅缃�妯″紡
	set_acctime(hub,100);//璁剧疆鐢垫満S鍨嬪姞閫熸椂闂翠负100ms
	set_dectime(hub,100);//璁剧疆鐢垫満S鍨嬪噺閫熸椂闂翠负100ms
	set_PosControl_MaxSpeed(hub,60);//璁剧疆鐢垫満鏈�澶ч�熷害涓�60rpm
	CAN_Enable(hub); //浣胯兘鐢垫満杩愬姩
	//娉�锛氳嫢闇�瑕佸垎鍒�璁剧疆宸﹀彸杞�,闇�瑕佽嚜琛屽垱寤哄嚱鏁�,鍙傝�冧笂闈㈠嚱鏁板嵆鍙�銆�
	
	//鎺у埗demo:
	//鐩稿�逛綅缃�鍚屾�ユ帶鍒禿emo
	set_MotorPOS(hub,32000,-32000);//璁剧疆宸﹀彸鐩�鏍囦綅缃�涓�32000
	Enable_POS_ABS(hub);          //鍚�鍔ㄧ浉瀵硅繍鍔�
	set_MotorPOS(hub,-32000,32000);//璁剧疆宸﹀彸鐩�鏍囦綅缃�涓�32000
	Enable_POS_ABS(hub);          //鍚�鍔ㄧ浉瀵硅繍鍔�
	CAN_Stop(hub);                //鍋滄�㈣繍鍔�
	
	//缁濆�逛綅缃�鍚屾�ユ帶鍒禿emo
	set_MotorPOS(hub,32000,-32000);//璁剧疆宸﹀彸鐩�鏍囦綅缃�涓�32000
	Enable_POS_REL(hub);          //鍚�鍔ㄧ浉瀵硅繍鍔�
	set_MotorPOS(hub,-32000,32000);//璁剧疆宸﹀彸鐩�鏍囦綅缃�涓�32000
	Enable_POS_REL(hub);          //鍚�鍔ㄧ浉瀵硅繍鍔�
	CAN_Stop(hub);                //鍋滄�㈣繍鍔�
}


//鍔涚煩鎺у埗demo
void TorControl_Demo(u32 hub)
{
	//鍒濆�嬪寲:
	set_ControlMode(hub,TOR_CONTROL);//璁剧疆鍔涚煩鎺у埗
	set_SynCon_state(hub,0);//璁剧疆鍚屾�ユ垨寮傛�ユ帶鍒�
	set_Tor_Slope(hub,100); //璁剧疆宸﹀彸杞�鐭╃殑鏂滅巼
	CAN_Enable(hub);        //浣胯兘鎺у埗
	
	//鎺у埗:
	set_MotorTOR(hub,1000,-1000);//璁剧疆鐩�鏍囪浆鐭�
	CAN_Stop(hub);                //鍋滄�㈣繍鍔�
	
}

void HUB_Init(u32 hub)
{
	/* 椹卞姩鍣ㄥ熀鏈�鍙傛暟閰嶇疆 */
	#if 1
	
	if(Car_Mode==S300) //S300 8瀵歌疆姣傜數鏈�
	{
		set_encoder_accuracy(hub,1024);  //缂栫爜鍣ㄧ嚎鏁�
		set_HallOffset(hub,0);           //HALL鍋忕Щ瑙掑害
		set_poles(hub,15);               //鐢垫満鏋佸�规暟
		set_rated_current(hub,150);      //棰濆畾鐢垫祦
		set_max_current(hub,300);        //鏈�澶х數娴�
		set_max_rpm(hub,1000);           //鏈�澶ц浆閫�
		set_overload_factor(hub,200);    //杩囪浇绯绘暟
		set_overload_time(hub,800);      //杩囪浇鏃堕棿
		set_encoder_ErrorAlarm(hub,409); //缂栫爜鍣ㄨ秴宸�闃堝��
		set_maxTemp(hub,800);            //娓╁害淇濇姢闃堝��
		set_speed_kp(hub,500);           //閫熷害鐜痥p
		set_speed_ki(hub,333);           //閫熷害鐜痥i
		set_speed_kf(hub,1000);          //閫熷害鐜痥f
		
		//tmp:
		set_tor_smooth(hub,100);         //杞�鐭╁钩婊戠郴鏁�
		set_tor_kp(hub,700);             //鐢垫祦鐜疜p
		set_tor_ki(hub,250);              //鐢垫祦鐜痥i
		
		set_speed_smooth(hub,50);        //閫熷害骞虫粦绯绘暟
		set_smooth_kf(hub,100);          //鍓嶉�堝钩婊戠郴鏁�
		set_acctime(hub,30);             //鍔犻�熸椂闂�
		set_dectime(hub,30);             //鍑忛�熸椂闂�
		set_stoptime(hub,100);           //鎬ュ仠鏃堕棿
		set_IO_StopMode(hub,IO_UNLOCK);  //澶栭儴IO鎬ュ仠鏃朵负瑙ｈ酱妯″紡
		EEPROM_Save(hub);                //淇濆瓨鍙傛暟鍒癊EPROM
	}
	else if(Car_Mode==S150||Car_Mode==S100) //S300 Mini 5瀵歌疆姣傜數鏈�
	{
		if(Car_Mode==S150)
			set_encoder_accuracy(hub,1024);  //缂栫爜鍣ㄧ嚎鏁�
		else
			set_encoder_accuracy(hub,4096);  //缂栫爜鍣ㄧ嚎鏁�
		
		set_HallOffset(hub,240);         //HALL鍋忕Щ瑙掑害
		set_poles(hub,10);               //鐢垫満鏋佸�规暟
		set_rated_current(hub,150);      //棰濆畾鐢垫祦
		set_max_current(hub,300);        //鏈�澶х數娴�
		set_max_rpm(hub,1000);           //鏈�澶ц浆閫�
		set_overload_factor(hub,200);    //杩囪浇绯绘暟
		set_overload_time(hub,800);      //杩囪浇鏃堕棿
		set_encoder_ErrorAlarm(hub,1638);//缂栫爜鍣ㄨ秴宸�闃堝��
		set_maxTemp(hub,800);            //娓╁害淇濇姢闃堝��
		set_speed_kp(hub,400);           //閫熷害鐜痥p
		set_speed_ki(hub,200);           //閫熷害鐜痥i
		set_speed_kf(hub,1000);          //閫熷害鐜痥f
		set_speed_smooth(hub,50);        //閫熷害骞虫粦绯绘暟
		set_smooth_kf(hub,100);          //鍓嶉�堝钩婊戠郴鏁�
		set_acctime(hub,5);             //鍔犻�熸椂闂�
		set_dectime(hub,5);             //鍑忛�熸椂闂�
		set_stoptime(hub,100);           //鎬ュ仠鏃堕棿
		set_IO_StopMode(hub,IO_UNLOCK);  //澶栭儴IO鎬ュ仠鏃朵负瑙ｈ酱妯″紡
		EEPROM_Save(hub);                //淇濆瓨鍙傛暟鍒癊EPROM
		read_SoftwareVersion(hub);		 //璇昏蒋浠剁増鏈�鍙�
	}
	else if(Car_Mode==S200) //鏃犲埛鍥涢┍杞�
	{
//		set_encoder_accuracy(hub,4096);  //缂栫爜鍣ㄧ嚎鏁�
//		set_HallOffset(hub,0);           //HALL鍋忕Щ瑙掑害
//		set_poles(hub,15);               //鐢垫満鏋佸�规暟
//		set_rated_current(hub,150);      //棰濆畾鐢垫祦
//		set_max_current(hub,300);        //鏈�澶х數娴�
//		set_max_rpm(hub,1000);           //鏈�澶ц浆閫�
//		set_overload_factor(hub,200);    //杩囪浇绯绘暟
//		set_overload_time(hub,800);      //杩囪浇鏃堕棿
//		set_encoder_ErrorAlarm(hub,409); //缂栫爜鍣ㄨ秴宸�闃堝��
//		set_maxTemp(hub,800);            //娓╁害淇濇姢闃堝��
//		set_speed_kp(hub,450);           //閫熷害鐜痥p
//		set_speed_ki(hub,250);           //閫熷害鐜痥i
//		set_speed_kf(hub,1000);          //閫熷害鐜痥f
//		set_speed_smooth(hub,50);        //閫熷害骞虫粦绯绘暟
//		set_smooth_kf(hub,100);          //鍓嶉�堝钩婊戠郴鏁�
//		
//		set_tor_smooth(hub,100);         //杞�鐭╁钩婊戠郴鏁�
//		set_tor_kp(hub,220);             //鐢垫祦鐜疜p
//		set_tor_ki(hub,180);              //鐢垫祦鐜痥i
//		
//		set_acctime(hub,30);             //鍔犻�熸椂闂�
//		set_dectime(hub,30);             //鍑忛�熸椂闂�
//		set_stoptime(hub,100);           //鎬ュ仠鏃堕棿
//		set_IO_StopMode(hub,IO_UNLOCK);  //澶栭儴IO鎬ュ仠鏃朵负瑙ｈ酱妯″紡
//		EEPROM_Save(hub);                //淇濆瓨鍙傛暟鍒癊EPROM

		set_encoder_accuracy(hub,4096);  //缂栫爜鍣ㄧ嚎鏁�
		set_HallOffset(hub,0);           //HALL鍋忕Щ瑙掑害
		set_poles(hub,15);               //鐢垫満鏋佸�规暟
		set_rated_current(hub,150);      //棰濆畾鐢垫祦
		set_max_current(hub,300);        //鏈�澶х數娴�
		set_max_rpm(hub,1000);           //鏈�澶ц浆閫�
		set_overload_factor(hub,300);    //杩囪浇绯绘暟
		set_overload_time(hub,6553);      //杩囪浇鏃堕棿
		set_encoder_ErrorAlarm(hub,409); //缂栫爜鍣ㄨ秴宸�闃堝��
		set_maxTemp(hub,1200);            //娓╁害淇濇姢闃堝��
		set_speed_kp(hub,420);           //閫熷害鐜痥p
		set_speed_ki(hub,200);           //閫熷害鐜痥i
		set_speed_kf(hub,800);          //閫熷害鐜痥f
		set_speed_smooth(hub,50);        //閫熷害骞虫粦绯绘暟
		set_smooth_kf(hub,100);          //鍓嶉�堝钩婊戠郴鏁�
		
		set_tor_smooth(hub,100);         //杞�鐭╁钩婊戠郴鏁�
		set_tor_kp(hub,220);             //鐢垫祦鐜疜p
		set_tor_ki(hub,180);              //鐢垫祦鐜痥i
		
		set_acctime(hub,30);             //鍔犻�熸椂闂�
		set_dectime(hub,30);             //鍑忛�熸椂闂�
		set_stoptime(hub,100);           //鎬ュ仠鏃堕棿
		set_IO_StopMode(hub,IO_UNLOCK);  //澶栭儴IO鎬ュ仠鏃朵负瑙ｈ酱妯″紡
		EEPROM_Save(hub);                //淇濆瓨鍙傛暟鍒癊EPROM
	}

	
	#else //閫氫俊娴嬭瘯浣跨敤,鏃犳剰涔夋暟鎹�
	set_encoder_accuracy(hub,9);  //缂栫爜鍣ㄧ嚎鏁�
	set_HallOffset(hub,-10);           //HALL鍋忕Щ瑙掑害
	set_poles(hub,11);               //鐢垫満鏋佸�规暟
	set_rated_current(hub,12);      //棰濆畾鐢垫祦 
	set_max_current(hub,13);        //鏈�澶х數娴� 
	set_max_rpm(hub,14);           //鏈�澶ц浆閫�
	set_overload_factor(hub,15);    //杩囪浇绯绘暟 
	set_overload_time(hub,16);      //杩囪浇鏃堕棿
	set_encoder_ErrorAlarm(hub,17); //缂栫爜鍣ㄨ秴宸�闃堝��
	set_maxTemp(hub,18);            //娓╁害淇濇姢闃堝��
	set_pos_kp(hub,19);              //浣嶇疆鐜痥p
	set_pos_kf(hub,20);             //浣嶇疆鐜痥f
	set_speed_kp(hub,21);           //閫熷害鐜痥p
	set_speed_ki(hub,22);           //閫熷害鐜痥i
	set_speed_kf(hub,23);          //閫熷害鐜痥f
	set_speed_smooth(hub,24);        //閫熷害骞虫粦绯绘暟
	set_smooth_kf(hub,25);          //鍓嶉�堝钩婊戠郴鏁�
	set_tor_smooth(hub,26);         //杞�鐭╁钩婊戠郴鏁� -->
	set_tor_kp(hub,27);            //鐢垫祦鐜痥p
	set_tor_ki(hub,28);             //鐢垫祦鐜痥i
	set_acctime(hub,29);             //鍔犻�熸椂闂�
	set_dectime(hub,30);             //鍑忛�熸椂闂�
	set_stoptime(hub,31);           //鎬ュ仠鏃堕棿
	set_PosControl_MaxSpeed(hub,32); //浣嶇疆妯″紡涓嬫渶澶ц浆閫�
	set_Tor_Slope(hub,33); //杞�鐭╂枩鐜�
	set_IO_StopMode(hub,IO_UNLOCK);  //澶栭儴IO鎬ュ仠鏃朵负瑙ｈ酱妯″紡
	EEPROM_Save(hub);                //淇濆瓨鍙傛暟鍒癊EPROM
	#endif
	
	/* 鏄犲皠閰嶇疆 */
	if(hub==COBID_Send_1)
	{
		start_encoder_tpdo(hub,0x185);
		start_motor_state_tpdo(hub,0x186);
		start_selfcheck_tpdo(hub,0x187);
	}
	else if(hub==COBID_Send_2)
	{
		start_encoder_tpdo(hub,0x285);
		start_motor_state_tpdo(hub,0x286);
		start_selfcheck_tpdo(hub,0x287);
	}
	
	/* 鐢垫満鎺у埗妯″紡閰嶇疆 */
	set_SynCon_state(hub,1);         //璁剧疆鐢垫満鎺у埗鏂瑰紡涓哄悓姝ユ帶鍒�
	set_ControlMode(hub,VEL_CONTROL);//璁剧疆鐢垫満鍦ㄩ�熷害妯″紡涓嬭繍鍔�
	EEPROM_Save(hub);                //淇濆瓨鍙傛暟鍒癊EPROM
	
	/* 鐢垫満浣胯兘鎺у埗 */
	set_MotorRPM(hub,0,0);           //璁剧疆鐢垫満杞�閫熶负0
	CAN_Enable(hub);                 //浣胯兘鐢垫満
}
