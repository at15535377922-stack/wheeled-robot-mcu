#include "read_distance.h"

float distance_rgb = 0.3f;//灯带响应的距离

u8 rgb_100Hz_control=0;//灯带频率控制

u8 Trigger_Group_1=0,Trigger_Group_2=0,Trigger_Group_3=0; //超声波触发分组
//注：中断里会清零，任务里累加，必须加volatile
volatile u8 timeout_A,timeout_B,timeout_C,timeout_D,timeout_E,timeout_F;//超声波触发超时时间

u8 rgb_lights_showmode=0;//灯带模式 单色/彩虹模式

u8 rgb_control_filter=0;//灯带控制滤波，用于超声波

u8 cur_light_filter=0;

u8 rainbow_rgb_reset=2;//灯带过度显示

u8 charge_full = 0; //充电情况指示灯

void my_delay_us(u32 us);

void ReadUS_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
		// This task runs at a frequency of 200Hz (5ms control once)
		//此任务以200Hz的频率运行（5ms控制一次）
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_200_HZ));
			
		rgb_100Hz_control++; //RGB频率控制走时

		//超声波超时时间监测
		//超时时间意义：超声波分组触发，当1组超声波未完成捕获时，另一组不会启动触发；故需要设置超时时间，防止单个超声波不工作时或被拔掉时影响其他超声波
		if(END_A==1) timeout_A++;
		if(END_B==1) timeout_B++;
		if(END_C==1) timeout_C++;
		if(END_D==1) timeout_D++;
		if(END_E==1) timeout_E++;
		if(END_F==1) timeout_F++;
		
		//S300机器人
		//注：S100原厂无超声波，此处为用户自行加装的6路，沿用S300的ACE/BDF分组触发。
		//    通道对应实际安装位置：D前左 E前右 C左侧 F右侧 B后左 A后右
		if(Car_Mode == S300 || Car_Mode == S200 || Car_Mode == S100)
		{
			//超声波ACE为1组一起触发
			if(!END_B&&!END_D&&!END_F&&!Trigger_Group_1)
			{
				Trigger_Group_1 = 1; //组1标记1次，在未完成组1的所有超声波任务前不会再进入此函数
				Trigger_Group_2 = 0; //完成了组2的任务，组2标记复位
				END_A = 1 , TriggerA = 1, delay_us(12) , TriggerA = 0 , timeout_A = 0;
				END_C = 1 , TriggerC = 1, delay_us(12) , TriggerC = 0 , timeout_C = 0;
				END_E = 1 , TriggerE = 1, delay_us(12) , TriggerE = 0 , timeout_E = 0;
			}

			//超声波BDF为1组一起触发
			if(!END_A&&!END_C&&!END_E&&!Trigger_Group_2)
			{
				Trigger_Group_2 = 1; //组2标记1次，未完成组2的所有超声波任务前不会再进入此函数
				Trigger_Group_1 = 0; //完成了组1的任务，组1标记复位		
				END_B = 1 , TriggerB = 1, delay_us(12) , TriggerB = 0 , timeout_B = 0;
				END_D = 1 , TriggerD = 1, delay_us(12) , TriggerD = 0 , timeout_D = 0;
				END_F = 1 , TriggerF = 1, delay_us(12) , TriggerF = 0 , timeout_F = 0;
			}
			
			//超时监测
			if(Trigger_Group_1==1&&(timeout_A>=7||timeout_C>=7||timeout_E>=7))  //允许超时时间：7*5=35ms
			{
				//哪一路超时就需要对哪一路进行初始化操作
				if(END_A)
				{
					US_A_FLag=0;
					timeout_A=0;
					Set_US_A_Rising;
					END_A = 0;
					us_filter_reset_A = 1; //丢弃超时前的旧数据，恢复后从新值重建缓冲区
					ultrasonic.A = 3;
				}
				if(END_C)
				{
					US_C_FLag=0;
					timeout_C=0;
					Set_US_C_Rising;
					END_C = 0;
					us_filter_reset_C = 1; //丢弃超时前的旧数据，恢复后从新值重建缓冲区
					ultrasonic.C = 3;
				}
				if(END_E)
				{
					US_E_FLag=0;
					timeout_E=0;
					Set_US_E_Rising;
					END_E = 0;
					us_filter_reset_E = 1; //丢弃超时前的旧数据，恢复后从新值重建缓冲区
					ultrasonic.E = 3 ;
				}
			}
			
			if(Trigger_Group_2==1&&(timeout_B>=7||timeout_D>=7||timeout_F>=7))//允许超时时间：7*5=35ms
			{
				//哪一路超时就需要对哪一路进行初始化操作
				if(END_B)
				{
					US_B_FLag=0;
					timeout_B=0;
					Set_US_B_Rising;
					END_B = 0;
					us_filter_reset_B = 1; //丢弃超时前的旧数据，恢复后从新值重建缓冲区
					ultrasonic.B = 3;
				}
				if(END_D)
				{
					US_D_FLag=0;
					timeout_D=0;
					Set_US_D_Rising;
					END_D = 0;
					us_filter_reset_D = 1; //丢弃超时前的旧数据，恢复后从新值重建缓冲区
					ultrasonic.D = 3;
				}
				if(END_F)
				{
					US_F_FLag=0;
					timeout_F=0;
					Set_US_F_Rising;
					END_F = 0;
					us_filter_reset_F = 1; //丢弃超时前的旧数据，恢复后从新值重建缓冲区
					ultrasonic.F = 3;
				}
			}
			
		}
		
		//S150机器人
		else if (Car_Mode == S150 )
		{
			ultrasonic.F = 5.2f; //S150没有F路超声波，设置固定值5.2m
			
			if(!END_B&&!END_D&&!Trigger_Group_1)
			{
				//ACE为1组
				Trigger_Group_1=1;
				Trigger_Group_2=0; //允许第2组进入等待状态
				Trigger_Group_3=1;
				END_A = 1 , TriggerA = 1, delay_us(12) , TriggerA = 0 , timeout_A = 0;
				END_C = 1 , TriggerC = 1, delay_us(12) , TriggerC = 0 , timeout_C = 0;
				END_E = 1 , TriggerE = 1, delay_us(12) , TriggerE = 0 , timeout_E = 0;
			}

			if(!END_A&&!END_C&&!END_E&&!Trigger_Group_2)
			{
				Trigger_Group_1=1;
				Trigger_Group_2=1;
				Trigger_Group_3=0;//允许第3组进入等待状态
				//B单独1组
				END_B = 1 , TriggerB = 1, delay_us(12) , TriggerB = 0 , timeout_B = 0;
			}

			if(!END_B&&!Trigger_Group_3)
			{
				Trigger_Group_1=0;//允许第1组进入等待状态
				Trigger_Group_2=1;
				Trigger_Group_3=1;
				//D单独1组
				END_D = 1 , TriggerD = 1, delay_us(12) , TriggerD = 0 , timeout_D = 0;	
			}
			
			//超时监测
			if((Trigger_Group_1&&!Trigger_Group_2&&Trigger_Group_3)&&(timeout_A>=7||timeout_C>=7||timeout_E>=7))  //允许超时时间：7*5=35ms
			{
				//哪一路超时就需要对哪一路进行初始化操作
				if(END_A)
				{
					US_A_FLag=0;
					timeout_A=0;
					Set_US_A_Rising;
					END_A = 0;
					us_filter_reset_A = 1; //丢弃超时前的旧数据，恢复后从新值重建缓冲区
					ultrasonic.A = 3;
				}
				if(END_C)
				{
					US_C_FLag=0;
					timeout_C=0;
					Set_US_C_Rising;
					END_C = 0;
					us_filter_reset_C = 1; //丢弃超时前的旧数据，恢复后从新值重建缓冲区
					ultrasonic.C = 3;
				}
				if(END_E)
				{
					US_E_FLag=0;
					timeout_E=0;
					Set_US_E_Rising;
					END_E = 0;
					us_filter_reset_E = 1; //丢弃超时前的旧数据，恢复后从新值重建缓冲区
					ultrasonic.E = 3;
				}
			}
			if((Trigger_Group_1&&Trigger_Group_2&&!Trigger_Group_3)&&timeout_B>=7)
			{
				if(END_B)
				{
					US_B_FLag=0;
					timeout_B=0;
					Set_US_B_Rising;
					END_B = 0;
					us_filter_reset_B = 1; //丢弃超时前的旧数据，恢复后从新值重建缓冲区
					ultrasonic.B = 3;
				}
			}
			if((!Trigger_Group_1&&Trigger_Group_2&&Trigger_Group_3)&&timeout_D>=7)
			{
				if(END_D)
				{
					US_D_FLag=0;
					timeout_D=0;
					Set_US_D_Rising;
					END_D = 0;
					us_filter_reset_D = 1; //丢弃超时前的旧数据，恢复后从新值重建缓冲区
					ultrasonic.D = 3;
				}
			}
	
		}

		#if USE_RGB_lights
		//灯带控制任务
		/* 优先级高到低，高优先级可以打断低优先级：
		1、自动回充充电中状态
		2、自动回充充满电状态
		3、超声波遇到障碍物显示状态
		4、自动回充模式
		4、电池低电量状态
		5、自然状态
		*/	
		//注：这里自增后立刻判1并清零，实际每拍都进，即200Hz而非变量名所指的100Hz。
		//    灯效渐变速度是按当前这个实际频率调出来的，若要真正的100Hz需改为 >=2 并同步放慢渐变步长。
		if(rgb_100Hz_control==1)
		{
			rgb_100Hz_control = 0;
					
			if(rgb_set[0]!=User_defined )//用户自定义灯带优先级最高
			{
				if(Allow_Recharge)//自动回充模式
				{
					rgb_set[0]=Auto_recharge;
					
					if(Charging||ChargDelay)//充电状态颜色
					{				
						if(charge_full==0)
						{
							/* 红色 */
							if(rgb_set[1]<255) rgb_set[1]++;
							if(rgb_set[2]!=0) rgb_set[2]--;
							if(rgb_set[3]!=0) rgb_set[3]--;
							rainbow_rgb_reset=1;
						}
						else
						{
							/* 绿色 */
							if(rgb_set[1]<120) rgb_set[1]++;
							if(rgb_set[1]>120) rgb_set[1]--;
							if(rgb_set[2]<255) rgb_set[2]++;
							if(rgb_set[3]!=0) rgb_set[3]--;
							rainbow_rgb_reset=2;
						}
						
						//充电电流减小时，RGB灯带切换为绿色
						if( Charging_Current<500 && Voltage>25.0f)
						{
							cur_light_filter++;
							if(cur_light_filter>50) cur_light_filter = 0,charge_full = 1;
						}
						else if(Charging_Current>500) charge_full = 0;

					}
					else
					{
						charge_full = 0;
						/* 自动回充寻到充电桩时颜色 - 蓝色 */
						if(rgb_set[1]!=0) rgb_set[1]--;
						if(rgb_set[2]!=0) rgb_set[2]--;
						if(rgb_set[3]<255) rgb_set[3]++;
						rainbow_rgb_reset=4;
					}
				}
				else//自然状态
				{									
					//超声波距离过近
					if(ultrasonic.A<=distance_rgb||ultrasonic.B<=distance_rgb||ultrasonic.C<=distance_rgb||\
					   ultrasonic.D<=distance_rgb||ultrasonic.E<=distance_rgb||ultrasonic.F<=distance_rgb)
					{
						if(rgb_set[1]<253) rgb_set[1]+=2;
						if(rgb_set[2]<253) rgb_set[2]+=2;
						if(rgb_set[3]>=2)  rgb_set[3]-=2;			
						rainbow_rgb_reset = 3;				
					}
					else
					{
						if(Low_PowerMode)//低电量
						{
							//低电量提示
							rgb_set[0]=LowPower_mode;
							static u16 tmpcount;
							tmpcount++;
							if( tmpcount<200 ) rgb_set[1]=200,rgb_set[2]=0,rgb_set[3]=255;
							else if( tmpcount>200&&tmpcount<400 )rgb_set[1]=0,rgb_set[2]=0,rgb_set[3]=0;	
							else if( tmpcount>400 ) tmpcount=0;
//							/* 低电量紫色 */
//							if(rgb_set[1]<200) rgb_set[1]++;
//							if(rgb_set[1]>200) rgb_set[1]--;
//							if(rgb_set[2]!=0)  rgb_set[2]--;
//							if(rgb_set[3]<255) rgb_set[3]++;
							rainbow_rgb_reset = 5;
						}
						else//电量正常，超声波距离正常
						{
							rgb_set[0]=Natural_mode;
							
							//渐变关闭灯带
							if( rgb_set[1]!=0 ) rgb_set[1]--;
							if( rgb_set[2]!=0 ) rgb_set[2]--;
							if( rgb_set[3]!=0 ) rgb_set[3]--;
	
							//关闭普通状态的灯带,降低功耗
							//if(rgb_lights_showmode) Rainbow_RGB();
							//else  Monochrome_RGB(); //单色RGB
						}						
					}
				}
			}
			else
			{
				rgb_set[1]=rgb_r,rgb_set[2]=rgb_g,rgb_set[3]=rgb_b;/* 赋值用户自定义的颜色 */
			}
			
			//灯光提示安全模式状态
			static u8 tip_off_count=0,tip_off_times=0;
			static u8 tip_on_count=0,tip_on_times=4;
			if( SecurityPLY==1 )
			{
				tip_on_count = 0;
				tip_on_times = 0;
				if( tip_off_times<=3 )
				{
					tip_off_count++;
						  if( tip_off_count<20 )               RGB_Set(0,0,0);
					else if( tip_off_count>20&&tip_off_count<40 )  RGB_Set(255,0,0);
					else if( tip_off_count>=40 ) tip_off_times++,tip_off_count=0;
					continue;
				}

			}
			else
			{
				tip_off_count = 0;
				tip_off_times = 0;
				if( tip_on_times<=3 )
				{
					tip_on_count++;
						  if( tip_on_count<20 )                   RGB_Set(0,0,0);
					else if( tip_on_count>20&&tip_on_count<40 )  RGB_Set(0,255,0);
					else if( tip_on_count>=40 ) tip_on_times++,tip_on_count=0;
					continue;
				}
			}
			
			RGB_Set(rgb_set[1],rgb_set[2],rgb_set[3]);		
		}
		#endif
	}
}

void my_delay_us(u32 us)
{
	u32 delay = 21*us;
	while(delay--);
}

#if 1

//彩虹RGB呼吸灯
void Rainbow_RGB(void)
{
	static u8 r,g,b;
	static u8 now_mode=0;
	static u8 done_r,done_g,done_b;

	//从充电状态切换成正常状态的过度
	if(rainbow_rgb_reset==1)//续红色
	{
		r=255,g=0,b=0;
		done_r=0,done_g=0,done_b=0;
		now_mode=1;
		rainbow_rgb_reset=0;
	}
	else if(rainbow_rgb_reset==2)//续绿色
	{
		r=120,g=255,b=0;
		done_r=0,done_g=0,done_b=0;
		now_mode=2;
		rainbow_rgb_reset=0;
	}
	else if(rainbow_rgb_reset==3)//续黄色
	{
		r=255,g=255,b=0;
		done_r=0,done_g=0,done_b=0;
		now_mode=2;
		rainbow_rgb_reset=0;
	}
	else if(rainbow_rgb_reset==3)//续黄色
	{
		r=255,g=255,b=0;
		done_r=0,done_g=0,done_b=0;
		now_mode=2;
		rainbow_rgb_reset=0;
	}
	else if(rainbow_rgb_reset==4)//续蓝色
	{
		r=0,g=0,b=255;
		done_r=0,done_g=0,done_b=0;
		now_mode=5;
		rainbow_rgb_reset=0;
	}
	else if(rainbow_rgb_reset==5)
		rainbow_rgb_reset=0;
	
	//正常彩虹rgb
	if(now_mode==0)
	{
		if(++r==255) now_mode=1; //第一次r通道加满
	}
	
	if(now_mode==1)
	{
		if(++g==255) now_mode=2;//第二次g通道加满
	}
	
	if(now_mode==2)
	{
		if(--r==0) now_mode=3;//第三次r通道减少到0
	}
	
	if(now_mode==3) //g-  b+  最终（0,127,255）
	{
		if(--g==127) g=128,done_g=1;
		
		b+=1;
		if(b>=254) b=255,done_b=1;
		
		if(done_g&&done_b)
		{
			g=127;
			b=255;
			done_g=0;
			done_b=0;
			now_mode=4;
		}
	}
	
	if(now_mode==4)
	{
		if(--g==0) now_mode=5;
	}
	
	if(now_mode==5)
	{
		if(++r==138) now_mode=6;
	}
	
	if(now_mode==6)
	{
		done_r=1;
		b-=1;
		if(b<=1) b=0,done_b=1;
		
		if(done_r&&done_b)
		{
			r=138;
			b=0;
			done_r=0;
			done_b=0;
			now_mode=0;
		}
	}
	
	rgb_set[1] = r;
	rgb_set[2] = g;
	rgb_set[3] = b;
}

//单色RGB呼吸灯
void Monochrome_RGB(void)
{
	static u8 RGB=1,mode=1;
	static u8 tmp_r,tmp_b,tmp_g;
	static u8 pass = 0;
	
	//RGB过度
	if(rainbow_rgb_reset!=0)
	{
		pass = 0;
		
		if(rgb_set[1]!=0) rgb_set[1]--;
		if(rgb_set[2]!=0) rgb_set[2]--;
		if(rgb_set[3]!=0) rgb_set[3]--;
		
		if(rgb_set[1]==0&&rgb_set[2]==0&&rgb_set[3]==0)
		{
			pass = 1;
			
			if(rainbow_rgb_reset==1)//红色结束，绿色开始
			{
				RGB = 2 ;
			}
			else if(rainbow_rgb_reset==2)//绿色结束，蓝色开始
			{
				RGB = 3 ;
			}
			else if(rainbow_rgb_reset==4) //蓝色结束，红色开始
			{
				RGB = 1 ;
			}
			else //其他情况
			{
				RGB += 1;
				if(RGB>3) RGB = 1;
			}
			mode = 1, tmp_r = 0 , tmp_g = 0, tmp_b = 0;
			rainbow_rgb_reset = 0;
		}
	}
	
	if(pass==1)
	{
		if(RGB==1)
		{
			if(mode)
			{
				if(++tmp_r>254) mode=0;					
			}
			else
			{
				if(--tmp_r<1) mode=1,RGB=2;
			}	
		}
		else if (RGB==2)
		{
			if(mode)
			{
				if(++tmp_g>254) mode=0;						
			}
			else
			{
				if(--tmp_g<1) mode=1,RGB=3;
			}	
		}
		else if (RGB==3)
		{
			if(mode)
			{
				if(++tmp_b>254) mode=0;
			}
			else
			{
				if(--tmp_b<1) mode=1,RGB=1;
			}
		}
		
		rgb_set[1] = tmp_r;
		rgb_set[2] = tmp_g;
		rgb_set[3] = tmp_b;
	}

}

#endif
