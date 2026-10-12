#ifndef __Define__
#define __Define__

////////////////////////////////////////////////////////////////////
//                                                               //
// 修改了CANID后接收数据不用动，需要在发送函数处根据ID重新排序 //
//                                                               //
////////////////////////////////////////////////////////////////////
/*===| 底盘电机通信相关定义 |===*/
    #define Chassis_Motor_CAN                           hfdcan1  //底盘CAN
    #define Chassis_Motor_Send_CAN_ID                   0x200    //底盘电机CAN发送ID
    #define Chassis_Motor1_Feedback_CAN_ID              0x201    //底盘电机1CAN反馈ID
    #define Chassis_Motor2_Feedback_CAN_ID              0x202    //底盘电机2CAN反馈ID
    #define Chassis_Motor3_Feedback_CAN_ID              0x203    //底盘电机3CAN反馈ID
    #define Chassis_Motor4_Feedback_CAN_ID              0x204    //底盘电机4CAN反馈ID
            
/*===| 云台电机通信相关定义 |===*/
    #define Gimbal_Pitch1_CAN                            hfdcan2  //Pitch_使用的CAN
    #define Gimbal_Pitch1_Send_CAN_ID                    0x05  //Pitch_CAN发送ID
    #define Gimbal_Pitch1_Feedback_CAN_ID                0x105    //Pitch_CAN反馈ID
		
    #define Gimbal_Pitch2_CAN                            hfdcan2  //Pitch_使用的CAN
    #define Gimbal_Pitch2_Send_CAN_ID                    0x08  //Pitch_CAN发送ID
    #define Gimbal_Pitch2_Feedback_CAN_ID                0x108    //Pitch_CAN反馈ID
            
    #define Gimbal_Yaw_CAN                               hfdcan2  //Yaw_使用的CAN
    #define Gimbal_Yaw_Send_CAN_ID                       0x07    //Yaw_CAN发送ID
    #define Gimbal_Yaw_Feedback_CAN_ID                   0x107    //Yaw_CAN反馈ID
		
	#define Gimbal_Large_Yaw_CAN                         hfdcan2  //Yaw_使用的CAN
    #define Gimbal_Large_Yaw_Send_CAN_ID                 0x06  //Yaw_CAN发送ID
    #define Gimbal_Large_Yaw_Feedback_CAN_ID             0x106    //Yaw_CAN反馈ID

/*===| 发射电机通信相关定义 |===*/
    #define Shoot_Fric_VESC_CAN                            hfdcan3
    #define Shoot_Fric_VESC1_VESCID                             104
    #define Shoot_Fric_VESC1_Status1_Feedback_CAN_ID        (0x900+Shoot_Fric_VESC1_VESCID)
    #define Shoot_Fric_VESC2_VESCID                             102
    #define Shoot_Fric_VESC2_Status1_Feedback_CAN_ID        (0x900+Shoot_Fric_VESC2_VESCID)

    #define Shoot_Trigger_CAN                           hfdcan3  //Trigger_使用的CAN
    #define Shoot_Trigger_Send_CAN_ID                   0x144    //Trigger_CAN发送ID
    #define Shoot_Trigger_Feedback_CAN_ID               0x144    //Trigger_CAN反馈ID
/*===| 超电通信相关定义 |==*/       
    #define SuperCap_CAN                                hfdcan1		//hcan1
    #define SuperCap_Control_CAN_ID                     0x030    //超电控制CANID
    #define SuperCap_FeedBack_CAN_ID                    0x031    //超电反馈CANID
            
/*===| 陀螺仪通信相关定义 |==*/
    #define IMU_CAN                                     hfdcan3   //hcan3
    #define IMU_Control_CAN_ID                          0x001     //IMU控制CANID
    #define IMU_FeedBack_CAN_ID                         0x002     //IMU反馈CANID
/*===| 遥控CAN转发通信相关定义 |==*/  
    #define Remote_CAN                                  hfdcan3   
    #define Remote_CAN_Joystic_CAN_ID                   0x21   
    #define Remote_CAN_KeyboardMouse_CAN_ID             0x22    
    
    #define Aim_CAN_Target_Data_CAN_ID                  0x25
    #define Aim_CAN_Robo_Data_CAN_ID                    0x26

/*===| 数学运算定义 |===*/
    #define Rad2Angle                                (180.0f/PI)
    #define Angle2Rad                                (PI/180.0f)

/*===| 电机定义 |===*/
    #define Motor_3508_Reduction                        19.2032f
    #define Motor_2006_Reduction                        36.0f                               
    #define Motor_S3508_Reduction                       15.7647f
	#define Motor_MG4005_Reduction                      10.0f
        
/*===| 底盘相关参数定义 |===*/      
    #define Chassis_R_Distance                  0.196f     //轮子到底盘中心距离 根号(a2+b2)
    #define Chassis_Wheel_Radius                (0.166f/2.0f)   //轮半径
    #define Chassis_RPM_to_m_s                  (2*PI*Chassis_Wheel_Radius/60.0f/Motor_S3508_Reduction)  //底盘RPM转换成m/s的系数
    #define Robo_Vx_MaxSpeed                    3.0f     //机器最大X速度
    #define Robo_Vy_MaxSpeed                    3.0f     //机器最大Y速度
    #define Robo_Wz_MaxSpeed                    10.0f     //机器最大Z角速度
                
    #define Chassis_Acc                         12.0f     //底盘加速度
    #define Chassis_Dec                         20.0f     //底盘加速度 
        
/*===| 云台相关参数定义 |===*/  
    #define Gimbal_Median_Small_Yaw                     76.0952225          //云台朝前时Yaw电机的编码值
    #define Gimbal_Median_Large_Yaw                     113.596241f          //云台朝前时Yaw电机的编码值
    #define Gimbal_Median_Pitch                         -140.0f            //云台水平时Pitch电机编码值
    #define Gimbal_Yaw_Up_Limit                            0.0f
	#define Gimbal_Yaw_Down_Limit                          0.0f

    #define Gimbal_Pitch_Limit_Up              30.0f
    #define Gimbal_Pitch_Limit_Down             -10.0f
	#define Gimbal_Pitch_Offset                 (-5.0f+(-30.0f))/2.0f
    #define Gimbal_Pitch_Range	                (-5.0f-(-30.0f))/2.0f                                 
    #define Gimbal_Pitch_Search_T                0.5f             //s 
   

    #define Gimbal_Yaw_Search_Speed                     60.0f           //deg/s
    #define Gimbal_Large_Yaw_Search_Speed		        -80.0f           //deg/s		
		

/*===| 发射相关参数定义 |===*/
    #define Fric_Acc                                    0.1f            //摩擦轮加速度
    #define Fric_Radius                                 0.03f           //摩擦轮半径
    #define Fric_RPM_to_m_s                             (2*PI*Fric_Radius/60.0f)    //摩擦轮转速(RPM)转线速度(m/s)
    #define Trigger_Reduction                           (Motor_MG4005_Reduction/0.8f)  //拨弹轮减速比
    #define Trigger_Bullet_Num                          9.0f    //拨弹盘弹位数量

 
#endif
