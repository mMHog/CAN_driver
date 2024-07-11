#include "controlcan.h"
#include <iostream>
#include <string>
#include <unistd.h>

#define DRIVER_NUM 7
#define CAN_NUM 3


//CAN调试的开关
int CAN_DEBUG=0;
//每个电机对应分析仪的序号
int DRIVER_ASSIGN[DRIVER_NUM]={1,1,2,0,0,0,2};
//设定值，是否使能对应的电机
int DRIVEN_ENABLE[DRIVER_NUM]={1,1,1,1,1,1,0};
//对应电机的使能状态
int DRIVEN_ENABLE_STATUS[DRIVER_NUM]={0,0,0,0,0,0,0};

//电机位置的当前值，调用QUERY_POSITION后修改
int POSITION_CURRENT[DRIVER_NUM]={0,0,0,0,0,0,0};
//当前电机的目标值，手动修改，调用DRIVER_POSITION_ALL时使用
int POSITION_COMMAND[DRIVER_NUM]={0,0,0,0,0,0,0};

typedef struct _MOTION_PROFILE_{
    int velocity=200;
    int max_velocity=1000;
    int acceleration=100;
    int deceleration=100;
}MOTION_PROFILE_;
//电机的速度，最大速度，加减速度的配置
MOTION_PROFILE_ MOTION_PROFIEL[DRIVER_NUM];

//输出CAN报文的信息
int CAN_OBJ_INFO(VCI_CAN_OBJ obj){
        std::cout << "\tID: " << std::hex << obj.ID;  
        std::cout << "\tDataLen: " << (unsigned int)(obj.DataLen); 
        int* FeedBack=(int*)(obj.Data+4);
        std::cout << "\tDecode:\t" <<  std::dec << *FeedBack;
        std::cout << " " <<  std::hex << (unsigned  int)(obj.Data[0]); 
        std::cout << " " << (unsigned int)(obj.Data[1]); 
        std::cout << " " << (unsigned int)(obj.Data[2]); 
        std::cout << " " << (unsigned int)(obj.Data[3]);
        std::cout << " " << (unsigned int)(obj.Data[4]);
        std::cout << " " << (unsigned int)(obj.Data[5]);
        std::cout << " " << (unsigned int)(obj.Data[6]);
        std::cout << " " << (unsigned int)(obj.Data[7])  <<  std::dec << std::endl;
}

//用于发送CAN报文，ID，数据长度，数据内容，报文后四个字节的解析，报文的描述
int CAN_SEND(int DataID,unsigned char DataLen,unsigned char* Data,int* FeedBack,std::string desc){
    VCI_CAN_OBJ send[1];
    VCI_CAN_OBJ rec[10];
    int reclen,j;
    send[0].ID =DataID;
    send[0].SendType = 0;
    send[0].RemoteFlag = 0; // 0数据帧 1远程帧
    send[0].ExternFlag = 0; // 0标准帧（11位） 扩展帧（29位）
    send[0].DataLen = DataLen;
    for(j=0;j<DataLen;j++){
        send[0].Data[j] = Data[j];
    }
    int DEVICE_ID=DRIVER_ASSIGN[DataID-0x601];
    VCI_ClearBuffer(VCI_USBCAN2,DEVICE_ID,0);
    if(VCI_Transmit(VCI_USBCAN2, DEVICE_ID, 0, send, 1) == 1) {
        if(CAN_DEBUG){
            std::cout<<"SEND("<<DEVICE_ID<<"):"<<desc;
            CAN_OBJ_INFO(send[0]);
        }
        reclen = VCI_Receive(VCI_USBCAN2, DEVICE_ID, 0, rec, 10, 10000);
        if(CAN_DEBUG)std::cout<<"RECEIVE:"<<reclen;
        if(reclen > 0) {
            *FeedBack=*(int*)(rec[0].Data+4);
            for(j = 0; j < reclen; j++) {
                int* c=(int*)(send[0].Data);
                int* f=(int*)(rec[j].Data);
                if(*f==0x606443){     
                    POSITION_CURRENT[rec[j].ID-0x581]=*(int*)(rec[j].Data+4);
                    if(*c==0x606440)
                        *FeedBack=*(int*)(rec[j].Data+4);
                }
                if(CAN_DEBUG)CAN_OBJ_INFO(rec[j]);
            }
        }
        else{
            *FeedBack=0;
            if(CAN_DEBUG)std::cout<<std::endl;
        }
    }
}

int CAN_CLOSE(){    
    for(int i=0;i<CAN_NUM;i++){
        VCI_CloseDevice(VCI_USBCAN2, i);
    }
}

//初始化CAN板卡1
int CAN_INIT(){
    VCI_BOARD_INFO pInfo;//用来获取设备信息。
    int count = 0; //数据列表中，用来存储列表序号。
    VCI_BOARD_INFO pInfo1 [50];
    int num = 2;

    std::cout << ">>START CAN INIT" << std::endl; //指示程序已运行
    num = VCI_FindUsbDevice2(pInfo1);  //寻找USB接口

    std::cout << ">>USBCAN DEVICE NUM:" << num << std::endl;

    if(num!=CAN_NUM){
            std::cout << ">>CAN DEVICE NUMBER ERROR" << std::endl;
            exit(1);
    }

    for(int i=0;i<CAN_NUM;i++){    
        if (VCI_OpenDevice(VCI_USBCAN2, i, 0) == 1) { //打开设备
            // std::cout << ">>OPEN CAN "<<i<<" DEVICE SUCCESS" << std::endl; //打开设备成功
        } else {
            std::cout << ">>OPEN CAN "<<i<<" DEVICE FAILED" << std::endl;
            exit(1);
        }
    }
    printf(">>OPEN ALL CAN DEVICE\n");
    

    // if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo) == 1) { //读取设备序列号、版本等信息。
    //     std::cout << ">>GET VCIINFO SUCCESS!" << std::endl;
    // } else {
    //     std::cout << ">GET VCIINFO FAILED" << std::endl;
    //     exit(1);
    // }

    //初始化参数
    VCI_INIT_CONFIG config;  //结构体定义了初始化CAN的配置

    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;        //接收所有帧
    config.Timing0 = 0X00; /*波特率125 Kbps  0x03  0x1C*/
    config.Timing1 = 0x14;
    config.Mode = 0; //正常模式


    for(int i=0;i<CAN_NUM;i++){    
        if (VCI_InitCAN(VCI_USBCAN2, i, 0, &config) != 1) {
            std::cout << ">>INIT "<<i<<" ERROR\n" << std::endl;
            CAN_CLOSE();
        }

        if (VCI_StartCAN(VCI_USBCAN2, i, 0) != 1) {
            std::cout << ">>START "<<i<<" ERROR\n" << std::endl;
            CAN_CLOSE();
        }
    }
    printf(">>CAN ALL INIT SUCCEESS!\n");
}


//使能电机，根据DRIVEN_ENABLE
int DRIVER_ENABLE(){
    unsigned char send1[8]={0x2b,0x40,0x60,0x00,0x06,0x00,0x00,0x00};
    unsigned char send2[8]={0x2b,0x40,0x60,0x00,0x07,0x00,0x00,0x00};
    unsigned char send3[8]={0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00};
    unsigned char send4[8]={0x2f,0x46,0x35,0x00,0x00,0x00,0x00,0x00};
    unsigned char send5[8]={0x2f,0x60,0x60,0x00,0x01,0x00,0x00,0x00};
    int f;
    for(int i=0;i<DRIVER_NUM;i++){
        if(DRIVEN_ENABLE[i]==1&&DRIVEN_ENABLE_STATUS[i]==0){
            CAN_SEND(0x601+i,8,send1,&f,"READY TO SWITCHED ON: "+std::to_string(i));
            CAN_SEND(0x601+i,8,send2,&f,"SWITCHED ON: "+std::to_string(i));
            CAN_SEND(0x601+i,8,send3,&f,"OPERATION ENABLE: "+std::to_string(i));
            CAN_SEND(0x601+i,8,send4,&f,"DISABLE CYCLE MODE: "+std::to_string(i));
            CAN_SEND(0x601+i,8,send5,&f,"SET POSITION MODE: "+std::to_string(i));
            DRIVEN_ENABLE_STATUS[i]=1;
        }
    }
}
//根据序号禁能电机
int DRIVER_DISABLE(int i){
    int f;
    unsigned char send1[8]={0x2b,0x40,0x60,0x00,0x06,0x00,0x00,0x00};
    CAN_SEND(0x601+i,8,send1,&f,"READY TO SWITCHED ON: "+std::to_string(i));
    DRIVEN_ENABLE_STATUS[i]=0;
}
//禁能全部电机
int DRIVER_DISABLE_ALL(){
    for(int i=0;i<DRIVER_NUM;i++){
        if(DRIVEN_ENABLE[i]==1&&DRIVEN_ENABLE_STATUS[i]==1){
            DRIVER_DISABLE(i);
        }
    }
}
//根据序号查询当前电机位置，返回电机位置并保存在POSITION_CURRENT，因为丢包可能数据不准确，需要注意
int QUERY_POSITION(int i){
    int f;
    unsigned char send1[8]={0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00};
    if(DRIVEN_ENABLE[i]==1)
        CAN_SEND(0x601+i,8,send1,&f,"QUERY POSITION: "+std::to_string(i));
    //POSITION_CURRENT[i]=f;
    return f;
}
//查询全部电机位置并保存在POSITION_CURRENT，因为丢包可能数据不准确，需要注意
int QUERY_POSITION_ALL(){
    for(int i=0;i<DRIVER_NUM;i++){
        if(DRIVEN_ENABLE[i]==1){
            QUERY_POSITION(i);
        }
    }
}
//根据序号和数据使电机运动到指定位置，数据为编码器脉冲，指令保存在POSITION_COMMAND
int DRIVER_POSITION(int i,int command){
    unsigned char send1[8]={0x23,0x7a,0x60,0x00,0x00,0x00,0x00,0x00};
    unsigned char send2[8]={0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00};
    unsigned char send3[8]={0x2b,0x40,0x60,0x00,0x3f,0x00,0x00,0x00};
    int f;
    int *p;
    p=(int*)(send1+4);
    *p=command;
    POSITION_COMMAND[i]=command;
    CAN_SEND(0x601+i,8,send1,&f,"SET TARGET POSITION: "+std::to_string(i)+'('+std::to_string(command)+')');
    CAN_SEND(0x601+i,8,send1,&f,"SET TARGET POSITION: "+std::to_string(i)+'('+std::to_string(command)+')');
    CAN_SEND(0x601+i,8,send2,&f,"OPERATION ENABLE");
    CAN_SEND(0x601+i,8,send2,&f,"OPERATION ENABLE");
    CAN_SEND(0x601+i,8,send3,&f,"START MOTION: "+std::to_string(i)+'('+std::to_string(command)+')');
    CAN_SEND(0x601+i,8,send3,&f,"START MOTION: "+std::to_string(i)+'('+std::to_string(command)+')');
}
//根据POSITION_COMMAND发送全部电机位置指令
int DRIVER_POSITION_ALL(){
    for(int i=0;i<DRIVER_NUM;i++){
        if(DRIVEN_ENABLE[i]==1){
            DRIVER_POSITION(i,POSITION_COMMAND[i]);
        }
    }
}
//暂停运动，未测试过
int DRIVER_POSITION_PAUSE(int i){
    int f;
    unsigned char send1[8]={0x2b,0x40,0x60,0x00,0x3f,0x01,0x00,0x00};
    CAN_SEND(0x601+i,8,send1,&f,"DRIVER POSITION PAUSE: "+std::to_string(i));
}
//根据序号检查电机运动是否完成，返回1表示完成，0表示未完成
int DRIVER_CHECK_POSITION(int i){
    int f;
    unsigned char send1[8]={0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00};
    CAN_SEND(0x601+i,8,send1,&f,"DRIVER CHECK POSITION: "+std::to_string(i));
    if(f&1<<10){
        return 1;
    }else{
        return 0;
    }
}
//检查全部电机运动是否完成，返回1表示完成，0表示未完成
int DRIVER_CHECK_POSITION_ALL(){
    for(int i=0;i<DRIVER_NUM;i++){
        if(DRIVEN_ENABLE[i]==1){
            if(!DRIVER_CHECK_POSITION(i))
            return 0;
        }
    }
    return 1;
}
//根据序号和MOTION_PROFIEL设置速度，最大速度，加减速度
int SET_MOTION_PROFILE(int i){
    unsigned char send1[8]={0x23,0x81,0x60,0x00,0x00,0x00,0x00,0x00};
    unsigned char send2[8]={0x23,0x7f,0x60,0x00,0x00,0x00,0x00,0x00};
    unsigned char send3[8]={0x23,0x83,0x60,0x00,0x00,0x00,0x00,0x00};
    unsigned char send4[8]={0x23,0x84,0x60,0x00,0x00,0x00,0x00,0x00};
    int f;
    int *p;
    p=(int*)(send1+4);
    *p=MOTION_PROFIEL[i].velocity;
    p=(int*)(send2+4);
    *p=MOTION_PROFIEL[i].max_velocity;
    p=(int*)(send3+4);
    *p=MOTION_PROFIEL[i].acceleration;
    p=(int*)(send4+4);
    *p=MOTION_PROFIEL[i].deceleration;
    CAN_SEND(0x601+i,8,send1,&f,"SET VELOCITY: "+std::to_string(i)+'('+std::to_string(MOTION_PROFIEL[i].velocity)+')');
    CAN_SEND(0x601+i,8,send2,&f,"SET MAX VELOCITY: "+std::to_string(i)+'('+std::to_string(MOTION_PROFIEL[i].max_velocity)+')');
    CAN_SEND(0x601+i,8,send3,&f,"SET ACCELERATION: "+std::to_string(i)+'('+std::to_string(MOTION_PROFIEL[i].acceleration)+')');
    CAN_SEND(0x601+i,8,send4,&f,"SET DECELERATION: "+std::to_string(i)+'('+std::to_string(MOTION_PROFIEL[i].deceleration)+')');
}
//根据MOTION_PROFIEL设置全部电机的速度，最大速度，加减速度
int SET_MOTION_PROFILE_ALL(){
    for(int i=0;i<DRIVER_NUM;i++){
        if(DRIVEN_ENABLE[i]==1){
            SET_MOTION_PROFILE(i);
        }
    }
}
//清除错误
int CLEAR_ERROR(){
    unsigned char send1[8]={0x2b,0x40,0x60,0x00,0x80,0x00,0x00,0x00};
    int f;
    for(int i=0;i<DRIVER_NUM;i++){
        if(DRIVEN_ENABLE[i]==1){
            CAN_SEND(0x601+i,8,send1,&f,"CLEAR ERROR");
        }
    }

}
//读取并输出全部电机的当前位置和指令，因为丢包可能数据不准确，需要注意
int MOTION_INFO(){
    QUERY_POSITION_ALL();
    std::cout<<"CURRENT:";
    for(int i=0;i<DRIVER_NUM;i++)
        std::cout<<POSITION_CURRENT[i]<<"\t";
    std::cout<<std::endl;
    std::cout<<"COMMAND:";
    for(int i=0;i<DRIVER_NUM;i++)
        std::cout<<POSITION_COMMAND[i]<<"\t";
    std::cout<<std::endl;
}

int main(int argc, char **argv) {
    int f;
    int i=30;
    //初始化
    CAN_INIT();//打开板卡
    CLEAR_ERROR();//清除错误
    DRIVER_ENABLE();//使能电机
    SET_MOTION_PROFILE_ALL();//设置运动参数

    //设置目标位置并等待
    POSITION_COMMAND[0]=-1000;
    POSITION_COMMAND[1]=-1000;
    POSITION_COMMAND[2]=-1000;
    POSITION_COMMAND[3]=-1000;
    POSITION_COMMAND[4]=-1000;
    POSITION_COMMAND[5]=-1000;
    POSITION_COMMAND[6]=-1000;
    DRIVER_POSITION_ALL();

    std::cout<<">>MOTION 1 START"<<std::endl;
    while(!DRIVER_CHECK_POSITION_ALL()){
        MOTION_INFO();
        usleep(1000*1000);
    }
    std::cout<<">>MOTION 1 FINISHED"<<std::endl;

    // MOTION_PROFIEL[0].velocity=200;
    // MOTION_PROFIEL[1].velocity=200;
    // SET_MOTION_PROFILE_ALL();

    std::cout<<">>MOTION 2 START"<<std::endl;
    POSITION_COMMAND[0]=1000;
    POSITION_COMMAND[1]=1000;
    POSITION_COMMAND[2]=1000;
    POSITION_COMMAND[3]=1000;
    POSITION_COMMAND[4]=1000;
    POSITION_COMMAND[5]=1000;
    POSITION_COMMAND[6]=1000;
    DRIVER_POSITION_ALL();
    while(!DRIVER_CHECK_POSITION_ALL()){
        MOTION_INFO();
        usleep(1000*1000);
    }
    std::cout<<">>MOTION 2 FINISHED"<<std::endl;


    while(1) {
        MOTION_INFO();
        usleep(1000*1000);
    }

    std::cout << ">>MAIN EXIT" << std::endl;
    DRIVER_DISABLE_ALL();
    CAN_CLOSE();
    std::cout << ">>CAN CLOSED" << std::endl;

    return 0;
}



