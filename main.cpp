#include "mbed.h"
#include "PS3BT.h"
#include <usbhub.h>
#include "hcsr04.h"
#include "EPOS_CAN.h"

#define DELTA_T1 0.1
#define target_val1 0
#define Kp1 3
#define Ki1 0
#define Kd1 0

#define RPM_RIDE 400
#define RPM_CLEAN 200
#define ACC_RIDE 1000
#define DEC_RIDE 700
#define ACC_CLEAN 1000
#define DEC_CLEAN 1000

#define KICK 2000
#define CLEAN_OFFSET 3

#define WALL 45
#define WALL_MIN 25
#define Standby_Time 30
#define GETOFF_TIME 1

#define MAX_RPM 1500
#define MAX_RPM_BUTTON 1000
#define BUTTON_DEADBAND 0
#define STICK_DEADBAND 50

#define ACC 3000
#define DEC 3000

Timer t;
Timer cr_t;
double Time = 0;
double cr_time = 0;
int crflag=0;

char Serialdata;

int encX = 0;
int encY = 0;
int posX = 0;
int posY = 0;

HCSR04 u1(PA_0, PA_1),u2(PA_5, PA_6),u3(PB_0, PA_10);

BusOut LED(PA_4,PB_6,PB_7,PA_9);
//DigitalOut LEDa(PA_4),LEDb(PB_6),LEDc(PB_7),LEDd(PA_9);

Serial pc(USBTX, USBRX, 115200);
DigitalIn select(PF_1);

//Nucleo f303k8用
USB Usb(D11, D12, D13, A3, A2); // mosi, miso, sclk, ssel, intr
BTD Btd(&Usb);
PS3BT PS3(&Btd);

//CAN canPort(PA_11, PA_12);  //CAN name(PinName rd, PinName td) F303k8

/*
void LED(int led){
    if(led == 0){
        LEDa = 1;
        LEDb = 1;
        LEDc = 1;
        LEDd = 1;
    } 
}
*/

//unsigned int get_cm_n(HCSR04, unsigned int);
//USE -> unsigned int dist_UnitA = get_cm_n(u2, 5);
unsigned int get_cm_n(HCSR04 &echo_unit,int echo_n) {
    unsigned int sampled_dist=0;
    for (int iter_n = 0; iter_n <echo_n; iter_n++) {
        echo_unit.start();
        sampled_dist += echo_unit.get_dist_cm();
    }
    return  (sampled_dist / echo_n);
}

int nodeall=4;

void vel_right(int rpm) {
    sendTgtVel(1,rpm);
    sendTgtVel(2,rpm*(-1));
    sendTgtVel(3,rpm*(-1));
    sendTgtVel(4,rpm);
    for(int i=1; i<= 4; i++) {
        sendCtrlEN(i);
    }
}

void vel_right_con(int rpmA) {
    int dis1 = get_cm_n(u1,5);
    int dis2 = get_cm_n(u2,5);

    //速度を指定
    int robot_angle = ((dis1 - dis2)*5);
    sendTgtVel(1,rpmA+robot_angle);
    sendTgtVel(2,rpmA*(-1)+robot_angle);
    sendTgtVel(3,rpmA*(-1)+robot_angle);
    sendTgtVel(4,rpmA+robot_angle);
    //指令値を送信
    for(int i=1; i<= 4; i++) {
        sendCtrlEN(i);
    }
}


void vel_left(int rpm) {
    sendTgtVel(1,rpm*(-1));
    sendTgtVel(2,rpm);
    sendTgtVel(3,rpm);
    sendTgtVel(4,rpm*(-1));
    for(int i=1; i<= 4; i++) {
        sendCtrlEN(i);
    }
}


void vel_left_con(int rpmA) {
    int dis1 = get_cm_n(u1,5);
    int dis2 = get_cm_n(u2,5);

    //速度を指定
    int robot_angle = ((dis1 - dis2)*5);
    sendTgtVel(1,rpmA*(-1)+robot_angle);
    sendTgtVel(2,rpmA+robot_angle);
    sendTgtVel(3,rpmA+robot_angle);
    sendTgtVel(4,rpmA*(-1)+robot_angle);
    //指令値を送信
    for(int i=1; i<= 4; i++) {
        sendCtrlEN(i);
    }
}

void vel_stop() {
    //速度を指定
    sendTgtVel(1,0);
    sendTgtVel(2,0);
    sendTgtVel(3,0);
    sendTgtVel(4,0);
    //指令値を送信
    for(int i=1; i<= 4; i++) {
        sendCtrlEN(i);
    }
}

void vel_backward(int rpm) {
    //速度を指定
    sendTgtVel(1,rpm);
    sendTgtVel(2,rpm);
    sendTgtVel(3,rpm*(-1));
    sendTgtVel(4,rpm*(-1));
    //指令値を送信
    for(int i=1; i<= 4; i++) {
        sendCtrlEN(i);
    }
}

void vel_backward_con(int rpmA) {
    int dis1 = get_cm_n(u1,5);
    int dis2 = get_cm_n(u2,5);

    //速度を指定
    int robot_angle = ((dis1 - dis2)*10);
    sendTgtVel(1,rpmA+robot_angle);
    sendTgtVel(2,rpmA+robot_angle);
    sendTgtVel(3,rpmA*(-1)+robot_angle);
    sendTgtVel(4,rpmA*(-1)+robot_angle);
    //指令値を送信
    for(int i=1; i<= 4; i++) {
        sendCtrlEN(i);
    }
}


void vel_forward(int rpmA) {

    //速度を指定
    sendTgtVel(1,rpmA*(-1));
    sendTgtVel(2,rpmA*(-1));
    sendTgtVel(3,rpmA);
    sendTgtVel(4,rpmA);
    //指令値を送信
    for(int i=1; i<= 4; i++) {
        sendCtrlEN(i);
    }
}

void vel_forward_con(int rpmA) {
    int dis1 = get_cm_n(u1,5);
    int dis2 = get_cm_n(u2,5);

    //速度を指定
    int robot_angle = ((dis1 - dis2)*10);
    sendTgtVel(1,rpmA*(-1)+robot_angle);
    sendTgtVel(2,rpmA*(-1)+robot_angle);
    sendTgtVel(3,rpmA+robot_angle);
    sendTgtVel(4,rpmA+robot_angle);
    //指令値を送信
    for(int i=1; i<= 4; i++) {
        sendCtrlEN(i);
    }
}

void vel_rotate_cw(int rpm){
    sendTgtVel(1,rpm*(-1));
    sendTgtVel(2,rpm*(-1));
    sendTgtVel(3,rpm*(-1));
    sendTgtVel(4,rpm*(-1));
    for(int i=1;i<= 4;i++){
        sendCtrlEN(i);
    }
}

void vel_rotate_ccw(int rpm){
    sendTgtVel(1,rpm);
    sendTgtVel(2,rpm);
    sendTgtVel(3,rpm);
    sendTgtVel(4,rpm);
    for(int i=1;i<= 4;i++){
        sendCtrlEN(i);
    }
}

void set_ACC(int setACC_val) {
    sendProAcc(1,setACC_val);
    sendProAcc(2,setACC_val);
    sendProAcc(3,setACC_val);
    sendProAcc(4,setACC_val);
}

void set_DEC(int setDEC_val) {
    sendProDec(1,setDEC_val);
    sendProDec(2,setDEC_val);
    sendProDec(3,setDEC_val);
    sendProDec(4,setDEC_val);
}

void set_MODE_V() {
    sendOPModeV(1);
    sendOPModeV(2);
    sendOPModeV(3);
    sendOPModeV(4);
}

void set_MODE_T() {
    sendOPModeT(1);
    sendOPModeT(2);
    sendOPModeT(3);
    sendOPModeT(4);
}

void SerialRX(void) {
    Serialdata = pc.getc();
    //pc.printf("%c\r\n",Serialdata);
}

void  update_pos(){
    readActPos(1);
    encX=canmsgRx.data[4]+canmsgRx.data[5]*256+canmsgRx.data[6]*65536;
    if(encX > 8388608) {
        encX -= 0x1000000;//16進数でのマイナス処理 16^7 = 16,777,216 = 0x1000000
    }
    wait_ms(10);
    readActPos(2);
    encY=canmsgRx.data[4]+canmsgRx.data[5]*256+canmsgRx.data[6]*65536;
    if(encY > 8388608) {
        encY -= 0x1000000;//16進数でのマイナス処理 16^7 = 16,777,216 = 0x1000000
    }
    wait_ms(10);
    posX = (encX*0.707)-(encY*0.707);
    posY = (-encX*0.707)-(encY*0.707);
    //printf("encX=[%4d],encY=[%4d],posX=[%4d],posY=[%d]\r\n",encX,encY,posX,posY);
}

void gohome(){
    update_pos();
    while(posX>0) {
        update_pos();
        set_ACC(1000);//加速度設定
        set_DEC(1000);//減速度設定
        set_MODE_V();//速度制御モード送信
        vel_right(-300);//前進速度指令
    }
    vel_right(0);
    while(posY>0) {
        update_pos();
        set_ACC(1000);//加速度設定
        set_DEC(1000);//減速度設定
        set_MODE_V();//速度制御モード送信
        vel_forward_con(-300);//前進速度指令
    }
    
}

int Stick_vel_converter(int dead_band,int max_vel,int stick_input){    
    int dead_band_min;
    int dead_band_max;
    int output_data;

    double slope,intercept;

    dead_band_min = 127 - dead_band/2 - 1; 
    dead_band_max = 128 + dead_band/2 - 1;

    slope = max_vel / (255 - dead_band_max);
    intercept = max_vel - slope * 255;

    if(stick_input < dead_band_min){
        output_data = slope * stick_input - max_vel;
    }else if(stick_input > dead_band_max){
        output_data = slope * stick_input + intercept;
    }else{
        output_data = 0;
    }

    return output_data;    
}

int main(){
    
    //Serial
    pc.attach(SerialRX);
    //pc.baud(115200);
    select.mode(PullUp);

    //LED.mode(PullUp);

    //CAN
    canPort.frequency(1000000); //Bit Rate:1MHz
    canPort.attach(CANdataRX,CAN::RxIrq);

    int deadbanded_rpm_x, deadbanded_rpm_y;
    int rpm_l, rpm_r;
    int rpm_button;

    //エンコーダ関係
    int ActPos = 0;
    int Init_Pos = 0;

    //超音波センサ関係パラメータ
    int dist1,dist2,dist3,dist4;

    printf("\nstart\r\n");
    //LED = 0b1111;

    while (1){
        pc.printf("%d\n\r",select.read());
                
        if(select == 0){ //auto
            //printf("auto_start\r\n");
            PS3.disconnect();

            set_MODE_T();

            sendCtrlSD(1);
            sendCtrlSD(2);
            sendCtrlSD(3);
            sendCtrlSD(4);

            sendCtrlEN(1);
            sendCtrlEN(2);
            sendCtrlEN(3);
            sendCtrlEN(4);

            //初期加減速度
            int Acc = 2000;
            int Dec = 2000;

            set_ACC(Acc);
            set_DEC(Dec);

            //トルク設定
            int trq = 100;   //torque Setting[mA]

            sendTgtTrq(1,trq);
            sendTgtTrq(2,trq);
            sendTgtTrq(3,trq);
            sendTgtTrq(4,trq);

            int state_1 = 0;
            int state_2 = 0;
            int ride_count = 0;

            int X_DIST_TMP = 0;
            int dist1_ori = 0;
            int dist2_ori = 0;

            dist1 = 0;
            readActPos(1);
            ActPos=canmsgRx.data[4]+canmsgRx.data[5]*256+canmsgRx.data[6]*65536;
            if(ActPos > 8388608) {
                ActPos -= 0x1000000;//16進数でのマイナス処理 16^7 = 16,777,216 = 0x1000000
            }
            Init_Pos = ActPos;//起動時の角度を保存
            t.reset();
            t.start();
            cr_t.reset();

            while(select == 0){ //auto loop
                Time = t.read();
                cr_time = cr_t.read();
                
                pc.printf("%d\n\r",select.read());
                //pc.printf("state_1:%d  state_2:%d\r\n",state_1,state_2);
                //pc.printf("state_1:%d state_2:%d ACT:%d TMP:%d \r\n",state_1,state_2,dist3,X_DIST_TMP);
                readActPos(1);
                ActPos=canmsgRx.data[4]+canmsgRx.data[5]*256+canmsgRx.data[6]*65536;
                if(ActPos > 8388608) {
                    ActPos -= 0x1000000;//16進数でのマイナス処理 16^7 = 16,777,216 = 0x1000000
                    //printf("check\r\n");
                }
                dist3 = get_cm_n(u3, 5);
                dist1 = get_cm_n(u1, 5);
                dist2 = get_cm_n(u2, 5);

                /*--------------------------*/
                //
                if(state_1 == 0) { //入力判断フェーズ
                    state_2 = 0;
                    if(ride_count >= 2 && Time > Standby_Time) {
                        state_1 = 20;
                    } else {
                        if(ActPos < (Init_Pos - KICK)) { //前入力検出
                            ride_count++;
                            LED = 0b0111;
                            state_1 = 1;
                        } else if(ActPos > (Init_Pos + KICK)) { //右入力検出
                            ride_count++;
                            LED = 0b1101;;
                            state_1 = 11;
                        } else {
                            set_MODE_T();
                            LED = 0b1111;
                        }
                    }

                } else if(state_1 == 1) { //前進→壁検出フェーズ
                    if(dist1 < WALL && dist1 >= WALL_MIN) {
                        vel_stop();
                        wait(GETOFF_TIME);
                        LED = 0b1011;
                        state_1 = 2;
                    } else {
                        set_ACC(ACC_RIDE);//加速度設定
                        set_DEC(DEC_CLEAN);//減速度設定
                        set_MODE_V();//速度制御モード送信
                        vel_forward(RPM_RIDE);//前進速度指令
                    }
                } else if(state_1 == 2) { //前進からの帰還フェーズ
                    if(ActPos > -3000) {
                        vel_stop();
                        LED = 0b1111;
                        t.reset();
                        t.start();
                        state_1 = 0;
                        wait(1.0);
                    } else {
                        vel_backward(RPM_RIDE);
                    }
                } else if(state_1 == 11) { //右進→壁検出フェーズ
                    if(dist3 < WALL && dist3 >= WALL_MIN) {
                        vel_stop();
                        wait(GETOFF_TIME);
                        LED = 0b1110;
                        state_1 = 12;
                    } else {
                        set_ACC(ACC_RIDE);//加速度設定
                        set_DEC(DEC_RIDE);//減速度設定
                        set_MODE_V();//速度制御モード送信
                        vel_right(RPM_RIDE);//R進速度指令
                    }
                } else if(state_1 == 12) { //右進からの帰還フェーズ
                    if(ActPos < 3000) {
                        vel_stop();
                        LED = 1111;
                        t.reset();
                        t.start();
                        state_1 = 0;
                        wait(1.0);
                    } else {
                        vel_left(RPM_RIDE);
                    }

                    //////////////////////////////
                } else if(state_1 == 20) { //消毒モード

                    if(state_2 == 0) {
                        if(dist1 < WALL && dist1 >= WALL_MIN) {
                            X_DIST_TMP = dist3;
                            state_2 = 1;
                        } else {
                            set_ACC(ACC_CLEAN);//加速度設定
                            set_DEC(DEC_CLEAN);//減速度設定
                            set_MODE_V();//速度制御モード送信
                            vel_forward_con(RPM_CLEAN);//前進速度指令
                        }
                    } else if(state_2 == 1) {
                        int dist3_tmp = get_cm_n(u1,5);
                        wait_ms(10);
                        // if(dist3 < WALL && dist3 >= WALL_MIN){
                        // if(dist3_tmp<10){
                        update_pos();
                        if(posX>12000) {
                            gohome();
                            state_2 = 4;
                        } else {
                            set_ACC(ACC_CLEAN);//加速度設定
                            set_DEC(DEC_CLEAN);//減速度設定
                            set_MODE_V();//速度制御モード送信
                            vel_right(RPM_CLEAN);//右進速度指令
                            wait(CLEAN_OFFSET);
                            state_2 = 2;
                        }

                    } else if(state_2 == 2) {
                        if(ActPos > -3000) {
                            state_2 = 3;
                        } else {
                            set_ACC(ACC_CLEAN);//加速度設定
                            set_DEC(DEC_CLEAN);//減速度設定
                            set_MODE_V();//速度制御モード送信
                            vel_backward_con(RPM_CLEAN);//後進速度指令
                        }
                    } else if(state_2 == 3) {
                        if(dist3 < WALL && dist3 >= WALL_MIN) {
                            state_2 = 4;
                        } else {
                            set_ACC(ACC_CLEAN);//加速度設定
                            set_DEC(DEC_CLEAN);//減速度設定
                            set_MODE_V();//速度制御モード送信
                            vel_right(RPM_CLEAN);//右進速度指令
                            wait(CLEAN_OFFSET);
                            state_2 = 0;
                        }
                    } else if(state_2 == 4) {
                        if(ActPos < 3000) {
                            state_2 = 5;
                        } else {
                            vel_left(RPM_CLEAN);
                        }
                    } else if(state_2 == 5) {
                        if(ActPos > -3000) {
                            t.reset();
                            t.start();
                            state_1 = 0;
                            state_2 = 0;
                        } else {
                            vel_backward_con(RPM_CLEAN);
                        }
                    }
                }
            }
            
        }else{ //manual

            
            set_MODE_V();
            sendCtrlSD(1);
            sendCtrlSD(2);
            sendCtrlSD(3);
            sendCtrlSD(4);
 
            sendCtrlEN(1);
            sendCtrlEN(2);
            sendCtrlEN(3);
            sendCtrlEN(4);
            
            set_ACC(ACC);
            set_DEC(DEC);


            printf("manual_start\r\n");
            if (Usb.Init() == -1){
                //pc1.printf("\r\nOSC did not start");
                while (1); // Halt
            }
            //pc1.printf("\r\nPS3 USB Library Started");

            //LED = 0b1111;
            //LED(0);

            while(select != 0){ //manual loop
            
                pc.printf("%d\n\r",select.read());
                
                /*
                readActPos(1);
                ActPos=canmsgRx.data[4]+canmsgRx.data[5]*256+canmsgRx.data[6]*65536;
                if(ActPos > 8388608) {
                    ActPos -= 0x1000000;//16進数でのマイナス処理 16^7 = 16,777,216 = 0x1000000
                }
                printf("%d \r\n",ActPos);
                */

                Usb.Task();

                if (PS3.PS3Connected){
                    if(PS3.getAnalogButton(UP) > BUTTON_DEADBAND){
                        rpm_button = PS3.getAnalogButton(UP) * MAX_RPM_BUTTON / 255;
                        vel_forward(rpm_button);
                        LED = 0b0111;
                    }else if(PS3.getAnalogButton(DOWN) > BUTTON_DEADBAND){
                        rpm_button = PS3.getAnalogButton(DOWN) * MAX_RPM_BUTTON / 255;
                        vel_backward(rpm_button);
                        LED = 0b1011;
                    }else if(PS3.getAnalogButton(RIGHT) > BUTTON_DEADBAND){
                        rpm_button = PS3.getAnalogButton(RIGHT) * MAX_RPM_BUTTON / 255;
                        vel_right(rpm_button);
                        LED = 0b1101;
                    }else if(PS3.getAnalogButton(LEFT) > BUTTON_DEADBAND){
                        rpm_button = PS3.getAnalogButton(LEFT) * MAX_RPM_BUTTON / 255;
                        vel_left(rpm_button);
                        LED = 0b1110;
                    }else if(PS3.getAnalogButton(R2) > BUTTON_DEADBAND){
                        rpm_button = PS3.getAnalogButton(R2) * MAX_RPM_BUTTON / 255;
                        vel_rotate_ccw(rpm_button);
                        LED = 0b1111;
                    }else if(PS3.getAnalogButton(L2) > BUTTON_DEADBAND){
                        rpm_button = PS3.getAnalogButton(L2) * MAX_RPM_BUTTON / 255;
                        vel_rotate_cw(rpm_button);
                        LED = 0b1111;
                    }else{
                    
                        rpm_button = 0;
                        deadbanded_rpm_y = Stick_vel_converter(STICK_DEADBAND,MAX_RPM,PS3.getAnalogHat(RightHatX));
                        deadbanded_rpm_x = Stick_vel_converter(STICK_DEADBAND,MAX_RPM,PS3.getAnalogHat(RightHatY)) * -1;

                        rpm_l = (deadbanded_rpm_x + deadbanded_rpm_y);
                        rpm_r = (deadbanded_rpm_x - deadbanded_rpm_y); 

                        sendTgtVel(1,rpm_r*(-1));
                        sendTgtVel(2,rpm_r*(-1));
                        sendTgtVel(3,rpm_l);
                        sendTgtVel(4,rpm_l);
                        //指令値を送信
                        for(int i=1;i<= 4;i++){
                            sendCtrlEN(i);   
                        }
                        
                        if(rpm_l > 0 && rpm_r > 0){
                            LED = 0b0111;
                        }else if(rpm_l < 0 && rpm_r < 0){
                            LED = 0b1011;
                        }else{
                            LED = 0b1111;
                        }

                    }
                    
                }else{
                    LED = 0b1111;
                    //LED(0);
                    vel_stop();
                }
            }
        }
    }     
}