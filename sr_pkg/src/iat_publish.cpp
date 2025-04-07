#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "/home/nuc1003a/ARTS_TEST/src/sr_pkg/include/qisr.h"
#include "/home/nuc1003a/ARTS_TEST/src/sr_pkg/include/msp_cmn.h"
#include "/home/nuc1003a/ARTS_TEST/src/sr_pkg/include/msp_errors.h"
#include "/home/nuc1003a/ARTS_TEST/src/sr_pkg/include/speech_recognizer.h"
#include <iconv.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#define FRAME_LEN   640 
#define BUFFER_SIZE 4096

int wakeupFlag   = 0;
int resultFlag   = 0;

static void show_result(char *string, char is_over)
{
    resultFlag = 1;   
    printf("\rResult: [ %s ]", string);
    if (is_over)
        putchar('\n');
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
    if (result) {
        size_t left = g_buffersize - 1 - strlen(g_result);
        size_t size = strlen(result);
        if (left < size) {
            g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
            if (g_result)
                g_buffersize += BUFFER_SIZE;
            else {
                printf("mem alloc failed\n");
                return;
            }
        }
        strncat(g_result, result, size);
        show_result(g_result, is_last);
    }
}

void on_speech_begin()
{
    if (g_result) {
        free(g_result);
    }
    g_result = (char*)malloc(BUFFER_SIZE);
    g_buffersize = BUFFER_SIZE;
    memset(g_result, 0, g_buffersize);

    printf("Start Listening...\n");
}

void on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT)
        printf("\nSpeaking done \n");
    else
        printf("\nRecognizer error %d\n", reason);
}

static void demo_mic(const char* session_begin_params)
{
    int errcode;
    int i = 0;

    struct speech_rec iat;

    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end
    };

    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
    if (errcode) {
        printf("speech recognizer init failed\n");
        return;
    }
    errcode = sr_start_listening(&iat);
    if (errcode) {
        printf("start listen failed %d\n", errcode);
    }

    while(i++ < 10)
        sleep(1);
    errcode = sr_stop_listening(&iat);
    // 按下E键结束监听
    // char c;
    // while (1) {
    //     c = getchar();
    //     if (c == 'E' || c == 'e') {
    //         break;
    //     }
    // }

    errcode = sr_stop_listening(&iat);
    if (errcode) {
        printf("stop listening failed %d\n", errcode);
    }

    sr_uninit(&iat);
}

void WakeUp(const std_msgs::String::ConstPtr& msg)
{
    printf("waking up\r\n");
    usleep(700*1000);
    wakeupFlag = 1;
}

void Wake()
{
    printf("waking up\r\n");
    usleep(700*1000);
    wakeupFlag = 1;
}

// 读取键盘输入的函数
int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int main(int argc, char* argv[])
{
    // 初始化ROS
    ros::init(argc, argv, "voiceRecognition");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // 声明Publisher和Subscriber
    ros::Subscriber wakeUpSub = n.subscribe("voiceWakeup", 1000, WakeUp);   
    ros::Publisher voiceWordsPub = n.advertise<std_msgs::String>("voiceWords", 1000);  

    ROS_INFO("Press Q to start listening...");
    while (ros::ok())
    {
        // 检查Q键是否按下
        if (kbhit())
        {
            char c = getchar();
            if (c == 'Q' || c == 'q')
            {
                ROS_INFO("Wakeup...");
                int ret = MSP_SUCCESS;
                const char* login_params = "appid = adc23377, work_dir = .";

                const char* session_begin_params =
                    "sub = iat, domain = iat, language = zh_cn, "
                    "accent = mandarin, sample_rate = 16000, "
                    "result_type = plain, result_encoding = utf8";

                ret = MSPLogin(NULL, NULL, login_params);
                if (MSP_SUCCESS != ret) {
                    MSPLogout();
                    printf("MSPLogin failed , Error code %d.\n", ret);
                }

                printf("Demo recognizing the speech from microphone\n");

                demo_mic(session_begin_params);

                MSPLogout();
            }
        }

        // 语音识别完成
        if (resultFlag) {
            resultFlag = 0;
            std_msgs::String msg;
            msg.data = g_result;
            voiceWordsPub.publish(msg);
            printf("pub\n");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    MSPLogout(); // Logout...

    return 0;
}
