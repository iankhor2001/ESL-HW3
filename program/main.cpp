#include "mbed.h"
#include "uLCD_4DGL.h"
#include "TFconfig.h"
#include <cmath>
#include "magic_wand_model_data.h"
#include "accelerometer_handler.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "stm32l475e_iot01_accelero.h"
#include "mbed_rpc.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

#define PI 3.14159265

// Initialize a pins to perform analog and digital output functions
// Adjust analog output pin name to your board spec.
uLCD_4DGL uLCD(D1, D0, D2); 
InterruptIn btn(USER_BUTTON);
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
Thread gesT,t,btn_th;
int mode=1;
int angle[6]={15,20,25,30,35,40};
int n=10;
int UI_state=1;
EventQueue gesture_queue,mqtt_queue,queue,btn_queue;
BufferedSerial pc(USBTX, USBRX);
int msg_mode=0;
int gesture_mode=1;

//WIFI
WiFiInterface *wifi;
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
const char* topic = "Mbed";
Thread mqtt_thread(osPriorityHigh);

//TILT
Thread tiltT;
Thread acce_thread;
EventQueue tilt_queue;
EventQueue acce_queue;
double tilt_angle_deg=0, ref_angle_deg=0;
int16_t pDataXYZ_tilt[3] = {0}, init_pDataXYZ_tilt[3]={999,999,999};
int running = 1;
int over_max_times=0;
int init_angle_confirm=0;

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    printf(msg);
    ThisThread::sleep_for(100ms);
    char payload[300];
    sprintf(payload, "Payload :%.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}
void close_mqtt() {
    closed = true;
}
void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    led3=!led3;
    MQTT::Message message;
    char buff[100];
    switch(msg_mode){
        case 1: 
            switch(mode){
                case 0: sprintf(buff, "---close gesture---Selected 15deg maximum angle---"); break; 
                case 1: sprintf(buff, "---close gesture---Selected 20deg maximum angle---"); break;
                case 2: sprintf(buff, "---close gesture---Selected 25deg maximum angle---"); break;
                case 3: sprintf(buff, "---close gesture---Selected 30deg maximum angle---"); break;
                case 4: sprintf(buff, "---close gesture---Selected 35deg maximum angle---"); break;
                case 5: sprintf(buff, "---close gesture---Selected 40deg maximum angle---"); break;
            }   break;
        case 2:
            sprintf(buff, "---none---Angle over threshold #%d---",over_max_times); break; 
            break;     

    }
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic, message);
    close_mqtt();
    gesture_mode=0;
    // printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
}

///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

// Return the result of the last prediction
int PredictGesture(float* output) {
    // How many times the most recent gesture has been matched in a row
    static int continuous_count = 0;
    // The result of the last prediction
    static int last_predict = -1;

    // Find whichever output has a probability > 0.8 (they sum to 1)
    int this_predict = -1;
    for (int i = 0; i < label_num; i++) {
        if (output[i] > 0.9) this_predict = i;
    }

    // No gesture was detected above the threshold
    if (this_predict == -1) {
        continuous_count = 0;
        last_predict = label_num;
        return label_num;
    }

    if (last_predict == this_predict) {
        continuous_count += 1;
    } else {
        continuous_count = 0;
    }
    last_predict = this_predict;

      // If we haven't yet had enough consecutive matches for this gesture,
    // report a negative result
    if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
      return label_num;
    }
    // Otherwise, we've seen a positive result, so clear all our variables
    // and report it
    continuous_count = 0;
    last_predict = -1;

    return this_predict;
}

void gesture_display(){
    uLCD.locate(0,0);
    uLCD.printf("Gesture Mode");

    uLCD.text_width(2); 
    uLCD.text_height(2);
    uLCD.locate(0,1);
    uLCD.color(GREEN);
    uLCD.printf(" >"); 
    uLCD.printf(" 15"); 

    uLCD.text_width(2); 
    uLCD.text_height(2);
    uLCD.locate(0,2);
    uLCD.color(GREEN);
    uLCD.printf(" >"); 
    uLCD.printf(" 20"); 

    uLCD.text_width(2); 
    uLCD.text_height(2);
    uLCD.locate(0,3);
    uLCD.color(GREEN);
    uLCD.printf(" >"); 
    uLCD.printf(" 25"); 
  
    uLCD.text_width(2); 
    uLCD.text_height(2);
    uLCD.locate(0,4);
    uLCD.color(GREEN);
    uLCD.printf(" >"); 
    uLCD.printf(" 30");

    uLCD.text_width(2); 
    uLCD.text_height(2);
    uLCD.locate(0,5);
    uLCD.color(GREEN);
    uLCD.printf(" >"); 
    uLCD.printf(" 35");

    uLCD.text_width(2); 
    uLCD.text_height(2);
    uLCD.locate(0,6);
    uLCD.color(GREEN);
    uLCD.printf(" >"); 
    uLCD.printf(" 40");
}
void change_mode(int mode_in){
    int row_now = mode_in+1;
    for(int num=1;num<=6;num++){
        uLCD.locate(0,num);
        if (num==row_now){
            uLCD.color(WHITE);
            uLCD.printf(" >"); 
        }
        else {
            uLCD.printf("  ");
        }
    }
}
void gesture_tf(){
    while(gesture_mode){
        gesture_display();
        ThisThread::sleep_for(1s);
        // Whether we should clear the buffer next time we fetch data
        bool should_clear_buffer = false;
        bool got_data = false;

        // The gesture index of the prediction
        int gesture_index;

        // Set up logging.
        static tflite::MicroErrorReporter micro_error_reporter;
        tflite::ErrorReporter* error_reporter = &micro_error_reporter;

        // Map the model into a usable data structure. This doesn't involve any
        // copying or parsing, it's a very lightweight operation.
        const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
        if (model->version() != TFLITE_SCHEMA_VERSION) {
            error_reporter->Report(
                "Model provided is schema version %d not equal "
                "to supported version %d.",
                model->version(), TFLITE_SCHEMA_VERSION);
        }

        // Pull in only the operation implementations we need.
        // This relies on a complete list of all the ops needed by this graph.
        // An easier approach is to just use the AllOpsResolver, but this will
        // incur some penalty in code space for op implementations that are not
        // needed by this graph.
        static tflite::MicroOpResolver<6> micro_op_resolver;
        micro_op_resolver.AddBuiltin(
            tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
            tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
        micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                                    tflite::ops::micro::Register_MAX_POOL_2D());
        micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                                    tflite::ops::micro::Register_CONV_2D());
        micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                                    tflite::ops::micro::Register_FULLY_CONNECTED());
        micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                                    tflite::ops::micro::Register_SOFTMAX());
        micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                                    tflite::ops::micro::Register_RESHAPE(), 1);

        // Build an interpreter to run the model with
        static tflite::MicroInterpreter static_interpreter(
            model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
        tflite::MicroInterpreter* interpreter = &static_interpreter;

        // Allocate memory from the tensor_arena for the model's tensors
        interpreter->AllocateTensors();

        // Obtain pointer to the model's input tensor
        TfLiteTensor* model_input = interpreter->input(0);
        if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
            (model_input->dims->data[1] != config.seq_length) ||
            (model_input->dims->data[2] != kChannelNumber) ||
            (model_input->type != kTfLiteFloat32)) {
            error_reporter->Report("Bad input tensor parameters in model");

        }

        int input_length = model_input->bytes / sizeof(float);

        TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
        if (setup_status != kTfLiteOk) {
            error_reporter->Report("Set up failed\n");

        }

        //error_reporter->Report("Set up successful...\n");

    ///////////////////////////////////////////////////////////////////////////////////////////////////

        while (gesture_mode) {

        // Attempt to read new data from the accelerometer
        got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                        input_length, should_clear_buffer);

        // If there was no new data,
        // don't try to clear the buffer again and wait until next time
        if (!got_data) {
            should_clear_buffer = false;
            continue;
        }

        // Run inference, and report any error
        TfLiteStatus invoke_status = interpreter->Invoke();
        if (invoke_status != kTfLiteOk) {
            error_reporter->Report("Invoke failed on index: %d\n", begin_index);
            continue;
        }

        // Analyze the results to obtain a prediction
        gesture_index = PredictGesture(interpreter->output(0)->data.f);

        // Clear the buffer next time we read data
        should_clear_buffer = gesture_index < label_num;

        // Produce an output
        // if (gesture_index < label_num) {
        //     error_reporter->Report(config.output_message[gesture_index]);
        // }
        int mode_temp = mode;
        if (gesture_index==1)
            mode++;
        else if (gesture_index==0)
            mode--;
        if(mode>4)
            mode=-1;
        if (mode<-1)
            mode=4;
        if(mode_temp!=mode)
            change_mode(mode);
        
    
        }
    }
    ThisThread::sleep_for(100ms);
}
int gesture()
{
    msg_mode=1;
    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
    }
    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }
    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
    //TODO: revise host to your IP
    const char* host = "192.168.0.107";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }
    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    btn.rise(mqtt_queue.event(&publish_message,&client));
    t.start(callback(&queue, &EventQueue::dispatch_forever));
    printf("starting gesture_tf");
    queue.call(gesture_tf);

    int num = 0;
    while (num != 5) {
            client.yield(100);
            ++num;
    }

    while (1) {
            if (closed) break;
            client.yield(500);
            ThisThread::sleep_for(500ms);
    }
    uLCD.cls();
}
void gesture_activate(Arguments *in, Reply *out){
    uLCD.cls();
    gesture_mode = 1;
    led2=1;
    led1=1;
    gesture_queue.call(gesture);
}
void gesture_terminate(Arguments *in, Reply *out){
    gesture_mode = 0;
    led2=0;
    led1=0;
    uLCD.cls();
    uLCD.locate(0,1);
    uLCD.printf("gesture");
    uLCD.locate(0,2);
    uLCD.printf("Angle = %d",angle[mode]);
}
RPCFunction rpcGestureActive(&gesture_activate, "gesture_activate");
RPCFunction rpcGestureDeactive(&gesture_terminate, "gesture_terminate");

//////////////////////////////////////////////////////////////////////////////////////////
int tilt_msg=0;

void tilt_init(){
    BSP_ACCELERO_AccGetXYZ(init_pDataXYZ_tilt);
    double ref_angle_rad = acos ( init_pDataXYZ_tilt[2]/ sqrt( pow(init_pDataXYZ_tilt[0],2) + pow(init_pDataXYZ_tilt[1],2) + pow(pDataXYZ_tilt[2],2) ) );
    ref_angle_deg = ref_angle_rad  * 180.0 / PI;
    init_angle_confirm=1;
}
void tilt_angle(){
    while(running){
        BSP_ACCELERO_AccGetXYZ(pDataXYZ_tilt);
        double tilt_angle_rad = acos ( pDataXYZ_tilt[2]/ sqrt( pow(pDataXYZ_tilt[0],2) + pow(pDataXYZ_tilt[1],2) + pow(pDataXYZ_tilt[2],2) ) );
        tilt_angle_deg = tilt_angle_rad  * 180.0 / PI;
        int tilting = int( abs( tilt_angle_deg - ref_angle_deg ) );
        uLCD.text_width(2); 
        uLCD.text_height(2);
        uLCD.locate(1,1);
        uLCD.printf("Max: %d", angle[mode]);
        uLCD.text_width(3); 
        uLCD.text_height(3);
        uLCD.locate(1,2);
        uLCD.printf("deg:"); 
        uLCD.locate(1,3);
        if(tilting>=angle[mode]) uLCD.color(RED);
        else uLCD.color(GREEN);
        uLCD.printf("%3d", int(tilting)); 

        if(tilting>=angle[mode]) {
            tilt_msg=1;
            uLCD.text_width(1); 
            uLCD.text_height(1);
            uLCD.locate(0,0);
            uLCD.color(RED);
            uLCD.printf("OVER #%d", over_max_times);
            over_max_times=over_max_times+1;
            uLCD.color(GREEN);
            uLCD.text_width(2); 
            uLCD.text_height(2);
            ThisThread::sleep_for(1s);
        }
        if(over_max_times==5) running=0;
        // if(tilting>=angle[mode]) running = 0;
        ThisThread::sleep_for(100ms);
    }
}
void tilt_op(){
    init_angle_confirm=0;
    uLCD.cls();
    running = 1;
    BSP_ACCELERO_Init();
    led2=1;
    while(init_angle_confirm==0){
        uLCD.text_width(2); 
        uLCD.text_height(2);
        uLCD.locate(1,0);
        uLCD.printf("PUSH\n BUTTON\n TO\n INIT"); 
        // btn.fall(tilt_init);
        btn.rise(mqtt_queue.event(&tilt_init));
    }
    uLCD.cls();
    uLCD.text_width(1); 
    uLCD.text_height(1);
    uLCD.locate(1,0);
    uLCD.printf("ref: %d,%d,%d", init_pDataXYZ_tilt[0],init_pDataXYZ_tilt[1],init_pDataXYZ_tilt[2]); 
    over_max_times=0;
    tilt_angle();
    ThisThread::sleep_for(100ms);
    uLCD.cls();
}
int tilt_wifi(){
    msg_mode=2;
    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
    }
    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }
    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
    //TODO: revise host to your IP
    const char* host = "192.168.0.107";
    printf("Connecting to TCP network...\r\n");
    ///////////////////////////////////////////////////////////
    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);
    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting
    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";
    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }
    
    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    t.start(callback(&queue, &EventQueue::dispatch_forever));
    printf("starting tilt_op");
    queue.call(tilt_op);

    int num = 0;
    while (num != 5) {
            client.yield(100);
            ++num;
    }

    while (1) {
            if (tilt_msg) {
                mqtt_queue.event(&publish_message,&client);
                // ThisThread::sleep_for(250ms);
                tilt_msg=0;
            }
            if (closed) break;
            client.yield(500);
            ThisThread::sleep_for(500ms);
    }
    uLCD.cls();
}
void tilt_activate(Arguments *in, Reply *out){
    tiltT.start(callback(&tilt_queue, &EventQueue::dispatch_forever));
    tilt_queue.call(tilt_op);
}
void tilt_terminate(Arguments *in, Reply *out){
    led2=0;
    uLCD.cls();
    uLCD.locate(0,1);
    uLCD.printf("MENU");
    uLCD.locate(0,2);
    uLCD.printf("Angle = %d",angle[mode]);
}
RPCFunction rpcTiltActive(&tilt_activate, "tilt_activate");
RPCFunction rpcTiltDeactive(&tilt_terminate, "tilt_terminate");

///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////

int main(){

    uLCD.locate(0,1);
    uLCD.printf("MENU");
    uLCD.locate(0,2);
    uLCD.printf("Angle = %d",angle[mode]);

    gesT.start(callback(&gesture_queue, &EventQueue::dispatch_forever));

    // receive commands, and send back the responses
    char buf[256], outbuf[256];
    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");
    while(1) {
        memset(buf, 0, 256);
        for (int i = 0; ; i++) {
            char recv = fgetc(devin);
            if (recv == '\n') {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        //Call the static call method on the RPC class
        RPC::call(buf, outbuf);
        printf("%s\r\n", outbuf);
    }
    
    return 0;

}

