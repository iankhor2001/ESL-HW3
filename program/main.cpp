#include "mbed.h"
#include "uLCD_4DGL.h"
#include "config.h"
#include "magic_wand_model_data.h"
#include "accelerometer_handler.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "mbed_rpc.h"

// Initialize a pins to perform analog and digital output functions
// Adjust analog output pin name to your board spec.
uLCD_4DGL uLCD(D1, D0, D2); 
InterruptIn btn(USER_BUTTON);
Thread t,mqtt_thread;
int mode=0;
int n=10;
int UI_state=1;
EventQueue gesture_queue;
BufferedSerial pc(USBTX, USBRX);


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

void display(){
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
    uLCD.printf("  20"); 

    uLCD.text_width(2); 
    uLCD.text_height(2);
    uLCD.locate(0,3);
    uLCD.color(GREEN);
    uLCD.printf(" >"); 
    uLCD.printf("  25"); 
  
    uLCD.text_width(2); 
    uLCD.text_height(2);
    uLCD.locate(0,4);
    uLCD.color(GREEN);
    uLCD.printf(" >"); 
    uLCD.printf("  30");

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
    int row_now = mode_in+2;
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

void gesture()
{

    display();
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

    error_reporter->Report("Set up successful...\n");

///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
    while (1) {

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
    if (gesture_index < label_num) {
        error_reporter->Report(config.output_message[gesture_index]);
    }
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
    // btn.fall(mqtt_queue.event(&publish_message, &client));
    

  }

        ThisThread::sleep_for(100ms);
}

void gesture_activate(Arguments *in, Reply *out){
    t.start(callback(&gesture_queue, &EventQueue::dispatch_forever));
    gesture_queue.call(gesture);
}
void gesture_terminate(Arguments *in, Reply *out){
    t.terminate();
    uLCD.cls();
}

RPCFunction rpcGestureActive(&gesture_activate, "gesture_activate");
RPCFunction rpcGestureDeactive(&gesture_terminate, "gesture_terminate");


int main(){

    // mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));

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


}
