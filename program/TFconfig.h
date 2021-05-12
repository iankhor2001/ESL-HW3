#ifndef TFCONFIG_H_
#define TFCONFIG_H_

// The number of labels (without negative)
#define label_num 2

struct Config {

  // This must be the same as seq_length in the src/model_train/config.py
  const int seq_length = 64;

  // The number of expected consecutive inferences for each gesture type.
  const int consecutiveInferenceThresholds[label_num] = {20, 10};

  const char* output_message[label_num] = {
        "UP:\n\r"
        "          *       \n\r"
        "       *  *  *    \n\r"
        "          *       \n\r"
        "          *       \n\r"
        "          *       \n\r"
        "          *       \n\r"
        "          *       \n\r",
        "DOWN:\n\r"
        "         *        \n\r"
        "         *        \n\r"
        "         *        \n\r"
        "         *        \n\r"
        "         *        \n\r"
        "         *        \n\r"
        "      *  *  *     \n\r"
        "         *        \n\r"
        };
};

Config config;
#endif // CONFIG_H_
