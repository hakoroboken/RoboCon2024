#ifndef RMPID_H_
#define RMPID_H_

namespace hakorobo2024
{
    class RMPID{
        public:
        void setup(float pid_param_k)
        {
            pid_pram_k = pid_param_k;
            prev_input_UsinginputDelay = 0.0;
            prev_output_UsinginputDelay = 0.0;
            prev_ampare = 0.0;
            prev_output = 0.0;
        }

        void set_value(float target, float actual)
        {
            target_speed_rpm = target;
            sense_speed_rpm = actual;
        }

        float calculate()
        {
            const double filteredRPM = inputDelay(target_speed_rpm);
            const double needRPM = filteredRPM + pid_pram_k * (filteredRPM - sense_speed_rpm);
            const double wantAmpare = needRPM * 0.29411764705882354;
            const double modelValue = Pm( wantAmpare);
            double inputAmpare = wantAmpare + ( modelValue - sense_speed_rpm);
            
            return inputAmpare;
        }

        private:
        const float inputDelay(const float & input)
        {
            const auto output = 
                0.04761904761904762 * input + 
                0.04761904761904762 * prev_input_UsinginputDelay + 
                0.9047619047619048 * prev_output_UsinginputDelay;

            prev_input_UsinginputDelay = input;
            prev_output_UsinginputDelay = output;

            return output;
        }
        const float Pm(const float & wantAmpare)
        {
            const auto plantModel = 0.085 * wantAmpare + 0.085 * prev_ampare + 0.95 * prev_output;

            prev_ampare = wantAmpare;
            prev_output = plantModel;

            return plantModel;
        }
        float sense_speed_rpm;
        float target_speed_rpm;
        float pid_pram_k;
        float control_order;
        float prev_input_UsinginputDelay;
        float prev_output_UsinginputDelay;
        double prev_ampare;
        static double prev_output;
    };
}

#endif