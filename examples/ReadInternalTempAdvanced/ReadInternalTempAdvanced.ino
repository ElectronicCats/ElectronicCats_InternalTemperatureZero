#include <TemperatureZero.h>

TemperatureZero TempZero = TemperatureZero();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  TempZero.init();
  
  // Correct the output using a 2-point measurement.
  // Let's setup a case where the sensor mistakenly shifts down by one degree and halves the temperature:
  // Real cold temp = 10 degres, TempZero indicates (10 - 1) / 2 = 4.5
  // Real hot temp = 100 degrees, TempZero indicates (100 - 1) / 2 = 49.5
  // We would now expect the user correction to double the temperature and shift it up 1 degree
  TempZero.setUserCalibration2P(10.0, 4.5, 100.0, 49.5, false);
}

uint16_t averaging_mode[] = {
  TZ_AVERAGING_1,
  TZ_AVERAGING_2,
  TZ_AVERAGING_4,
  TZ_AVERAGING_8,
  TZ_AVERAGING_16,
  TZ_AVERAGING_32,
  TZ_AVERAGING_64,
  TZ_AVERAGING_128,
  TZ_AVERAGING_256,
};

const char * average_mode_desc[] = {"1  ", "2  ", "4  ", "8  ", "16 ", "32 ", "64 ", "128", "256"};

#define NR_AVG_MODES 9
#define MEASUREMENTS_PER_MODE 100

struct {
  uint16_t delay_avg;
  float temp_min;
  float temp_max;
  float temp_avg;
  float temp_var;
} stats_avg_mode;

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("\nUser calibration evaluation, expectation T x 2 + 1: ");

  // Default averaging is set to 64 samples per measurement, to cancel some noise.
  // Let's increase the averaging to use 256 samples, it will slow things down,
  // but will result in a more accurate measurement.
  TempZero.setAveraging(TZ_AVERAGING_256);

  TempZero.enableUserCalibration(false);
  float temp = TempZero.readInternalTemperature();
  Serial.print("Internal Temperature is : ");
  Serial.print(temp, 1);
  
  TempZero.enableUserCalibration(true);
  float userTemp = TempZero.readInternalTemperature();
  Serial.print(", user corrected : ");
  Serial.println(userTemp, 1);

  // Now let's evaluate the effect of varying the averaging from 1 to 256 samples
  // This influences the measurement speed as well as the variance in the results
  // As you can see the default of 64 samples is a compromise
  TempZero.enableUserCalibration(false);
  Serial.print("\nAveraging mode evaluation (");
  Serial.print(MEASUREMENTS_PER_MODE);
  Serial.println("measurements per mode):");

  for (int i=0; i<NR_AVG_MODES; i++) {
    TempZero.setAveraging(averaging_mode[i]);
    float stat_temp_sum = 0;
    float stat_temp_var_shift;   // Use first sample for shifted variance calc to ensure numeric stability
    float stat_temp_var_Ex = 0;  // Error for shifted variance calc
    float stat_temp_var_Ex2 = 0; // Error squared for shifted variance calc
    stats_avg_mode.temp_max = -1000;
    stats_avg_mode.temp_min = 1000;
    uint32_t stat_time_sum = 0;
    for (int j=0; j<MEASUREMENTS_PER_MODE; j++) {

      uint16_t start = millis();
      temp = TempZero.readInternalTemperature();
      stat_time_sum += (millis() - start);

      if (j==0) {
        stat_temp_var_shift = temp; // we'll use first sample in the shifted variance calc 
      }
      stats_avg_mode.temp_max = temp > stats_avg_mode.temp_max ? temp : stats_avg_mode.temp_max;
      stats_avg_mode.temp_min = temp < stats_avg_mode.temp_min ? temp : stats_avg_mode.temp_min;   
      stat_temp_sum += temp;
      stat_temp_var_Ex += stat_temp_var_shift - temp;
      stat_temp_var_Ex2 += (stat_temp_var_shift - temp) * (stat_temp_var_shift - temp);
    }
    stats_avg_mode.delay_avg = float(stat_time_sum) / MEASUREMENTS_PER_MODE;
    stats_avg_mode.temp_avg = stat_temp_sum / MEASUREMENTS_PER_MODE;
    stats_avg_mode.temp_var = (stat_temp_var_Ex2 - (stat_temp_var_Ex * stat_temp_var_Ex)/MEASUREMENTS_PER_MODE)/(MEASUREMENTS_PER_MODE - 1);

    Serial.print("Mode : ");
    Serial.print(average_mode_desc[i]);
    Serial.print(" - delay : ");
    Serial.print(stats_avg_mode.delay_avg);
    Serial.print(" ms, temp (max-min) / avg / var : ");
    Serial.print(stats_avg_mode.temp_max - stats_avg_mode.temp_min, 1);
    Serial.print(" / ");
    Serial.print(stats_avg_mode.temp_avg, 1);
    Serial.print(" / ");
    Serial.println(stats_avg_mode.temp_var, 4);
  }
}
