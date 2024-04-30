template <typename T>
class SensorFilter
{
private:
  T stored_val = 0;
  uint32_t count = 0;
  uint32_t N = 1;
  bool first_run = true;
public:
  SensorFilter(int sensor_hz, int telem_hz) {
    if (telem_hz > sensor_hz) {
      N = 1;
      return;
    }
    N = sensor_hz/telem_hz;
  }

  void update_sensor(T value) {
    if (first_run) {
      stored_val = value;
      first_run = false;
      return;
    }
    //Rolling average formula
    stored_val -= stored_val / N;
    stored_val += value / N;
  }

  T output(void) {
    return stored_val; 
  }
};