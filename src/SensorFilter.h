template <typename T>
class SensorFilter
{
private:
  T stored_val_telem = 0;
  T stored_val_flash = 0;
  uint32_t N_telem = 1;
  uint32_t N_flash = 1;
  bool first_run = true;
public:
  SensorFilter(int sensor_hz, int telem_hz, int flash_hz) {
    if (telem_hz > sensor_hz) {
      N_telem = 1;
      return;
    }
    N_telem = sensor_hz/telem_hz;

    if (flash_hz > sensor_hz) {
      N_flash = 1;
      return;
    }
    N_flash = sensor_hz/flash_hz;
  }

  void update_sensor(T value) {
    if (first_run) {
      stored_val_telem = value;
      stored_val_flash = value;
      first_run = false;
      return;
    }
    //Rolling average formula
    stored_val_telem -= stored_val_telem / N_telem;
    stored_val_telem += value / N_telem;

    stored_val_flash -= stored_val_flash / N_flash;
    stored_val_flash += value / N_flash;
  }

  T output(void) {
    return stored_val_telem; 
  }

  T output_flash(void) {
    return stored_val_flash;
  }
};