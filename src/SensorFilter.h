template <typename T>
class SensorFilter
{
private:
  T stored_val_telem = 0;
  T stored_val_flash = 0;
  T unfiltered_value = 0;
  uint32_t N_telem = 1;
  uint32_t N_flash = 1;
  bool first_run = true;
  bool enable = true;
public:
  SensorFilter(int sensor_hz, int telem_hz, int flash_hz, bool is_active=true) {
    if (telem_hz > sensor_hz) {
      N_telem = 1;
      return;
    }
    N_telem = sensor_hz/(2*telem_hz);

    if (flash_hz > sensor_hz) {
      N_flash = 1;
      return;
    }
    N_flash = sensor_hz/(2*flash_hz);
  }

  void update_sensor(T value) {
    unfiltered_value = value;
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
    if (enable) {
      return stored_val_telem; 
    }
    return unfiltered_value;
  }

  T output_flash(void) {
    if (enable) {
      return stored_val_flash;
    }
    return unfiltered_value;
  }

  void disable(void) {
    enable = false;
  }
};