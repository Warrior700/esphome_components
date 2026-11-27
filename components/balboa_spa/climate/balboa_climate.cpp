#include "esphome/components/balboa_spa/climate/balboa_climate.h"

namespace esphome
{
  namespace balboa_spa
  {
    static const char *TAG = "balboa_spa.climate";

    void BalboaClimate::setup()
    {
      this->parent_->register_sensor_callback(40, [this](const float value) {
        this->target_temperature = value;
        this->publish_state();
      });
      this->parent_->register_sensor_callback(41, [this](const float value) {
        this->current_temperature = value;
        this->publish_state();
      });
      this->mode = climate::CLIMATE_MODE_HEAT;
      this->publish_state();
    }

    void BalboaClimate::control(const climate::ClimateCall &call)
    {
      if (call.get_target_temperature().has_value())
      {
        float temp = *call.get_target_temperature();
        this->parent_->on_set_temp(temp);
      }
    }

    climate::ClimateTraits BalboaClimate::traits()
    {
      auto traits = climate::ClimateTraits();
      traits.set_supports_current_temperature(true);
      traits.set_supported_modes({climate::CLIMATE_MODE_HEAT});
      traits.set_visual_min_temperature(10.0);
      traits.set_visual_max_temperature(40.0);
      traits.set_visual_temperature_step(0.5);
      return traits;
    }
  }
}
