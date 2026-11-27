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
      this->parent_->register_binary_sensor_callback(29, [this](const bool high_range) {
        if (this->is_high_range_ != high_range) {
          this->is_high_range_ = high_range;
          this->publish_state();
        }
      });
      this->mode = climate::CLIMATE_MODE_HEAT;
      this->publish_state();
    }

    void BalboaClimate::control(const climate::ClimateCall &call)
    {
      if (call.get_target_temperature().has_value())
      {
        float temp = *call.get_target_temperature();
        if (this->is_high_range_) {
          if (temp < 26.0f) temp = 26.0f;
          if (temp > 40.0f) temp = 40.0f;
        } else {
          if (temp < 10.0f) temp = 10.0f;
          if (temp > 37.0f) temp = 37.0f;
        }
        this->parent_->on_set_temp(temp);
      }
    }

    climate::ClimateTraits BalboaClimate::traits()
    {
      auto traits = climate::ClimateTraits();
      traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
      traits.set_supported_modes({climate::CLIMATE_MODE_HEAT});
      if (this->is_high_range_) {
        traits.set_visual_min_temperature(26.0);
        traits.set_visual_max_temperature(40.0);
      } else {
        traits.set_visual_min_temperature(10.0);
        traits.set_visual_max_temperature(37.0);
      }
      traits.set_visual_temperature_step(0.5);
      return traits;
    }
  }
}
