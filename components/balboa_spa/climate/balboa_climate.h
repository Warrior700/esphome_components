#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/balboa_spa/balboa_spa.h"

namespace esphome
{
  namespace balboa_spa
  {
    class BalboaClimate : public Component, public climate::Climate
    {
    public:
      void setup() override;
      void control(const climate::ClimateCall &call) override;
      climate::ClimateTraits traits() override;
      void set_balboa_parent(BalboaSpa *parent) { this->parent_ = parent; }

    protected:
      BalboaSpa *parent_{nullptr};
      bool is_high_range_{true};
    };
  }
}
