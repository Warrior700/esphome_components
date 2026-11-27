#include "esphome/components/balboa_spa/button/balboa_button.h"

namespace esphome
{
    namespace balboa_spa
    {
        static const char *TAG = "balboa_spa.button";

        void BalboaButton::press_action()
        {
            switch (this->button_id)
            {
            case 0:
            case 6:
                this->parent_->on_toggle_light();
                break;
            case 1:
                this->parent_->on_toggle_jet1();
                break;
            case 2:
                this->parent_->on_toggle_jet2();
                break;
            case 3:
                this->parent_->on_toggle_blower();
                break;
            case 4:
                this->parent_->on_toggle_heatingmode();
                break;
            case 5:
                this->parent_->on_toggle_range();
                break;
            }
        }
    }
}
