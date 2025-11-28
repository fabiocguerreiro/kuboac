#pragma once

#include "esphome/core/automation.h"

namespace esphome {
namespace tclac {

class tclacClimate;  // forward declaration

template<typename... Ts> class VerticalAirflowAction : public Action<Ts...> {
 public:
  VerticalAirflowAction(tclacClimate *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(AirflowVerticalDirection, direction)
  void play(Ts... x) { this->parent_->set_vertical_airflow(this->direction_.value(x...)); }

 protected:
  tclacClimate *parent_;
};

template<typename... Ts> class VerticalSwingDirectionAction : public Action<Ts...> {
 public:
  VerticalSwingDirectionAction(tclacClimate *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(VerticalSwingDirection, direction)
  void play(Ts... x) { this->parent_->set_vertical_swing_direction(this->direction_.value(x...)); }

 protected:
  tclacClimate *parent_;
};

template<typename... Ts> class DisplayOnAction : public Action<Ts...> {
 public:
  DisplayOnAction(tclacClimate *parent) : parent_(parent) {}
  void play(Ts... x) { this->parent_->set_display_state(true); }

 protected:
  tclacClimate *parent_;
};

template<typename... Ts> class DisplayOffAction : public Action<Ts...> {
 public:
  DisplayOffAction(tclacClimate *parent) : parent_(parent) {}
  void play(Ts... x) { this->parent_->set_display_state(false); }

 protected:
  tclacClimate *parent_;
};

template<typename... Ts> class BeeperOnAction : public Action<Ts...> {
 public:
  BeeperOnAction(tclacClimate *parent) : parent_(parent) {}
  void play(Ts... x) { this->parent_->set_beeper_state(true); }

 protected:
  tclacClimate *parent_;
};

template<typename... Ts> class BeeperOffAction : public Action<Ts...> {
 public:
  BeeperOffAction(tclacClimate *parent) : parent_(parent) {}
  void play(Ts... x) { this->parent_->set_beeper_state(false); }

 protected:
  tclacClimate *parent_;
};

}  // namespace tclac
}  // namespace esphome
