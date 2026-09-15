#pragma once

namespace tap_sim
{
// Caller serializes access. Time is monotonic wall seconds, independent of /clock.
class ActuatorGate
{
  bool stopped_ = false, armed_ = false, enabled_ = false;
  double received_ = -1;
  void expire(double now)
  {
    if (enabled_ && (received_ < 0 || now - received_ > 0.1)) {
      enabled_ = false; armed_ = false;
    }
  }
public:
  void disarm() {armed_ = false; enabled_ = false;}
  void stop(bool value)
  {
    stopped_ = value;
    if (value) {armed_ = false; enabled_ = false;}
  }
  void receive(bool value, double now)
  {
    // Check the previous sample before a resumed callback renews its timestamp.
    expire(now);
    if (stopped_) {return;}
    if (!value) {armed_ = true; enabled_ = false;}
    else if (armed_) {enabled_ = true;}
    received_ = now;
  }
  bool enabled(double now)
  {
    expire(now);
    return !stopped_ && enabled_;
  }
};
}  // namespace tap_sim
