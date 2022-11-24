#include <stdint.h>

class PeriodicTimer {
public:
  typedef void (*TimerCallback)(uint32_t);

  PeriodicTimer(
    uint32_t interval, const TimerCallback cb)
    : interval_(interval), cb_(cb) {
  }

  void Start(uint32_t time) {
    last_ = time;
  }

  void Check(uint32_t time) {
    if (time - last_ >= interval_) {
      if (cb_) {
        cb_(time);
      }
      last_ = time;
    }
  }
private:
  uint32_t last_;
  const uint32_t interval_;
  const TimerCallback cb_;
};

template<int N>
void StartAllTimers(PeriodicTimer (&timers)[N], uint32_t time) {
  for(auto& timer : timers) {
    timer.Start(time);
  }
}

template<int N>
void CheckAllTimers(PeriodicTimer (&timers)[N], uint32_t time) {
  for(auto& timer : timers) {
    timer.Check(time);
  }
}
