#include "lcm_joystick.h"

#include <atomic>
#include <cstring>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "smp_hw_lcm/joy_t.hpp"

namespace sim2sim {
namespace {

// Linux xpad layout, matching what the joystick bridge publishes and what
// XBoxJoystick reads off /dev/input/js0. Indices rather than names because
// that is what sensor_msgs/Joy — which joy_t mirrors — carries.
constexpr int kAxisLx = 0;
constexpr int kAxisLy = 1;
constexpr int kAxisLt = 2;
constexpr int kAxisRx = 3;
constexpr int kAxisRy = 4;
constexpr int kAxisRt = 5;
constexpr int kAxisDpadX = 6;
constexpr int kAxisDpadY = 7;

constexpr int kBtnA = 0;
constexpr int kBtnB = 1;
constexpr int kBtnX = 2;
constexpr int kBtnY = 3;
constexpr int kBtnLb = 4;
constexpr int kBtnRb = 5;
constexpr int kBtnBack = 6;
constexpr int kBtnStart = 7;
constexpr int kBtnLs = 9;
constexpr int kBtnRs = 10;

float Axis(const std::vector<float>& axes, int index) {
  return index < static_cast<int>(axes.size()) ? axes[index] : 0.0f;
}

int Btn(const std::vector<int32_t>& buttons, int index) {
  return index < static_cast<int>(buttons.size()) ? buttons[index] : 0;
}

}  // namespace

struct LcmJoystick::Impl {
  std::unique_ptr<lcm::LCM> lcm;
  std::thread thread;
  std::atomic<bool> stop{false};
  std::mutex mutex;
  std::vector<float> axes;
  std::vector<int32_t> buttons;

  void Handle(const lcm::ReceiveBuffer* rbuf, const std::string&) {
    smp_hw_lcm::joy_t msg;
    if (msg.decode(rbuf->data, 0, rbuf->data_size) < 0) return;
    std::lock_guard<std::mutex> lock(mutex);
    axes = msg.axes;
    buttons = msg.buttons;
  }
};

LcmJoystick::LcmJoystick(const std::string& channel, const std::string& lcm_url)
    : impl_(new Impl) {
  impl_->lcm.reset(lcm_url.empty() ? new lcm::LCM() : new lcm::LCM(lcm_url));
  if (!impl_->lcm->good()) {
    std::cerr << "[lcm_joystick] LCM failed to initialise — the FSM cannot be "
                 "driven from " << channel << "\n";
    impl_->lcm.reset();
    return;
  }
  impl_->lcm->subscribe(channel, &Impl::Handle, impl_.get());
  impl_->thread = std::thread([this] {
    while (!impl_->stop.load(std::memory_order_relaxed)) {
      impl_->lcm->handleTimeout(50);
    }
  });
  ok_ = true;
  std::cout << "[lcm_joystick] driving the FSM from '" << channel << "'\n";
}

LcmJoystick::~LcmJoystick() {
  if (!impl_) return;
  impl_->stop = true;
  if (impl_->thread.joinable()) impl_->thread.join();
}

void LcmJoystick::update() {
  if (!ok_) return;

  std::vector<float> axes;
  std::vector<int32_t> buttons;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    axes = impl_->axes;
    buttons = impl_->buttons;
  }
  if (axes.empty() && buttons.empty()) return;
  received_.fetch_add(1, std::memory_order_relaxed);

  A(Btn(buttons, kBtnA));
  B(Btn(buttons, kBtnB));
  X(Btn(buttons, kBtnX));
  Y(Btn(buttons, kBtnY));
  LB(Btn(buttons, kBtnLb));
  RB(Btn(buttons, kBtnRb));
  back(Btn(buttons, kBtnBack));
  start(Btn(buttons, kBtnStart));
  LS(Btn(buttons, kBtnLs));
  RS(Btn(buttons, kBtnRs));

  // The D-pad is a hat axis, not four buttons: -1/+1 on axis 6 and 7. Up is
  // negative on the Linux hat, which is why the comparison looks inverted.
  up(Axis(axes, kAxisDpadY) < -0.5f);
  down(Axis(axes, kAxisDpadY) > 0.5f);
  left(Axis(axes, kAxisDpadX) < -0.5f);
  right(Axis(axes, kAxisDpadX) > 0.5f);

  // Triggers rest at -1 and travel to +1 on a real xpad, but a bridge may
  // normalise them to [0, 1]. `Axis::operator()` thresholds internally, and
  // both conventions cross that threshold only when actually pulled, so
  // passing the raw value through works for either.
  LT(Axis(axes, kAxisLt));
  RT(Axis(axes, kAxisRt));

  lx(Axis(axes, kAxisLx));
  ly(-Axis(axes, kAxisLy));  // Linux reports forward as negative
  rx(Axis(axes, kAxisRx));
  ry(-Axis(axes, kAxisRy));
}

}  // namespace sim2sim
