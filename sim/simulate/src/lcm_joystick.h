#pragma once

// A gamepad fed from the JOYSTICK LCM channel instead of /dev/input.
//
// The controller's FSM is driven entirely by button combinations — the G1
// reaches its policy states via chords like `RT + X.on_pressed`, and there is
// no other way in. That is fine on hardware, where somebody is holding the
// pad, and fatal for automated sim2sim, where nobody is.
//
// The sim already accepts any `UnitreeJoystick`, packs it into `LowState`, and
// the controller reads it from there. So this is a third implementation
// alongside XBoxJoystick and SwitchJoystick, differing only in where the
// button states come from: `joy_t` on LCM rather than a kernel device.
//
// Two things fall out of that:
//
//   * A real pad still works — smp_hw's `scripts/joy_bridge.py` puts
//     /dev/input/js0 on this channel, so a physical gamepad drives the sim
//     from another machine, or through an SSH tunnel.
//   * A script can drive an FSM transition by publishing three messages
//     (smp_hw's `scripts/sim_fsm.py`).
//
// The axis and button indices follow the Linux xpad layout, which is what the
// bridge publishes and what XBoxJoystick assumes — so a real pad behaves
// identically whether it reaches the sim through this path or the direct one.

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/dds_wrapper/common/unitree_joystick.hpp>

namespace sim2sim {

class LcmJoystick : public unitree::common::UnitreeJoystick {
 public:
  // `channel` is the LCM channel carrying joy_t (JOYSTICK by convention).
  LcmJoystick(const std::string& channel, const std::string& lcm_url);
  // Not `override`: the SDK base declares no virtual destructor. It is still
  // run — the bridge holds a shared_ptr built from the concrete type, and
  // shared_ptr type-erases the deleter — so the receive thread does get joined.
  ~LcmJoystick();

  // Called by the bridge every cycle. Copies whatever the receive thread last
  // saw into the button/axis state — no decoding here, same discipline as
  // every other LCM consumer here.
  void update() override;

  bool ok() const { return ok_; }
  // Messages received since start; a zero here after a scripted press is the
  // difference between "the FSM ignored it" and "it never arrived".
  long received() const { return received_.load(std::memory_order_relaxed); }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::atomic<long> received_{0};
  bool ok_ = false;
};

}  // namespace sim2sim
