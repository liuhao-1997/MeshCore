#include "RotaryInputGPIO.h"

#ifdef THINKNODE_M8
RotaryInputGPIO* RotaryInputGPIO::_instance = nullptr;

void RotaryInputGPIO::onChange() {
  if (_instance) _instance->sample();
}

void RotaryInputGPIO::sample() {
  const uint8_t state = (digitalRead(_pin_a) << 1) | digitalRead(_pin_b);
  if (state == _state) return;
  // Only adjacent Gray-code states are valid. Bounce cancels quarter steps.
  static const int8_t transitions[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };
  if ((state ^ _state) == 3) {
    _quarters = 0;
    _synced = false;
  } else if (_synced) {
    _quarters += transitions[(_state << 2) | state];
  }
  _state = state;
  if (state == 3) {
    // One event per complete cycle, preserving the former B-rising direction.
    if (_synced && _quarters == 4 && _pending < 16) ++_pending;
    if (_synced && _quarters == -4 && _pending > -16) --_pending;
    _quarters = 0;
    _synced = true;
  }
}
#endif

bool RotaryInputGPIO::begin() {
#ifdef THINKNODE_M8
  if (_begun) return true;
#endif

  if (_pin_a >= 0) {
    pinMode(_pin_a, _pull_a ? INPUT_PULLUP : INPUT);
  }
  if (_pin_b >= 0) {
    pinMode(_pin_b, _pull_b ? INPUT_PULLUP : INPUT);
  }

  b_prec = digitalRead(_pin_b);

#ifdef THINKNODE_M8
  // Capture during blocking e-paper refreshes as well as between UI polls.
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  _instance = this;
  const int a_irq = attachInterrupt(digitalPinToInterrupt(_pin_a), onChange, CHANGE);
  const int b_irq = attachInterrupt(digitalPinToInterrupt(_pin_b), onChange, CHANGE);
  _interrupts = a_irq && b_irq;
  if (!_interrupts) {
    if (a_irq) detachInterrupt(digitalPinToInterrupt(_pin_a));
    if (b_irq) detachInterrupt(digitalPinToInterrupt(_pin_b));
  }
  _state = (digitalRead(_pin_a) << 1) | digitalRead(_pin_b);
  _quarters = 0;
  _synced = _state == 3;
  _pending = 0;
  _begun = true;
  __set_PRIMASK(primask);
#endif
  return true;
}

RotaryInputEvent RotaryInputGPIO::poll() {
#ifdef THINKNODE_M8
  if (!_begun) begin();
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (!_interrupts) sample();
  RotaryInputEvent ev = RotaryInputEvent::None;
  if (_pending > 0) { --_pending; ev = RotaryInputEvent::Next; }
  else if (_pending < 0) { ++_pending; ev = RotaryInputEvent::Prev; }
  __set_PRIMASK(primask);
  return ev;
#else
  RotaryInputEvent ev = RotaryInputEvent::None;
  bool a = digitalRead(_pin_a);
  bool b = digitalRead(_pin_b);

  // this is the simplest scheme and it works well
  // for thinknode M8, just read A when B rises ;)
  if (!b_prec && b) { // rising edge of A
    if (a) {
      ev = RotaryInputEvent::Next;
    } else { 
      ev = RotaryInputEvent::Prev;
    }
  }
  b_prec = b;

  return ev;
#endif
}
