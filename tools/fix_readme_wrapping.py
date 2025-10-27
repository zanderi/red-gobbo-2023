import re
from pathlib import Path
p=Path('README.md')
s=p.read_text(encoding='utf-8')

# Replace Notes block between 'Notes:' and 'Example (physical signal flow for LED1):'
s = re.sub(r"Notes:\n[\s\S]*?Example \(physical signal flow for LED1\):\n\n[\s\S]*?\n\n## Red-gobbo:",
           "Notes:\n- ULN2803 is a low-side driver: connect each LED cathode to the corresponding ULN OUTx.\n  The LED anode goes to +V through a suitable resistor (220-330Ω typical).\n\n- Tie ULN GND to ESP32 GND. The ULN COM pin is only required for inductive loads; for LEDs it can usually be\n  left unconnected.\n\n- Ensure a common ground between ESP32 and the LED supply. If LEDs use a higher voltage than the ESP's Vcc,\n  supply that voltage to the LED anodes and keep ULN ground common.\n\nExample (physical signal flow for LED1):\n\n    +V --- resistor ---+--> LED anode\n                        |\n                      LED cathode\n                        |\n                    ULN OUT1 -> ULN GND -> common GND\n\n## Red-gobbo:",
           s, flags=re.M)

# Shorten long one-line project description paragraph
s = s.replace("This project contains firmware to drive a small set of LEDs using an ESP32-C3 and a ULN2803 Darlington sink driver. The lighting code was refactored into a small library `lib/LightEffects` so `src/main.cpp` stays compact.",
              "This project contains firmware to drive a small set of LEDs using an ESP32-C3 and a ULN2803\nDarlington sink driver. The lighting code was refactored into a small library `lib/LightEffects` so\n`src/main.cpp` stays compact.")

s = s.replace("- FIRE and FUSE single-LED effects remain in `src/main.cpp` using LEDC hardware PWM (for lower CPU usage and smoother flicker)",
              "- FIRE and FUSE single-LED effects remain in `src/main.cpp` using LEDC hardware PWM.\n  This reduces CPU usage and provides smoother flicker for those two LEDs.")

s = s.replace("- LightEffects(const int pins[6]) — constructor; pins must be a 6-element array with the GPIO numbers for the\n ULN2803 inputs.",
              "- LightEffects(const int pins[6]) — constructor; pins must be a 6-element array with the GPIO numbers for the\n  ULN2803 inputs.")

s = s.replace("- void setModeDuration(unsigned long ms) — optional runtime override for how long each Christmas mode runs (default 15000 ms).",
              "- void setModeDuration(unsigned long ms) — optional runtime override for how long each Christmas mode runs\n  (default 15000 ms).")

s = s.replace("- Software PWM: the 6 Christmas outputs use an ISR-driven software PWM (64-step) to avoid exhausting LEDC channels and keep FIRE/FUSE on LEDC.",
              "- Software PWM: the 6 Christmas outputs use an ISR-driven software PWM (64-step) to avoid exhausting LEDC\n  channels and keep FIRE/FUSE on LEDC.")

s = s.replace("- If you want a runtime UI to change mode duration or force a mode, I can add a simple serial command parser (tiny menu) or expose a button input to cycle modes.",
              "- If you want a runtime UI to change mode duration or force a mode, I can add a simple serial command parser\n  (tiny menu) or expose a button input to cycle modes.")

# Remove trailing spaces at ends of lines
s = '\n'.join([line.rstrip() for line in s.splitlines()]) + '\n'

p.write_text(s, encoding='utf-8')
print('README wrapped and cleaned')
