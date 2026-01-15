# main_show.py — example usage of the libs
from tts_lib import pre_synth, play_wav_async, warm_piper
from eyes_lib import get_eyes
from camera_lib import get_cam
from cmdvel_drifts import DriftLeft, DriftRight, SpinLeft, Forward, Stop, TinyFidget
from showrunner import ShowRunner
import threading, time
try:
    from robot_moves import horn  # your existing horn()
except Exception:
    def horn():
        print("[horn] (placeholder) install mpg123 and set HORN_FILE env if using robot_moves.horn")

eyes = get_eyes()
cam  = get_cam()
AMBER = (253,208,0)

# warm TTS once after boot
warm_piper("ryan")

# cache the promo line for zero-lag start
PROMO = ("Kia ora! I'm Turbo, I'm a robot from Matamoe. "
         "We're building something big in twenty twenty-six. "
         "Robots, Python, and A I. Stay tuned for more!")
WAV = pre_synth(PROMO, voice="ryan", length_scale="1.02", sentence_silence="0.07")

def speak_with_blink():
    eyes.set_both(*AMBER)
    def mid_blink():
        time.sleep(2.1)
        eyes.blink_big(0.9, open_color=AMBER)
    threading.Thread(target=mid_blink, daemon=True).start()
    threading.Thread(target=lambda: TinyFidget(2.6, amp_v=0.02, amp_w=0.10, step_s=0.22), daemon=True).start()
    threading.Thread(target=lambda: cam.wiggle(cycles=1, amplitude=120), daemon=True).start()
    play_wav_async(WAV)  # zero-lag playback

def perform_show():
    show = ShowRunner()
    show.add(0.0,  lambda: eyes.set_both(*AMBER))
    show.add(0.2,  lambda: cam.shake(amt=240))
    show.add(0.8,  lambda: eyes.blink_big(0.9, open_color=AMBER))
    show.add(2.1,  lambda: SpinLeft(0.8, w=1.4))
    show.add(3.2,  lambda: DriftLeft(0.9))
    show.add(4.3,  lambda: DriftRight(0.9))
    show.add(5.4,  lambda: Forward(0.9, v=0.16))
    show.add(6.5,  speak_with_blink)
    show.add(10.9, lambda: cam.nod(amt=240))
    show.add(11.6, lambda: (horn(),))
    show.add(11.9, lambda: Forward(0.8, v=0.12))
    show.add(12.8, Stop)
    show.run()

if __name__ == "__main__":
    perform_show()
    print("Show complete.")
