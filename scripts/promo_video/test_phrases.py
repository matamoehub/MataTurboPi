"""
PHRASE TEST — check pronunciation before filming
Run this on either robot to hear all spoken lines.
The script prints label + text before each phrase so you can see
exactly which version is playing and note what needs changing.

ADJUSTING MĀORI PRONUNCIATION:
  Edit the phonetic spellings below until they sound right.
  Piper TTS is phonetic — changing the spelling changes the sound.
  e.g.  "Key ora"  vs  "Kia ora"  vs  "Kee-ah or-ah"
"""
from lesson_header import *
import time

myRobot = bot(base_speed=300)
myRobot.voice.set_volume(90)

def say(label, text, voice="amy"):
    myRobot.voice.select(voice)
    print(f"\n  [{voice.upper()}] {label}")
    print(f"         \"{text}\"")
    myRobot.voice.say(text, block=True)
    time.sleep(0.6)

# ════════════════════════════════════════════════════════════════════════════
print("\n══════════════════════════════════════════")
print("  MĀORI PRONUNCIATION — adjust these first")
print("══════════════════════════════════════════")

# Try each version — pick the one that sounds best and copy it to the scripts
say("Kia ora — attempt 1 (phonetic)", "Key ora")
say("Kia ora — attempt 2 (native spelling)", "Kia ora")
say("Kia ora — attempt 3 (split)", "Kee-ah or-ah")

say("Poneke — attempt 1 (phonetic)", "Po Knee key")
say("Poneke — attempt 2 (joined)", "Ponehkeh")
say("Poneke — attempt 3 (native spelling)", "Poneke")

say("Matamoe — attempt 1", "Mata moe E")
say("Matamoe — attempt 2 (native)", "Matamoe")

# ════════════════════════════════════════════════════════════════════════════
print("\n══════════════════════════════════════════")
print("  AMY (Toni) — ALL LINES")
print("══════════════════════════════════════════")

say("intro",          "Key ora. I am Toni.")
say("league name",    "Welcome to the Po Knee key AI Robot League.")
say("programme",      "A free after-school programme for Wellington secondary school students,")
say("company",        "run by Mata moe E, based at Scots College.")
say("explore",        "In this programme, you will explore robotics, coding, and artificial intelligence.")
say("can move",       "Our robots can move,")
say("objects",        "recognise objects,")
say("colours",        "and detect colours.")
say("welcome",        "No experience needed. Everyone is welcome.")
say("now",            "Now.")
say("drift cue",      "You may drift.")

# ════════════════════════════════════════════════════════════════════════════
print("\n══════════════════════════════════════════")
print("  RYAN (Turbo) — ALL LINES")
print("══════════════════════════════════════════")

say("intro",          "And I am Turbo!",                              voice="ryan")
say("drift?",         "Can I drift yet?",                             voice="ryan")
say("curiosity",      "You just bring the curiosity!",                voice="ryan")
say("sign off",       "Now that is how you drift!",                   voice="ryan")

# ════════════════════════════════════════════════════════════════════════════
print("\n══════════════════════════════════════════")
print("  AMY — OUTTAKE LINES")
print("══════════════════════════════════════════")

say("flub",           "Key ora. I am... I am...")
say("not a word",     "Not. A. Word.")
say("not now",        "Not now.")
say("every time",     "Every single time.")
say("snoring?",       "Are you snoring?")
say("unbelievable",   "Unbelievable.")
say("hypno eyes",     "Not the hypno eyes!")
say("thank you",      "Thank you.")
say("too bright",     "Too bright.")
say("yes you",        "Yes. You.")
say("turbo call 1",   "Turbo.")
say("saw a ball",     "It saw a ball.")
say("you go",         "You go.")
say("start again",    "Start again.")

# ════════════════════════════════════════════════════════════════════════════
print("\n══════════════════════════════════════════")
print("  RYAN — OUTTAKE LINES")
print("══════════════════════════════════════════")

say("drift time",     "I thought it was drift time!",                 voice="ryan")
say("who me",         "Who, me?",                                     voice="ryan")
say("cut off 1",      "And I am Tur—",                                voice="ryan")
say("cut off 2",      "And I am—",                                    voice="ryan")
say("full intro",     "And I am Turbo!",                              voice="ryan")
say("cut off 3",      "And I—",                                       voice="ryan")

# ════════════════════════════════════════════════════════════════════════════
print("\n══════════════════════════════════════════")
print("  DONE")
print("  Edit the phonetic spellings above for")
print("  any phrase that sounded wrong, then")
print("  copy the winner into the video scripts.")
print("══════════════════════════════════════════\n")

myRobot.eyes.off()
myRobot.camera.center()
