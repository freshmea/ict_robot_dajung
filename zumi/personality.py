'''
from zumi import Zumi
from screen import Screen
from threading import Thread
from protocol import Note
from time import sleep
'''
from zumi.zumi import Zumi
from zumi.util.screen import Screen
from zumi.protocol import Note
from time import sleep
from threading import Thread
from socket import gethostname


class Personality:
    def __init__(self, _zumi, _screen):

        self.movement = Movement(_zumi)
        self.screen = _screen
        self.sound = Sound(_zumi)

    def start_threading(self, *args):
        for thread in args:
            thread.start()
        for thread in args:
            thread.join()

    def happy(self):
        screen_thread = Thread(target=self.screen.happy)
        sound_thread = Thread(target=self.sound.happy)
        move_thread = Thread(target=self.movement.happy)
        self.start_threading(screen_thread, sound_thread, move_thread)

    def celebrate(self):
        screen_thread = Thread(target=self.screen.happy)
        sound_thread = Thread(target=self.sound.celebrate)
        move_thread = Thread(target=self.movement.celebrate)
        self.start_threading(screen_thread, sound_thread, move_thread)

    def angry(self):
        screen_thread = Thread(target=self.screen.angry)
        sound_thread = Thread(target=self.sound.angry)
        self.start_threading(sound_thread, screen_thread)

    def look_around(self):
        # looks around with half opened eyes
        sound_thread = Thread(target=self.sound.look_around)
        screen_thread = Thread(target=self.screen.look_around)
        move_thread = Thread(target=self.movement.look_around)
        self.start_threading(sound_thread, screen_thread, move_thread)

    def snoring(self):
        screen_thread = Thread(target=self.screen.sleeping)
        sound_thread = Thread(target=self.sound.snoring)
        self.start_threading(sound_thread, screen_thread)

    def look_around_open(self):
        # looks around with open eyes
        sound_thread = Thread(target=self.sound.look_around)
        screen_thread = Thread(target=self.screen.look_around_open)
        move_thread = Thread(target=self.movement.look_around)
        self.start_threading(sound_thread, screen_thread, move_thread)

    def disoriented(self):
        sound_thread = Thread(target=self.sound.disoriented_1)
        screen_thread = Thread(target=self.screen.draw_image_by_name("disoriented"))
        self.start_threading(sound_thread, screen_thread)
        
    def disoriented_left(self):
        sound_thread = Thread(target=self.sound.disoriented_1)
        screen_thread = Thread(target=self.screen.draw_image_by_name("lookleft2"))
        self.start_threading(sound_thread, screen_thread)

    def disoriented_right(self):
        sound_thread = Thread(target=self.sound.disoriented_1)
        screen_thread = Thread(target=self.screen.draw_image_by_name("lookright2"))
        self.start_threading(sound_thread, screen_thread)

    def connected(self):
        sound_thread = Thread(target=self.sound.happy)
        screen_thread = Thread(target=self.screen.draw_image_by_name("connected"))
        move_thread = Thread(target=self.movement.happy)
        self.start_threading(sound_thread, screen_thread, move_thread)

    def online_fail(self):
        sound_thread = Thread(target=self.sound.uh_oh)
        screen_thread = Thread(target=self.screen.draw_image_by_name("onlinefail"))
        self.start_threading(sound_thread, screen_thread)

    def awake(self):
        sound_thread = Thread(target=self.sound.wake_up)
        sleep(0.8)
        screen_thread = Thread(target=self.screen.hello)
        sound_thread.start()
        screen_thread.start()
        sound_thread.join()
        screen_thread.join()

    def start(self):
        host = gethostname()
        sound_thread = Thread(target=self.sound.happy)
        screen_thread = Thread(target=self.screen.draw_text_center("Find \""+host+"\" in your WiFi list"))
        self.start_threading(sound_thread, screen_thread)
        sleep(3)

    def found_me(self):
        # You found me!
        sound_thread = Thread(target=self.sound.celebrate)
        screen_thread = Thread(target=self.screen.draw_image_by_name("foundme"))
        self.start_threading(sound_thread, screen_thread)
        sleep(2)
        # Connect to Zumi.local in browser
        host = gethostname()
        self.screen.draw_text_center("Go to \"zumidashboard.ai\" in your browser")
        sleep(1)

    def attempt_connection(self):
        # Trying to connect
        sound_thread = Thread(target=self.sound.calibrating)
        screen_thread = Thread(target=self.screen.draw_image_by_name("tryingtoconnect"))
        self.start_threading(sound_thread, screen_thread)

    def connected_wifi(self, wifi):
        # Connected
        sound_thread = Thread(target=self.sound.calibrated)
        screen_thread = Thread(target=self.screen.draw_text_center("I'm connected to \""+wifi+"\""))
        self.start_threading(sound_thread, screen_thread)
        sleep(3)
        self.awake()

    def offline_mode(self):
        self.screen.draw_text_center("Starting offline mode")
        sleep(3)
        self.awake()


class Movement:
    def __init__(self, _zumi):
        self.zumi = _zumi

    def celebrate(self):
        sleep(0.5)
        init_angle = self.zumi.read_z_angle()
        self.zumi.turn(init_angle+360, 2.5, accuracy=10)

    def look_around(self):
        sleep(2.5)
        init_angle = self.zumi.read_z_angle()
        self.zumi.turn(init_angle-20, 1, accuracy=10)
        sleep(.61)
        self.zumi.turn(init_angle+20, 1, accuracy=10)
        sleep(.48)
        self.zumi.turn(init_angle, 1, accuracy=10)
        self.zumi.stop()

    def happy(self):
        sleep(1)
        for i in range(2):
            self.zumi.control_motors(5, -5)
            sleep(0.1)
            self.zumi.stop()
            self.zumi.control_motors(-5, 5)
            sleep(0.1)
            self.zumi.stop()


class Sound:
    def __init__(self, _zumi):
        self.zumi = _zumi

    def blink(self):
        self.zumi.play_note(49, 20)
        self.zumi.play_note(50, 20)
        self.zumi.play_note(51, 20)
        self.zumi.play_note(0, 0)

    def look_around(self):
        sleep(2.7)
        self.blink()
        sleep(1.4)
        self.blink()
        sleep(1.4)
        self.blink()

    def celebrate(self):
        sleep(.25)
        self.zumi.play_note(48, 100)
        sleep(.005)
        self.zumi.play_note(52, 100)
        sleep(.005)
        self.zumi.play_note(55, 100)
        sleep(.005)
        self.zumi.play_note(55, 100)
        sleep(.20)
        self.zumi.play_note(52, 125)
        sleep(.005)
        self.zumi.play_note(55, 125)
        self.zumi.play_note(0, 0)

    def wake_up(self):
        note = 36
        tempo = 100
        self.zumi.play_note(note, tempo + 20)
        self.zumi.play_note(note, tempo)
        sleep(0.10)
        self.zumi.play_note(note + 5, tempo)
        sleep(0.1)
        self.zumi.play_note(note + 5, tempo)
        sleep(0.05)
        self.zumi.play_note(note + 7, tempo)
        sleep(0.15)
        self.zumi.play_note(note + 12, tempo)
        self.zumi.play_note(0, 0)

    def disoriented_1(self):
        tempo = 75
        for note in range(0, 1):
            for count in range(0, 2):
                self.zumi.play_note(46, tempo)
                self.zumi.play_note(45, tempo)
                self.zumi.play_note(44, tempo)
                self.zumi.play_note(45, tempo)
            sleep(0.5)
        self.zumi.play_note(0, 0)

    def disoriented_2(self):
        tempo = 60
        for start_note in range(35, 39):
            for i in range(0, 1):  # one or two times?
                self.zumi.play_note(start_note, tempo)
                self.zumi.play_note(start_note, tempo)
                self.zumi.play_note(start_note + 7, tempo)
                self.zumi.play_note(start_note + 7, tempo)
                sleep(.01)
        self.zumi.play_note(0, 0)

    def oops_front(self):
        tempo = 50
        self.zumi.play_note(49, tempo)
        sleep(.02)
        self.zumi.play_note(46, tempo)
        sleep(0.02)
        self.zumi.play_note(45, tempo)
        self.zumi.play_note(0, 0)

    def oops_back(self):
        tempo = 50
        self.zumi.play_note(45, tempo)
        sleep(.02)
        self.zumi.play_note(46, tempo)
        sleep(0.02)
        self.zumi.play_note(49, tempo)
        self.zumi.play_note(0, 0)

    def uh_oh(self):
        tempo = 100
        self.zumi.play_note(Note.B5, tempo)
        self.zumi.play_note(Note.AS5, tempo)
        self.zumi.play_note(Note.A5, tempo)
        self.zumi.play_note(Note.GS5, tempo)
        self.zumi.play_note(0, 0)

    def uh_oh2(self):
        tempo = 100
        self.zumi.play_note(Note.A5, tempo)
        self.zumi.play_note(Note.F5, 2 * tempo)
        self.zumi.play_note(0, 0)

    def angry(self):
        tempo = 80
        for i in range(2):
            self.zumi.play_note(Note.B4, tempo)
            self.zumi.play_note(Note.AS4, tempo)
        self.zumi.play_note(0, 0)

    def calibrating(self):
        for i in range(6):
            tempo = 75
            self.zumi.play_note(Note.G5, tempo)
            sleep(0.1)
            self.zumi.play_note(Note.G5, tempo)
            sleep(0.1)
            self.zumi.play_note(Note.D6, tempo)
            sleep(0.1)
            self.zumi.play_note(Note.D6, tempo)
            sleep(1.5)
        self.zumi.play_note(0, 0)

    def calibrated(self):
        tempo = 60
        self.zumi.play_note(41, tempo)
        self.zumi.play_note(43, tempo)
        self.zumi.play_note(45, tempo)
        self.zumi.play_note(0, 0)

    def happy(self):
        sleep(1)
        tempo = 75
        for i in range(1):
            self.zumi.play_note(Note.C5, tempo)
            self.zumi.play_note(Note.G5, tempo)
            self.zumi.play_note(Note.E5, tempo)
            sleep(0.5)
        self.zumi.play_note(0, 0)

    def snoring(self):
        tempo = 80
        self.zumi.play_note(Note.C4, tempo)
        self.zumi.play_note(Note.E4, tempo)
        self.zumi.play_note(Note.G4, tempo)
        self.zumi.play_note(Note.C5, tempo)
        sleep(0.5)
        self.zumi.play_note(Note.C5, tempo)
        self.zumi.play_note(Note.G4, tempo)
        self.zumi.play_note(Note.E4, tempo)
        self.zumi.play_note(Note.C4, tempo)
        self.zumi.play_note(0, 0)


def connecting_sample(personality):
    personality.start()
    personality.found_me()
    personality.attempt_connection()
    personality.connected()
    personality.offline_mode()


if __name__ == '__main__':
    zumi = Zumi()
    screen = Screen()
    p = Personality(zumi, screen)
    connecting_sample(p)
