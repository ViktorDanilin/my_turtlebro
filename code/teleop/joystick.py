import pygame as pg
import time
pg.joystick.init()
joystick_count = pg.joystick.get_count()

if joystick_count == 0:
    print("ERROR!")
    exit()
while True:
    joystick = pg.joystick.Joystick(0)
    joystick.init()

    axes = joystick.get_numaxes()
    axis = joystick.get_axis(0)
    buttons = joystick.get_numbuttons()
    button = joystick.get_button(1)
    hats = joystick.get_numhats()
    hat = joystick.get_hat(0)
    print(axis)
    time.sleep(0.2)






















