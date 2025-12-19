#!/usr/bin/env python3
import pigpio
import curses
import time

pi = pigpio.pi()
motors = [22, 23, 24, 25]

for m in motors:
    pi.set_mode(m, pigpio.OUTPUT)
    pi.set_PWM_frequency(m, 100)

duty_forward = 250

duty_direction = duty_forward + 5

def set_motors(a22, a23, a24, a25):
    pi.set_PWM_dutycycle(22, a22)
    pi.set_PWM_dutycycle(23, a23)
    pi.set_PWM_dutycycle(24, a24)
    pi.set_PWM_dutycycle(25, a25)

def main(stdscr):
    stdscr.nodelay(True)  # không chặn khi đọc phím
    stdscr.clear()
    stdscr.addstr(0, 0, "Điều khiển xe: W-tien, S-lui, A-trai, D-phai, Q-thoat")
    stdscr.addstr(1, 0, "Nhấn giữ phím để chạy, thả ra là dừng.")

    while True:
        key = stdscr.getch()
        if key != -1:
            key_char = chr(key).lower()
            if key_char == 'q':
                break
            elif key_char == 'w':
                set_motors(duty_forward, 0, duty_forward, 0)
                stdscr.addstr(3, 0, "Tiến       ")
            elif key_char == 's':
                set_motors(0, duty_forward, 0, duty_forward)
                stdscr.addstr(3, 0, "Lùi        ")
            elif key_char == 'a':
                set_motors(duty_direction, 0, 0, duty_direction)
                stdscr.addstr(3, 0, "Trái       ")
            elif key_char == 'd':
                set_motors(0, duty_direction, duty_direction, 0)
                stdscr.addstr(3, 0, "Phải       ")
        else:
            # Không có phím nào nhấn => dừng
            set_motors(0, 0, 0, 0)
            stdscr.addstr(3, 0, "Dừng       ")

        stdscr.refresh()
        time.sleep(0.05)

try:
    curses.wrapper(main)
finally:
    set_motors(0, 0, 0, 0)
    pi.stop()

