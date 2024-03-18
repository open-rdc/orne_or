import keyboard
import json
import time

commands = [
    ['1', [1,-1,0,0,0,0], False],
    ['2', [1,0,0,0,0,0], False],
    ['3', [1,1,0,0,0,0], False],
    ['4', [0,-1,0,0,0,0], False],
    ['6', [0,1,0,0,0,0], False],
    ['7', [-1,-1,0,0,0,0], False],
    ['8', [-1,0,0,0,0,0], False],
    ['9', [-1,1,0,0,0,0], False],
    ['-', [0,0,1,0,0,0], False],
    ['+', [0,0,-1,0,0,0], False],
    ['0', [0,0,0,1,0,0], False],
    ['.', [0,0,0,-1,0,0], False],
    ['w', [0,0,0,0,-1,0], False],
    ['q', [0,0,0,0,1,0], False],
    ['r', [0,0,0,0,0,1], False]
]

def on_press(key):
    for i in range(len(commands)):
        if key.name == commands[i][0]:
            commands[i][2] = True

def on_release(key):
    for i in range(len(commands)-3):
        if key.name == commands[i][0]:
            commands[i][2] = False

for key, _, _ in commands:
    keyboard.on_press_key(key, on_press)
    keyboard.on_release_key(key, on_release)

while True:
    sum_command = [0, 0, 0, 0, 0, 0]
    for _, command, is_pressed in commands:
        if is_pressed:
            sum_command = [x + y for x , y in zip(sum_command, command)]
    with open('key_command.json', 'w') as f:
        json.dump({'command': sum_command}, f)
    for i in range(len(commands)-3,len(commands)):
        commands[i][2] = False
    print(sum_command)
    time.sleep(0.1)