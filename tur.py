#!/usr/bin/env python
# manual

"""
This script allows you to manually control the simulator or Duckiebot
using the keyboard arrows.
"""
import argparse
import sys
import gym
import numpy as np
import pyglet
from pyglet.window import key
import turtle
from gym_duckietown.envs import DuckietownEnv



parser = argparse.ArgumentParser()
parser.add_argument("--env-name", default=None)
parser.add_argument("--map-name", default="udem1")
parser.add_argument("--distortion", default=False, action="store_true")
parser.add_argument("--camera_rand", default=False, action="store_true")
parser.add_argument("--draw-curve", action="store_true", help="draw the lane following curve")
parser.add_argument("--draw-bbox", action="store_true", help="draw collision detection bounding boxes")
parser.add_argument("--domain-rand", action="store_true", help="enable domain randomization")
parser.add_argument("--dynamics_rand", action="store_true", help="enable dynamics randomization")
parser.add_argument("--frame-skip", default=1, type=int, help="number of frames to skip")
parser.add_argument("--seed", default=1, type=int, help="seed")
args = parser.parse_args()

if args.env_name and args.env_name.find("Duckietown") != -1:
    env = DuckietownEnv(
        seed=args.seed,
        map_name=args.map_name,
        draw_curve=args.draw_curve,
        draw_bbox=args.draw_bbox,
        domain_rand=args.domain_rand,
        frame_skip=args.frame_skip,
        distortion=args.distortion,
        camera_rand=args.camera_rand,
        dynamics_rand=args.dynamics_rand,
    )
else:
    env = gym.make(args.env_name)

env.reset()
env.render()


@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
    """
    This handler processes keyboard commands that
    control the simulation
    """
    action = np.array([0.0, 0.0])
    if symbol == key.BACKSPACE or symbol == key.SLASH:
        print("RESET")
        env.reset()
        env.render()
    elif symbol == key.PAGEUP:
        env.unwrapped.cam_angle[0] = 0
    elif symbol == key.ESCAPE:
        env.close()
        sys.exit(0)


# Register a keyboard handler
key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)


class Map:
    # draw a window for the game
    screen = turtle.Screen()
    screen.title('Map')
    screen.bgcolor('cyan')
    screen.setup(650, 650, 720, 50)
    screen.tracer(0)

    # draw a game field border
    border = turtle.Turtle()
    border.hideturtle()
    border.penup()
    border.goto(-311, 311)
    border.pendown()
    border.goto(311, 311)
    border.goto(311, -311)
    border.goto(-311, -311)
    border.goto(-311, 311)

    snake = turtle.Turtle()
    snake.left(90)
    snake.pendown()
    snake.right(2.9)


alfa_0 = env.cur_angle * 57.30228433094796


def update(dt):
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """
    global alfa_0
    wheel_distance = 0.102
    min_rad = 0.08
    action = np.array([0.0, 0.0])
    alfa_1 = env.cur_angle * 57.30228433094796

    angle = alfa_1 - alfa_0

    if key_handler[key.UP]:
        action += np.array([0.44, 0.0])
        Map.snake.forward(1)

    if key_handler[key.DOWN]:
        action -= np.array([0.44, 0])
        Map.snake.back(1)

    if key_handler[key.LEFT]:
        action += np.array([0, 1])
        Map.snake.left(0.8509249070808)
        print(alfa_0, '||||||||', alfa_1, '||||||||', angle)

    if key_handler[key.RIGHT]:
        action -= np.array([0, 1])
        Map.snake.right(0.8509249070808)
        print(alfa_0, '||||||||', alfa_1, '||||||||', angle)

    if key_handler[key.SPACE]:
        action = np.array([0, 0])

    v1 = action[0]
    v2 = action[1]
    # Limit radius of curvature
    # if v1 == 0 or abs(v2 / v1) > (min_rad + wheel_distance / 2.0) / (min_rad - wheel_distance / 2.0):
    #     # adjust velocities evenly such that condition is fulfilled
    #     delta_v = (v2 - v1) / 2 - wheel_distance / (4 * min_rad) * (v1 + v2)
    #     v1 += delta_v
    #     v2 -= delta_v

    action[0] = v1
    action[1] = v2

    # Speed boost
    if key_handler[key.LSHIFT]:
        action *= 1.5

    alfa_0 = env.cur_angle * 57.30228433094796

    obs, reward, done, info = env.step(action)
    # print("step_count = %s, reward=%.3f" % (env.unwrapped.step_count, reward))
    # print(env.cur_angle * 57.30228433094796)

    if done:
        print("done!")
        env.reset()
        env.render()

    Map.screen.update()
    env.render()


pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)
# Enter main event loop
pyglet.app.run()
Map.screen.mainloop()
env.close()
