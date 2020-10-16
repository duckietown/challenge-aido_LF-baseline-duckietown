#!/usr/bin/env python3
from __future__ import unicode_literals
from PIL import Image
import io

import os
import time

import numpy as np
import roslaunch
from rosagent import ROSAgent
from aido_schemas import (Context, Duckiebot1Commands, Duckiebot1Observations, EpisodeStart,
                          GetCommands, LEDSCommands, protocol_agent_duckiebot1, PWMCommands, RGB, wrap_direct)


class ROSBaselineAgent(object):
    def __init__(self):
        # Now, initialize the ROS stuff here:

        # logger.info('Configuring logging')
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        roslaunch_path = os.path.join(os.getcwd(), "lf_slim.launch")
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_path])
        self.launch.start()
        # Start the ROSAgent, which handles publishing images and subscribing to action
        self.agent = ROSAgent()

    def init(self, context: Context):
        context.info("init()")

    def on_received_seed(self, context: Context, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        context.info('Starting episode %s.' % data)

    def on_received_observations(self, context: Context, data: Duckiebot1Observations):
        jpg_data = data.camera.jpg_data
        obs = jpg2rgb(jpg_data)
        self.agent._publish_img(obs)
        self.agent._publish_info()

    def on_received_get_commands(self, context: Context, data: GetCommands):
        print(f"Received get_command instate updated: {self.agent.updated}", flush=True)
        # while not self.agent.updated:
        #     time.sleep(0.01)

        pwm_left, pwm_right = self.agent.action
        self.agent.updated = False

        print(f"sending action {self.agent.action}", flush=True)
        grey = RGB(0.5, 0.5, 0.5)
        led_commands = LEDSCommands(grey, grey, grey, grey, grey)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = Duckiebot1Commands(pwm_commands, led_commands)

        context.write('commands', commands)

    def finish(self, context):
        context.info('finish()')


def jpg2rgb(image_data):
    """ Reads JPG bytes as RGB"""
    im = Image.open(io.BytesIO(image_data))
    im = im.convert('RGB')
    data = np.array(im)
    assert data.ndim == 3
    assert data.dtype == np.uint8
    return data


if __name__ == '__main__':
    agent = ROSBaselineAgent()
    wrap_direct(agent, protocol_agent_duckiebot1)
