#!/usr/bin/env python

import rospy
import pygame
import rospkg
import os
from std_msgs.msg import Float64


class Text(object):
    def __init__(self, font_name=None, size=30, text="empty text"):
        self.font_name = font_name
        self.font_size = size
        self.color_fg = (255, 255, 255)
        self.color_bg = (0, 0, 0)
        self._aa = True
        self._text = text
        self.font = pygame.font.SysFont(font_name, size)
        self.screen = pygame.display.get_surface()

        self.dirty = True
        self.image = None
        self.rect = None
        self._render()

    def _render(self):
        self.dirty = False
        self.image = self.font.render(self._text, self.aa, self.color_fg)
        self.rect = self.image.get_rect()

    def draw(self):
        if self.dirty or (self.image is None):
            self._render()
        self.screen.blit(self.image, self.rect)

    @property
    def text(self):
        return self._text

    @text.setter
    def text(self, text):
        if text != self._text:
            self.dirty = True
            self._text = text

    @property
    def aa(self):
        return self._aa

    @aa.setter
    def aa(self, aa):
        self.dirty = True
        self._aa = aa


class TextGrid(object):
    def __init__(self, layout, padding, size):
        self.texts = []
        for row in range(layout[0]):
            row_list = []
            for col in range(layout[1]):
                row_list.append(
                    Text("dejavusansmono", size, "{}x{}".format(row, col)))
            self.texts.append(row_list)

        self.padding = padding
        self.font_size = size
        self.screen = pygame.display.get_surface()
        self.layout = layout
        self.draw()

    def draw(self):
        self.dirty = False
        col_widths = self.get_column_widths()
        row_heights = self.get_row_heights()
        x_col_offsets = [self.padding[0]] * self.layout[1]
        y_row_offsets = [self.padding[1]] * self.layout[0]
        for col in range(1, self.layout[1]):
            x_col_offsets[col] = x_col_offsets[
                col - 1] + self.padding[0] + col_widths[col - 1]
        for row in range(1, self.layout[0]):
            y_row_offsets[row] = y_row_offsets[
                row - 1] + self.padding[1] + row_heights[row - 1]

        for row in range(self.layout[0]):
            for col in range(self.layout[1]):
                self.texts[row][col].rect.top = y_row_offsets[row]
                self.texts[row][col].rect.left = x_col_offsets[col]
                self.texts[row][col].draw()

    def get_column_widths(self):
        widths = []
        for col in range(self.layout[1]):
            max_width = 0
            for row in range(self.layout[0]):
                if self.texts[row][col].rect.width > max_width:
                    max_width = self.texts[row][col].rect.width
            widths.append(max_width)
        return widths

    def get_row_heights(self):
        heights = []
        for row in range(self.layout[0]):
            max_height = 0
            for col in range(self.layout[1]):
                if self.texts[row][col].rect.height > max_height:
                    max_height = self.texts[row][col].rect.height
            heights.append(max_height)
        return heights

    def set_text(self, row, col, string):
        self.texts[row][col].text = string


class KeyboardControlNode():
    WINDOW_SIZE = (640, 200)
    DISPLAY_FLAGS = pygame.DOUBLEBUF

    def __init__(self, name):
        pygame.init()
        pygame.mixer.quit()
        rospy.init_node(name)

        self.thrust = 0.0
        self.thrust_stepsize = 0.1
        self.thrust_scaler = 0.4
        self.lateral_thrust = 0.0
        self.lateral_thrust_stepsize = 0.1
        self.lateral_thrust_scaler = 0.4
        self.vertical_thrust = 0.0
        self.vertical_thrust_stepsize = 0.1
        self.vertical_thrust_scaler = 0.4
        self.yaw_rate = 0.0
        self.yaw_rate_stepsize = 0.1
        self.yaw_rate_scaler = 0.2

        # create GUI
        self.screen = self.init_display()
        self.text_grid = self.init_text_grid()

        self.controls = self.init_controls()

        self.roll_pub = rospy.Publisher("roll", Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher("pitch", Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher("yaw", Float64, queue_size=1)
        self.thrust_pub = rospy.Publisher("thrust", Float64, queue_size=1)
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust",
                                                  Float64,
                                                  queue_size=1)

    def init_display(self):
        screen = pygame.display.set_mode(self.WINDOW_SIZE, self.DISPLAY_FLAGS)
        vehicle_name = rospy.get_namespace().strip("/")
        pygame.display.set_caption(
            "Keyboard Control of {}".format(vehicle_name))

        icon_path = os.path.join(self.get_resource_path(), "icon.png")
        icon = pygame.image.load(icon_path)
        pygame.display.set_icon(icon)
        return screen

    def init_text_grid(self):
        text_grid = TextGrid((4, 2), (5, 5), 30)
        text_grid.set_text(0, 0, "Thrust Scaling (1/2):")
        text_grid.set_text(0, 1, "0.0")
        text_grid.set_text(1, 0, "Yaw Scaling(3/4):")
        text_grid.set_text(1, 1, "0.0")
        text_grid.set_text(2, 0, "Vertical Thrust Scaling(5/6):")
        text_grid.set_text(2, 1, "0.0")
        text_grid.set_text(3, 0, "Lateral Thrust Scaling(7/8):")
        text_grid.set_text(3, 1, "0.0")
        text_grid.draw()
        return text_grid

    def update_text_grid(self):
        self.text_grid.set_text(0, 1, "{:.2f}".format(self.thrust_scaler))
        self.text_grid.set_text(1, 1, "{:.2f}".format(self.yaw_rate_scaler))
        self.text_grid.set_text(2, 1,
                                "{:.2f}".format(self.vertical_thrust_scaler))
        self.text_grid.set_text(3, 1,
                                "{:.2f}".format(self.lateral_thrust_scaler))
        self.text_grid.draw()

    def run(self):
        rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():
            self.handle_events()
            self.screen.fill((0, 0, 0))
            self.update_text_grid()
            pygame.display.flip()
            self.publish_message()
            rate.sleep()

    def set_thrust(self, value):
        value *= self.thrust_scaler
        self.thrust = max(-1, min(1, value))

    def set_yaw_rate(self, value):
        value *= self.yaw_rate_scaler
        self.yaw_rate = max(-1, min(1, value))

    def set_vertical_thrust(self, value):
        value *= self.vertical_thrust_scaler
        self.vertical_thrust = max(-1, min(1, value))

    def set_lateral_thrust(self, value):
        value *= self.lateral_thrust_scaler
        self.lateral_thrust = max(-1, min(1, value))

    def increase_thrust_scaler(self, value):
        self.thrust_scaler += value
        self.thrust_scaler = max(0, min(1, self.thrust_scaler))

    def increase_yaw_rate_scaler(self, value):
        self.yaw_rate_scaler += value
        self.yaw_rate_scaler = max(0, min(1, self.yaw_rate_scaler))

    def increase_vertical_thrust_scaler(self, value):
        self.vertical_thrust_scaler += value
        self.vertical_thrust_scaler = max(0, min(1,
                                                 self.vertical_thrust_scaler))

    def increase_lateral_thrust_scaler(self, value):
        self.lateral_thrust_scaler += value
        self.lateral_thrust_scaler = max(0, min(1, self.lateral_thrust_scaler))

    def init_controls(self):
        controls = {
            pygame.K_LEFT:
            dict(
                pressed=False,
                changed=False,
                description="Turn left.",
                pressed_callback=(lambda: self.set_yaw_rate(1)),
                released_callback=(lambda: self.set_yaw_rate(0)),
            ),
            pygame.K_RIGHT:
            dict(
                pressed=False,
                changed=False,
                description="Turn right.",
                pressed_callback=(lambda: self.set_yaw_rate(-1)),
                released_callback=(lambda: self.set_yaw_rate(0)),
            ),
            pygame.K_UP:
            dict(
                pressed=False,
                changed=False,
                description="Positive vertical thrust.",
                pressed_callback=(lambda: self.set_vertical_thrust(1)),
                released_callback=(lambda: self.set_vertical_thrust(0)),
            ),
            pygame.K_DOWN:
            dict(
                pressed=False,
                changed=False,
                description="Negative vertical thrust.",
                pressed_callback=(lambda: self.set_vertical_thrust(-1)),
                released_callback=(lambda: self.set_vertical_thrust(0)),
            ),
            pygame.K_q:
            dict(
                pressed=False,
                changed=False,
                description="Positive lateral thrust.",
                pressed_callback=(lambda: self.set_lateral_thrust(1)),
                released_callback=(lambda: self.set_lateral_thrust(0)),
            ),
            pygame.K_d:
            dict(
                pressed=False,
                changed=False,
                description="Negative lateral thrust.",
                pressed_callback=(lambda: self.set_lateral_thrust(-1)),
                released_callback=(lambda: self.set_lateral_thrust(0)),
            ),
            pygame.K_z:
            dict(
                pressed=False,
                changed=False,
                description="Forward thrust.",
                pressed_callback=(lambda: self.set_thrust(1)),
                released_callback=(lambda: self.set_thrust(0)),
            ),
            pygame.K_s:
            dict(
                pressed=False,
                changed=False,
                description="Backward thrust.",
                pressed_callback=(lambda: self.set_thrust(-1)),
                released_callback=(lambda: self.set_thrust(0)),
            ),
            pygame.K_1:
            dict(
                pressed=False,
                changed=False,
                description="Decrease forward thrust.",
                pressed_callback=(
                    lambda: self.increase_thrust_scaler(-self.thrust_stepsize)),
            ),
            pygame.K_2:
            dict(
                pressed=False,
                changed=False,
                description="Increase forward thrust.",
                pressed_callback=(
                    lambda: self.increase_thrust_scaler(self.thrust_stepsize)),
            ),
            pygame.K_3:
            dict(
                pressed=False,
                changed=False,
                description="Decrease yaw rate.",
                pressed_callback=(lambda: self.increase_yaw_rate_scaler(
                    -self.yaw_rate_stepsize)),
            ),
            pygame.K_4:
            dict(
                pressed=False,
                changed=False,
                description="Increase yaw rate.",
                pressed_callback=(lambda: self.increase_yaw_rate_scaler(
                    self.yaw_rate_stepsize)),
            ),
            pygame.K_5:
            dict(
                pressed=False,
                changed=False,
                description="Decrease vertical thrust.",
                pressed_callback=(lambda: self.increase_vertical_thrust_scaler(
                    -self.vertical_thrust_stepsize)),
            ),
            pygame.K_6:
            dict(
                pressed=False,
                changed=False,
                description="Increase vertical thrust.",
                pressed_callback=(lambda: self.increase_vertical_thrust_scaler(
                    self.vertical_thrust_stepsize)),
            ),
            pygame.K_7:
            dict(
                pressed=False,
                changed=False,
                description="Decrease lateral thrust.",
                pressed_callback=(lambda: self.increase_lateral_thrust_scaler(
                    -self.lateral_thrust_stepsize)),
            ),
            pygame.K_8:
            dict(
                pressed=False,
                changed=False,
                description="Increase lateral thrust.",
                pressed_callback=(lambda: self.increase_lateral_thrust_scaler(
                    self.lateral_thrust_stepsize)),
            ),
        }
        return controls

    def print_controls(self, controls):
        print("Controls:")
        for key in controls:
            print("{}: {}".format(pygame.key.name(key),
                                  controls[key]["description"]))

    def get_resource_path(self):
        res_path = rospkg.RosPack().get_path("bluerov_sim")
        res_path = os.path.join(res_path, "res")
        return res_path

    def print_current_values(self):
        print("thrust: {}\nyaw_rate: {}\nlateral_thrust: {}\n"
              "vertical_thrust: {}".format(self.thrust, self.yaw_rate,
                                           self.lateral_thrust,
                                           self.vertical_thrust))

    def handle_events(self):
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key in self.controls:
                    control = self.controls[event.key]
                    if "pressed_callback" in control:
                        control["pressed_callback"]()
            elif event.type == pygame.KEYUP:
                if event.key in self.controls:
                    control = self.controls[event.key]
                    if "released_callback" in control:
                        control["released_callback"]()

            elif event.type == pygame.QUIT:
                pygame.quit()
                rospy.signal_shutdown("Quitting")

    def publish_message(self):
        self.thrust_pub.publish(Float64(self.thrust))
        self.vertical_thrust_pub.publish(Float64(self.vertical_thrust))
        self.lateral_thrust_pub.publish(Float64(self.lateral_thrust))
        self.yaw_pub.publish(Float64(self.yaw_rate))


def main():
    node = KeyboardControlNode("keyboard")
    node.run()


if __name__ == "__main__":
    main()
