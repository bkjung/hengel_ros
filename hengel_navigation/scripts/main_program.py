#!/usr/bin/env python

import rospy
import roslaunch
import sys
from paint_drawing import PaintDrawing
from paint_letter import PaintLetter
from test_code import GoToPoint

OPTION_LETTERS = 1
OPTION_DRAWING = 2
OPTION_TEST = 3


class HengelMain():
    def __init__(self):
        try:
            rospy.init_node(
                'hengel_main_program', anonymous=False, disable_signals=True)
            r = rospy.Rate(50)
            self.runningOption = 0

            self.initialOptionSelect()

            if self.runningOption == OPTION_LETTERS:
                PaintLetter()

            elif self.runningOption == OPTION_DRAWING:
                PaintDrawing()

            elif self.runningOption == OPTION_TEST:
                GoToPoint()

            else:
                raise Exception(
                    "WRONG INPUT OPTION FOR PAINTING (Neither 1 nor 2)")

        except Exception as e:
            print(e)
            sys.exit()

    def initialOptionSelect(self):
        word = raw_input(
            "HENGEL ROBOT Made By NAVER LABS Robotics 5th Intern.\nThere are two options of painting.\n[1] Print Letters.\n[2] Print Drawing of Yours.\nType 1 or 2 :"
        )
        self.runningOption = int(word)


if __name__ == '__main__':
    try:
        HengelMain()
        print("End of Main Function")
        sys.exit()

    except Exception as e:
        rospy.loginfo("shutdown program by following exception")
        print(e)
