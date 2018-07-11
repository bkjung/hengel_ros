#!/usr/bin/env python

import rospy
import roslaunch
import sys
from paint_drawing import PaintDrawing
from paint_letter import PaintLetter
from paint_lieul import PaintLieul
from paint_korea import PaintKorea
from paint_square import PaintSquare
from test_code import GoToPoint

OPTION_LETTERS = 1
OPTION_DRAWING = 2
OPTION_TEST = 3
OPTION_LIEUL = 4
OPTION_KOREA = 5
OPTION_SQUARE = 6


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
                app = PaintDrawing()
                app.run()

            elif self.runningOption == OPTION_TEST:
                GoToPoint()
            elif self.runningOption == OPTION_LIEUL:
                PaintLieul()
            elif self.runningOption == OPTION_KOREA:
                PaintKorea()
            elif self.runningOption == OPTION_SQUARE:
                PaintSquare()
            else:
                raise Exception(
                    "WRONG INPUT OPTION FOR PAINTING (Neither 1 nor 2)")

        except Exception as e:
            print(e)
            sys.exit()

    def initialOptionSelect(self):
        while True:
            word = raw_input(
                "HENGEL ROBOT Made By NAVER LABS Robotics 5th Intern.\n[1] Print Letters.\n[2] Print Drawing of Yours.\n[3] OPTION_TEST\n[4] OPTION_LIEUL\n[5]OPTION_KOREA\n[6]OPTION_SQUARE\nType :"
            )
            self.runningOption = int(word)
            if self.runningOption >= OPTION_LETTERS and self.runningOption <=OPTION_SQUARE:
                break


if __name__ == '__main__':
    try:
        HengelMain()
        print("End of Main Function")
        sys.exit()

    except Exception as e:
        rospy.loginfo("shutdown program by following exception")
        print(e)
