#!/usr/bin/env python

import rospy
import roslaunch

OPTION_LETTERS = 1
OPTION_DRAWING = 2


class HengelMain():
    def __init__(self):
        try:
            rospy.init_node('hengel_main_program', anonymous=False, disable_signals=True)
            r = rospy.Rate(50)
            self.runningOption = 0

            self.programStart()

            if runningOption == OPTION_LETTERS:
                package = 'hengel_navigation'
                executable = 'printer_letter'
                node = roslaunch.core.Node(package, executable)

                launch = roslaunch.scriptapi.ROSLaunch()
                launch.start()

                process = launch.launch(node)
                # print process.is_alive()
                # process.stop()

            elif runningOption == OPTION_DRAWING:
                package = 'hengel_navigation'
                executable = 'printer_drawing'
                node = roslaunch.core.Node(package, executable)

                launch = roslaunch.scriptapi.ROSLaunch()
                launch.start()

                process = launch.launch(node)
                # print process.is_alive()
                # process.stop()


            else:
                raise Exception("WRONG INPUT OPTION FOR PAINTING (Neither 1 nor 2)")


        except Exception as e:
            print(e)
            sys.exit()


    def programStart(self):
        word=raw_input("HENGEL ROBOT Made By NAVER LABS Robotics 5th Intern.\nThere are two options of painting.\n[1] Print Letters.\n[2] Print Drawing of Yours.\nType 1 or 2 :")
        print("Input:"+word)
        self.runningOption = int(word)


if __name__ == '__main__':
    try:
        HengelMain()
        print("End of Main Function")

    except Exception as e:
        rospy.loginfo("shutdown program by following exception")
        print(e)
