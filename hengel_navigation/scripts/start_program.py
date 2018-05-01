#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

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
