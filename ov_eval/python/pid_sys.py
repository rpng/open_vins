#!/usr/bin/env python

# OpenVINS: An Open Platform for Visual-Inertial Research
# Copyright (C) 2019 Patrick Geneva
# Copyright (C) 2019 OpenVINS Contributors
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
import psutil
import rospy
import sys


def get_process_name(process_name, doprint=False):
    # try to get the process using psutil
    # https://stackoverflow.com/a/2241047/7718197
    processes = []
    for proc in psutil.process_iter():
        name, exe, cmdline = "", "", []
        try:
            name = proc.name()
            cmdline = proc.cmdline()
            exe = proc.exe()
        except (psutil.AccessDenied, psutil.ZombieProcess):
            pass
        except psutil.NoSuchProcess:
            continue
        if name == process_name or cmdline[0] == process_name or os.path.basename(exe) == process_name:
            if doprint:
                rospy.loginfo("adding new node monitor (pid %d)" % (proc.pid))
            processes.append(proc)
    # if we have a process, then success
    if len(processes) > 0:
        return processes
    # else we have failed!
    rospy.logerr("unable to find process for %s" % (process_name))
    return False


if __name__ == '__main__':

    # initialize this ros node
    rospy.init_node("pid_sys")

    # check if we have our params
    if len(sys.argv) < 2:
        rospy.logerr("please specify process name")
        rospy.logerr("python pid_sys.py <command-name>")
        sys.exit(-1)

    # load our process, keep trying until we connect to it
    processes = False
    rate = rospy.Rate(2)
    while processes == False and not rospy.is_shutdown():
        processes = get_process_name(sys.argv[1], True)

    # exit if we should end
    if rospy.is_shutdown():
        sys.exit(-1)

    # now lets loop and get the stats for these processes
    while not rospy.is_shutdown():

        # summed over all the pid for this process
        sum_perc_cpu = 0.0
        sum_perc_mem = 0.0
        sum_threads = 0
        for p in processes:
            try:
                perc_cpu = p.cpu_percent(interval=None)
                perc_mem = p.memory_percent()
                threads = p.num_threads()
            except:
                continue
            sum_perc_cpu += perc_cpu
            sum_perc_mem += perc_mem
            sum_threads += threads

        # print what the total summed value is
        print("cpu percent = %.3f" % sum_perc_cpu)
        print("mem percent = %.3f" % sum_perc_mem)
        print("num threads = %d" % sum_threads)
        processes = False

        # try to get the process again, this allows us to handle
        # the spawning of new threads or removing of threads that have finished
        while processes == False and not rospy.is_shutdown():
            processes = get_process_name(sys.argv[1])
            if not processes == False:
                for p in processes:
                    p.cpu_percent(interval=None)
            rate.sleep()
