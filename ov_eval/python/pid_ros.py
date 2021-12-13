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
import rosnode
import rospy
import sys
import time

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy
import psutil


def get_process_ros(node_name, doprint=False):
    # get the node object from ros
    node_api = rosnode.get_api_uri(rospy.get_master(), node_name, skip_cache=True)[2]
    if not node_api:
        rospy.logwarn("could not get api of node %s (%s)" % (node_name, node_api))
        return False
    # now try to get the Pid of this process
    try:
        response = ServerProxy(node_api).getPid('/NODEINFO')
    except:
        rospy.logwarn("failed to get of the pid of ros node %s (%s)" % (node_name, node_api))
        return False
    # try to get the process using psutil
    try:
        process = psutil.Process(response[2])
        if doprint:
            rospy.loginfo("adding new node monitor %s (pid %d)" % (node_name, process.pid))
        return process
    except:
        rospy.logwarn("unable to open psutil object for %s" % (response[2]))
        return False


if __name__ == '__main__':

    # initialize this ros node
    rospy.init_node("pid_ros")

    # check if we have our params
    if not rospy.has_param('~nodes') or not rospy.has_param('~output'):
        rospy.logerr("please specify the nodes and output file for this logger 1")
        rospy.logerr("rosrun ov_eval pid_ros.py _nodes:=<comma,separated,node,names> _output:=<file.txt>")
        sys.exit(-1)

    # get our paramters
    node_csv = rospy.get_param("~nodes")
    node_list = node_csv.split(',')
    save_path = rospy.get_param("~output")

    # debug print to console
    rospy.loginfo("processes: %s (%d in total)" % (node_csv, len(node_list)))
    rospy.loginfo("save path: %s" % save_path)

    # ===================================================================
    # ===================================================================

    # make sure the directory is made
    if not os.path.exists(os.path.dirname(save_path)):
        try:
            os.makedirs(os.path.dirname(save_path))
        except:
            rospy.logerr("unable to create the save path!!!!!")
            sys.exit(-1)

    # open the file we will write the stats into
    file = open(save_path, "w")

    # write header to file
    header = "# timestamp(s) summed_cpu_perc summed_mem_perc summed_threads"
    for node in node_list:
        get_process_ros(node, True)  # nice debug print!
        header += " " + str(node) + "_cpu_perc " + str(node) + "_mem_perc " + str(node) + "_threads"
    header += "\n"
    file.write(header)

    # ===================================================================
    # ===================================================================

    # exit if we should end
    if rospy.is_shutdown():
        file.close()
        sys.exit(-1)

    # now lets loop and get the stats for these processes
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        # get the pid processes for this object
        ps_list = []
        for node in node_list:
            ps_list.append(get_process_ros(node, False))
            try:
                perc_cpu = ps_list[len(ps_list) - 1].cpu_percent(interval=None)
                perc_mem = ps_list[len(ps_list) - 1].memory_percent()
                threads = ps_list[len(ps_list) - 1].num_threads()
            except:
                continue

        # wait one second so we can collect data
        rate.sleep()

        # loop through and get our measurement readings
        perc_cpu = []
        perc_mem = []
        threads = []
        for i in range(0, len(node_list)):
            try:
                # get readings
                p_cpu = ps_list[i].cpu_percent(interval=None)
                p_mem = ps_list[i].memory_percent()
                p_threads = ps_list[i].num_threads()
                # append to our list
                perc_cpu.append(p_cpu)
                perc_mem.append(p_mem)
                threads.append(p_threads)
            except:
                # record just zeros if we do not have this value
                perc_cpu.append(0)
                perc_mem.append(0)
                threads.append(0)

        # print what the total summed value is
        rospy.loginfo("cpu%% = %.3f | mem%% = %.3f | threads = %d" % (sum(perc_cpu), sum(perc_mem), sum(threads)))

        # save the current stats to file!
        data = "%.8f %.3f %.3f %d" % (time.time(), sum(perc_cpu), sum(perc_mem), sum(threads))
        for i in range(0, len(node_list)):
            data += " %.3f %.3f %d" % (perc_cpu[i], perc_mem[i], threads[i])
        data += "\n"
        file.write(data)
        file.flush()

    # finally close the file!
    file.close()
