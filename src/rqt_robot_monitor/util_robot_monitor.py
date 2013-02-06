# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Isaac Saito, Ze'ev Klapow

import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from python_qt_binding.QtGui import QColor, QIcon


class Util(object):
    """
    This class provides variables and methods, which are commonly used
    throughout the rqt_robot_monitor package but probably not versatile enough
    for other pkgs.
    """
    #TODO(Isaac) Separate utility methodss and common configs are mixed in this
    # class.

    SECONDS_TIMELINE = 30

    # Instantiating icons that show the device status.
    ERR_ICON = QIcon.fromTheme('dialog-error')  # 'face-angry')
    WARN_ICON = QIcon.fromTheme('dialog-warning')  # 'face-sick')
    OK_ICON = QIcon.fromTheme('emblem-default')  # 'face-laugh')
    # Added following this QA thread http://goo.gl/83tVZ
    STALE_ICON = QIcon.fromTheme('dialog-question')  # 'face-tired')

    IMG_DICT = {0: OK_ICON, 1: WARN_ICON, 2: ERR_ICON, 3: STALE_ICON}

    COLOR_DICT = {0: QColor(85, 178, 76),
                   1: QColor(222, 213, 17),
                   2: QColor(178, 23, 46),
                   3: QColor(40, 23, 176)
                   }
    # DiagnosticStatus dosn't have Stale status. Related QA:http://goo.gl/83tVZ
    # It's not ideal to add STALE to DiagnosticStatus as you see in that
    # thread, but here this addition is only temporary for the purpose of
    # implementation simplicity.
    DiagnosticStatus.STALE = 3

    DICTKEY_TIMES_ERROR = 'times_errors'
    DICTKEY_TIMES_WARN = 'times_warnings'
    DICTKEY_INDEX = 'index'
    DICTKEY_STATITEM = 'statitem'

    def __init__(self):
        super(Util, self).__init__()

    @staticmethod
    def update_status_images(diagnostic_status, statusitem):
        """
        Update statusitem's status icon w.r.t. diagnostic_status.

        :type diagnostic_status: DiagnosticStatus
        :type node: StatusItem
        :author: Isaac Saito
        """

        name = diagnostic_status.name
        if (name is not None):
            # level = diagnosis_status.level
            level = diagnostic_status.level
            rospy.logdebug('New diag_status level: %s. Last lv: %s name: %s',
                       level, statusitem.last_level, name)
            if (diagnostic_status.level != statusitem.last_level):
                # TODO Apparently diagnosis_status doesn't contain last_level.
                statusitem.setIcon(0, Util.IMG_DICT[level])
                statusitem.last_level = level
                return

    @staticmethod
    def get_grn_resource_name(status_name):
        """
        Return the resource name, ie. the name at the right-end of GRN
        (Graph Resource Names, ref.
         http://www.ros.org/wiki/ROS/Concepts#Names.Graph_Resource_Names).

        Ex. If status_name = '/PR2/Power System/Smart Battery1.3',
            return value will become 'Smart Battery1.3'.

        :param: status_name is a string that may consists of status names that
                  are delimited by slash.
        :rtype: str
        """
        name = status_name.split('/')[-1]
        rospy.logdebug(' get_grn_resource_name name = %s', name)
        return name

    @staticmethod
    def get_parent_name(status_name):
        """
        Return all graph paths in the given status_name except for the resource
        name.

        status_name = Util.get_parent_name(status_name) + '/' +
                      Util.get_grn_resource_name(status_name)
        """
        return ('/'.join(status_name.split('/')[:-1])).strip()

    @staticmethod
    def gen_headline_status_green(diagnostic_status):
        """
        Internally uses get_grn_resource_name.

        :type diagnostic_status: DiagnosticStatus
        :return: Resource name of the input argument diagnostic_status.
        :rtype: str
        """
        return "%s" % Util.get_grn_resource_name(diagnostic_status.name)

    @staticmethod
    def gen_headline_warn_or_err(diagnostic_status):
        """
        :return: Tuple of status msg/text. 1st elem = resource name,
                 2nd = status text.
        :rtype: (str, str)
        """
        return "%s : %s" % (Util.get_grn_resource_name(diagnostic_status.name),
                            diagnostic_status.message)

    @staticmethod
    def get_color_for_message(msg, mode=0):
        """
        :param msg: Either DiagnosticArray or DiagnosticsStatus.
        :param mode: int. When 0, this func will iterate msg to find
                     DiagnosticsStatus.level and put it into a dict.
                     When 1, this func finds DiagnosticsStatus.level from msg
                     without iteration (thus, msg is expected to be
                     DiagnosticsStatus instance).
        :rtype: QColor
        """
        level = 0
        min_level = 255
        lookup = {}
        for status in msg.status:
            lookup[status.name] = status

        names = [status.name for status in msg.status]
        names = [name for name in names
                 if len(Util.get_parent_name(name)) == 0]
        for name in names:
            status = lookup[name]
            if (status.level > level):
                level = status.level
            if (status.level < min_level):
                min_level = status.level

        # Stale items should be reported as errors unless all stale
        if (level > 2 and min_level <= 2):
            level = 2

        #return Util.IMG_DICT[level]
        rospy.logdebug(' get_color_for_message color lv=%d', level)
        return Util.COLOR_DICT[level]

    @staticmethod
    def get_correspondent(key, list_statitem):
        """
        Return a dictionary of StatusItem instance and its index that
        corresponds to key from given list_statitem.

        :type key: String.
        :type list_statitem: DiagnosticsStatus
        :rtype: (str, StatusItem)
        """
        names_from_list = [Util.get_grn_resource_name(k.name) for k
                           in list_statitem]
        key_rsc_name = Util.get_grn_resource_name(key)
        index_key = -1
        statitem_key = None
        if key_rsc_name in names_from_list:
            index_key = names_from_list.index(key_rsc_name)
            statitem_key = list_statitem[index_key]
            rospy.logdebug(' get_correspondent index_key=%s statitem_key=%s',
                          index_key, statitem_key)
        return {Util.DICTKEY_INDEX: index_key,
                Util.DICTKEY_STATITEM: statitem_key}

    @staticmethod
    def get_children(name, diag_array):
        """
        :type msg: DiagnosticArray
        :rtype: DiagnosticStatus[]
        """

        ret = []
        for k in diag_array.status:  # k is DiagnosticStatus.
            if k.name.startswith(name):  # Starting with self.name means k is
                                        # either top/parent node or its child.
                if not k.name == name:  # Child's name must be different
                                            # from that of the top/parent node.
                    ret.append(k)
        return ret
