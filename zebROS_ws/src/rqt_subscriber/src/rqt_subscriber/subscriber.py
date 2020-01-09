#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import math
import random
import time

from python_qt_binding.QtCore import Slot, QSignalMapper, QTimer, qWarning

import roslib
import rospy
import genpy
from rqt_gui_py.plugin import Plugin
from .subscriber_widget import SubscriberWidget
from rqt_py_common.topic_helpers import get_field_type


class Subscriber(Plugin):

    def __init__(self, context):
        super(Subscriber, self).__init__(context)
        self.setObjectName('Subscriber')

        # create widget
        self._widget = SubscriberWidget()
        self._widget.add_subscriber.connect(self.add_subscriber)
        self._widget.change_subscriber.connect(self.change_subscriber)
        self._widget.publish_once.connect(self.publish_once)
        self._widget.remove_subscriber.connect(self.remove_subscriber)
        self._widget.clean_up_subscribers.connect(self.clean_up_subscribers)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # create context for the expression eval statement
        self._eval_locals = {'i': 0}
        for module in (math, random, time):
            self._eval_locals.update(module.__dict__)
        self._eval_locals['genpy'] = genpy
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']

        self._subscribers = {}
        self._id_counter = 0

        #self._timeout_mapper = QSignalMapper(self)
        #self._timeout_mapper.mapped[int].connect(self.publish_once)

        # add our self to the main window
        context.add_widget(self._widget)

    @Slot(str, str, float, bool)
    def add_subscriber(self, topic_name, type_name, rate, enabled):
        subscriber_info = {
            'topic_name': str(topic_name),
            'type_name': str(type_name),
            'enabled': bool(enabled),
        }
        self._add_subscriber(subscriber_info)

    def _add_subscriber(self, subscriber_info):
        subscriber_info['subscriber_id'] = self._id_counter
        self._id_counter += 1
        subscriber_info['counter'] = 0
        subscriber_info['enabled'] = subscriber_info.get('enabled', False)

        subscriber_info['message_instance'] = self._create_message_instance(
            subscriber_info['type_name'])
        if subscriber_info['message_instance'] is None:
            return

        # create subscriber and timer
       # try:
            subscriber_info['subscriber'] = rospy.Subscriber(
                subscriber_info['topic_name'], subscriber_info['type_name'], self.sub_callback)
        #except TypeError:
        #    subscriber_info['subscriber'] = rospy.Subscriber(
        #        subscriber_info['topic_name'], subscriber_info['type_name'], self.sub_callback))

        # add subscriber info to _subscribers dict and create signal mapping
        self._subscribers[subscriber_info['subscriber_id']] = subscriber_info
        
       # self._timeout_mapper.setMapping(subscriber_info['timer'], subscriber_info['subscriber_id'])
       # subscriber_info['timer'].timeout.connect(self._timeout_mapper.map)
       # if subscriber_info['enabled'] and subscriber_info['rate'] > 0:
       #     subscriber_info['timer'].start(int(1000.0 / subscriber_info['rate']))

        #add subscriber to tree widget
        self._widget.subscriber_tree_widget.model().add_subscriber(subscriber_info)

    def sub_callback(self, msg):
        if(True):
            print("hi")

#    @Slot(int, str, str, str, object)
#    def change_subscriber(self, subscriber_id, topic_name, column_name, new_value, setter_callback):
#        handler = getattr(self, '_change_subscriber_%s' % column_name, None)
#        if handler is not None:
#            new_text = handler(self._subscribers[subscriber_id], topic_name, new_value)
#            if new_text is not None:
#                setter_callback(new_text)
#
#    def _change_subscriber_topic(self, subscriber_info, topic_name, new_value):
#        subscriber_info['enabled'] = (new_value and new_value.lower() in ['1', 'true', 'yes'])
#        # qDebug(
#        #   'Subscriber._change_subscriber_enabled(): %s enabled: %s' %
#        #   (subscriber_info['topic_name'], subscriber_info['enabled']))
#        if subscriber_info['enabled'] and subscriber_info['rate'] > 0:
#            subscriber_info['timer'].start(int(1000.0 / subscriber_info['rate']))
#        else:
#            subscriber_info['timer'].stop()
#        return None
#
#    def _change_subscriber_type(self, subscriber_info, topic_name, new_value):
#        type_name = new_value
#        # create new slot
#        slot_value = self._create_message_instance(type_name)
#
#        # find parent slot
#        slot_path = topic_name[len(subscriber_info['topic_name']):].strip('/').split('/')
#        parent_slot = eval('.'.join(["subscriber_info['message_instance']"] + slot_path[:-1]))
#
#        # find old slot
#        slot_name = slot_path[-1]
#        slot_index = parent_slot.__slots__.index(slot_name)
#
#        # restore type if user value was invalid
#        if slot_value is None:
#            qWarning('Subscriber._change_subscriber_type(): could not find type: %s' % (type_name))
#            return parent_slot._slot_types[slot_index]
#
#        else:
#            # replace old slot
#            parent_slot._slot_types[slot_index] = type_name
#            setattr(parent_slot, slot_name, slot_value)
#
#            self._widget.subscriber_tree_widget.model().update_subscriber(subscriber_info)
#
#    def _change_subscriber_rate(self, subscriber_info, topic_name, new_value):
#        try:
#            rate = float(new_value)
#        except Exception:
#            qWarning('Subscriber._change_subscriber_rate(): could not parse rate value: %s' %
#                     (new_value))
#        else:
#            subscriber_info['rate'] = rate
#            # qDebug(
#            #   'Subscriber._change_subscriber_rate(): %s rate changed: %fHz' %
#            #   (subscriber_info['topic_name'], subscriber_info['rate']))
#            subscriber_info['timer'].stop()
#            if subscriber_info['enabled'] and subscriber_info['rate'] > 0:
#                subscriber_info['timer'].start(int(1000.0 / subscriber_info['rate']))
#        # make sure the column value reflects the actual rate
#        return '%.2f' % subscriber_info['rate']
#
#    def _change_subscriber_expression(self, subscriber_info, topic_name, new_value):
#        expression = str(new_value)
#        if len(expression) == 0:
#            if topic_name in subscriber_info['expressions']:
#                del subscriber_info['expressions'][topic_name]
#                # qDebug(
#                #   'Subscriber._change_subscriber_expression(): removed expression'
#                #   'for: %s' % (topic_name))
#        else:
#            slot_type, is_array = get_field_type(topic_name)
#            if is_array:
#                slot_type = list
#            # strip possible trailing error message from expression
#            error_prefix = '# error'
#            error_prefix_pos = expression.find(error_prefix)
#            if error_prefix_pos >= 0:
#                expression = expression[:error_prefix_pos]
#            success, _ = self._evaluate_expression(expression, slot_type)
#            if success:
#                old_expression = subscriber_info['expressions'].get(topic_name, None)
#                subscriber_info['expressions'][topic_name] = expression
#                # print('Subscriber._change_subscriber_expression(): topic: %s, type: %s,'
#                #   'expression: %s') % (topic_name, slot_type, new_value)
#                self._fill_message_slots(
#                    subscriber_info['message_instance'], subscriber_info['topic_name'],
#                    subscriber_info['expressions'], subscriber_info['counter'])
#                try:
#                    subscriber_info['message_instance']._check_types()
#                except Exception as e:
#                    print('serialization error: %s' % e)
#                    if old_expression is not None:
#                        subscriber_info['expressions'][topic_name] = old_expression
#                    else:
#                        del subscriber_info['expressions'][topic_name]
#                    return '%s %s: %s' % (expression, error_prefix, e)
#                return expression
#            else:
#                return '%s %s evaluating as "%s"' % (expression, error_prefix, slot_type.__name__)
#
#    def _extract_array_info(self, type_str):
#        array_size = None
#        if '[' in type_str and type_str[-1] == ']':
#            type_str, array_size_str = type_str.split('[', 1)
#            array_size_str = array_size_str[:-1]
#            if len(array_size_str) > 0:
#                array_size = int(array_size_str)
#            else:
#                array_size = 0
#
#        return type_str, array_size
#
#    def _create_message_instance(self, type_str):
#        base_type_str, array_size = self._extract_array_info(type_str)
#
#        base_message_type = roslib.message.get_message_class(base_type_str)
#        if base_message_type is None:
#            print('Could not create message of type "%s".' % base_type_str)
#            return None
#
#        if array_size is not None:
#            message = []
#            for _ in range(array_size):
#                message.append(base_message_type())
#        else:
#            message = base_message_type()
#        return message
#
    def _evaluate_expression(self, expression, slot_type):
        successful_eval = True

        try:
            # try to evaluate expression
            value = eval(expression, {}, self._eval_locals)
        except Exception:
            successful_eval = False

        if slot_type is str:
            if successful_eval:
                value = str(value)
            else:
                # for string slots just convert the expression to str, if it did not
                # evaluate successfully
                value = str(expression)
            successful_eval = True

        elif successful_eval:
            type_set = set((slot_type, type(value)))
            # check if value's type and slot_type belong to the same type group, i.e. array types,
            # numeric types and if they do, make sure values's type is converted to the exact
            # slot_type
            if type_set <= set((list, tuple)) or type_set <= set((int, float)):
                # convert to the right type
                value = slot_type(value)

        if successful_eval and isinstance(value, slot_type):
            return True, value
        else:
            qWarning(
                'Subscriber._evaluate_expression(): failed to evaluate expression: "%s" as '
                'Python type "%s"' % (expression, slot_type.__name__))

        return False, None

    def _fill_message_slots(self, message, topic_name, expressions, counter):
        if topic_name in expressions and len(expressions[topic_name]) > 0:

            # get type
            if hasattr(message, '_type'):
                message_type = message._type
            else:
                message_type = type(message)

            self._eval_locals['i'] = counter
            success, value = self._evaluate_expression(expressions[topic_name], message_type)
            if not success:
                value = message_type()
            return value

        # if no expression exists for this topic_name, continue with it's child slots
        elif hasattr(message, '__slots__'):
            for slot_name in message.__slots__:
                value = self._fill_message_slots(
                    getattr(message, slot_name), topic_name + '/' + slot_name, expressions, counter)
                if value is not None:
                    setattr(message, slot_name, value)

        elif type(message) in (list, tuple) and (len(message) > 0):
            for index, slot in enumerate(message):
                value = self._fill_message_slots(
                    slot, topic_name + '[%d]' % index, expressions, counter)
                # this deals with primitive-type arrays
                if not hasattr(message[0], '__slots__') and value is not None:
                    message[index] = value

        return None


    @Slot(int)
    def remove_subscriber(self, subscriber_id):
        subscriber_info = self._subscribers.get(subscriber_id, None)
        if subscriber_info is not None:
            subscriber_info['timer'].stop()
            subscriber_info['subscriber'].unregister()
            del self._subscribers[subscriber_id]

    def save_settings(self, plugin_settings, instance_settings):
        subscriber_copies = []
        for subscriber in self._subscribers.values():
            subscriber_copy = {}
            subscriber_copy.update(subscriber)
            subscriber_copy['enabled'] = False
            del subscriber_copy['timer']
            del subscriber_copy['message_instance']
            del subscriber_copy['subscriber']
            subscriber_copies.append(subscriber_copy)
        instance_settings.set_value('subscribers', repr(subscriber_copies))

    def restore_settings(self, plugin_settings, instance_settings):
        subscribers = eval(instance_settings.value('subscribers', '[]'))
        for subscriber in subscribers:
            self._add_subscriber(subscriber)

    def clean_up_subscribers(self):
        self._widget.subscriber_tree_widget.model().clear()
        for subscriber_info in self._subscribers.values():
            subscriber_info['timer'].stop()
            subscriber_info['subscriber'].unregister()
        self._subscribers = {}

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
        self.clean_up_subscribers()
