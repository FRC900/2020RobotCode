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

from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction

from .subscriber_tree_model import SubscriberTreeModel
from rqt_py_common.message_tree_widget import MessageTreeWidget
from rqt_py_common.item_delegates import SpinBoxDelegate


class SubscriberTreeWidget(MessageTreeWidget):
    remove_subscriber = Signal(int)
    publish_once = Signal(int)

    def __init__(self, parent=None):
        super(SubscriberTreeWidget, self).__init__(parent)
        self.setModel(SubscriberTreeModel(self))
        self._action_remove_subscriber = QAction(
            QIcon.fromTheme('list-remove'), 'Remove Selected', self)
        self._action_remove_subscriber.triggered[bool].connect(self._handle_action_remove_subscriber)
        self._action_publish_once = QAction(
            QIcon.fromTheme('media-playback-start'), 'Publish Selected Once', self)
        self._action_publish_once.triggered[bool].connect(self._handle_action_publish_once)
        self.setItemDelegateForColumn(self.model()._column_index['rate'],
                                      SpinBoxDelegate(min_value=0, max_value=1000000, decimals=2))

    @Slot()
    def remove_selected_subscribers(self):
        subscriber_ids = self.model().get_subscriber_ids(self.selectedIndexes())
        for subscriber_id in subscriber_ids:
            self.remove_subscriber.emit(subscriber_id)
        self.model().remove_items_with_parents(self.selectedIndexes())

    def _context_menu_add_actions(self, menu, pos):
        if self.selectionModel().hasSelection():
            menu.addAction(self._action_remove_subscriber)
            menu.addAction(self._action_publish_once)
        # let super class add actions
        super(SubscriberTreeWidget, self)._context_menu_add_actions(menu, pos)

    def _handle_action_remove_subscriber(self, checked):
        self.remove_selected_subscribers()

    def _handle_action_publish_once(self, checked):
        for subscriber_id in self.model().get_subscriber_ids(self.selectedIndexes()):
            self.publish_once.emit(subscriber_id)
