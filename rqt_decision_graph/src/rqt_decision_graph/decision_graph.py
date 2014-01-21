"""
Copyright (c) 2013, Cogniteam
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

*   Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

*   Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

*   Neither the name of the Cogniteam nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from .graph import Graph


class DecisionGraph(Graph):

    def __init__(self, name, graph_id, node_name, node_exe_file, node_exe_dir, dot_processor, key, parent=None):
        super(DecisionGraph, self).__init__(dot_processor=dot_processor, source=None, parent=parent)
        self.name = name
        self.graph_id = graph_id
        self.node_name = node_name
        self.node_exe_file = node_exe_file
        self.node_exe_dir = node_exe_dir
        self.key = key

        self.source = self._get_dot_file_name()

    def _get_dot_file_name(self):
        if not self.node_exe_file.startswith('/'):
            return self.node_exe_dir.replace('lib', 'share') + '/graphs/' + self.name + '.dot'

        return self.node_exe_file.replace('lib', 'share').replace(self.node_exe_file.split('/')[-1], 'graphs') \
               + '/' + self.name + '.dot'