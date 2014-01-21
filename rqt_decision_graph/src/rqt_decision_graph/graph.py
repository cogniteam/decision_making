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

import pydot


class Graph(object):

    @staticmethod
    def __read_dot_data_from_file(path):
        try:
            with open(path, 'r') as file_handler:
                return file_handler.read()
        except IOError:
            return None

    def __init__(self, dot_processor, source, parent=None):
        super(Graph, self).__init__()
        self.dot_processor = dot_processor
        self.parent = parent
        self.name = None
        self.nodes = None
        self.edges = None
        self.source = source

    def load(self):
        print 'Reading dot data from', self.source
        self._create_dot_from_file(self.source)

    def _create_dot_from_file(self, dot_file_path):
        dot_data = self.__read_dot_data_from_file(dot_file_path)
        if dot_data is None:
            raise GraphParseException("Could not read dot file")

        graph = pydot.graph_from_dot_data(dot_data.encode("ascii", "ignore"))
        if graph is None:
            raise GraphParseException("Could not create graph based on loaded dot data")

        dot_graph = graph.create_dot()
        if dot_graph is None:
            raise GraphParseException("Could not create DOT graph based on graph")

        self.nodes, self.edges = self.dot_processor.process(dot_graph)


class GraphParseException(Exception):
    pass