#!/usr/bin/python
import re
import pydot
import xml.etree.ElementTree as ET
from sys import stdout
import ntpath
from optparse import OptionParser
import sys
import os


node_counter=0;
def map_all_ids(xml):
	global node_counter
	map_ids = {}
	for node in xml.iter():
		if "id" not in node.attrib: continue
		map_ids[node.attrib["id"]]=str(node_counter)
		node_counter+=1
	return map_ids

def path_leaf(path):
	head, tail = ntpath.split(path)
	return tail or ntpath.basename(head)
	
class NodeShape:
	def __init__(self, tag_name):
		self.nshape=""
		self.nfillcolor="#FFFFFF"
		self.npenwidth="1"
		if tag_name in ['tsk', 'task']:
			self.nshape="box"
			self.npenwidth="3"
		elif tag_name in ['seq', 'plan', 'pln']:
			self.nshape="rarrow"
		elif tag_name=='par':
			self.nshape="parallelogram"
			self.nfillcolor="#FFCC99"
		else:
			self.nshape="ellipse"
	def set(self, node):
		node.set_shape(self.nshape)
		node.set_fillcolor(self.nfillcolor)
		#node.set_penwidth(self.npenwidth)

def graph_gen_nodes(xml, node, graph, elem, ids):
	#print 'proc',node.tag, node.attrib['name'] if 'name' in node.attrib else ""
	if node.tag in ['scxml']:
		for chnode in node:
			graph_gen_nodes(xml, chnode, graph, elem, ids)
	if node.tag in ['parallel']:
		for chnode in node:
			graph_gen_nodes(xml, chnode, graph, elem, ids)
	if node.tag in ['state']:
		if len(node.findall('state')+node.findall('plan')+node.findall('parallel'))==0:
			gr_node=pydot.Node(ids[node.attrib["id"]],label=node.attrib["name"], URL=node.attrib["id"])
			elem.add_node(gr_node)
		else:
			has_init_state = 'initialstate' in node.attrib
			lbl = node.attrib["name"]
			if has_init_state : lbl = 'FSM['+lbl+']'
			gr_cluster=pydot.Cluster(ids[node.attrib["id"]],label=lbl, URL=node.attrib["id"])
			elem.add_subgraph(gr_cluster)
			if has_init_state:
				gr_st_node = pydot.Node(ids[node.attrib["id"]]+"start", shape="point")
				gr_cluster.add_node(gr_st_node)
			for chnode in node:
				graph_gen_nodes(xml, chnode, graph, gr_cluster, ids)
	if node.tag in ['plan', 'par', 'seq', 'sel', 'dec', 'pln']:
		nlabel = node.attrib["name"]
		if node.tag in ['plan','pln']: nlabel='BT['+nlabel+']'
		gr_node=pydot.Node(ids[node.attrib["id"]],label=nlabel, URL=node.attrib["id"])
		shape = NodeShape(node.tag)
		shape.set(gr_node)
		elem.add_node(gr_node)
		for chnode in node:
			graph_gen_nodes(xml, chnode, graph, elem, ids)
	if node.tag in ['task','tsk']:
		if len(node.findall('scxml'))==0:
			gr_node=pydot.Node(ids[node.attrib["id"]],label=node.attrib["name"], URL=node.attrib["id"])
			shape = NodeShape(node.tag)
			shape.set(gr_node)
			elem.add_node(gr_node)
			for chnode in node.findall('plan'):
				graph_gen_nodes(xml, chnode, graph, elem, ids)
		else:
			gr_cluster=pydot.Cluster(ids[node.attrib["id"]],label=node.attrib["name"], URL=node.attrib["id"])
			elem.add_subgraph(gr_cluster)
			for chnode in node.findall('scxml'):
				graph_gen_nodes(xml, chnode, graph, gr_cluster, ids)

def find_simple_node(state):
	if state.tag == "state" and len([x for x in state if x.tag!='transition'])==0 : return state.attrib["id"]
	if state.tag in ['plan','pln', 'par', 'dec', 'seq', 'sel' ] : return state.attrib["id"]
	if state.tag in ["task","tsk"] and len(state.findall('scxml'))==0 : return state.attrib["id"]
	for n in state.iter(): 
		if n!=state:
			r = find_simple_node(n)
			if r!=None : return r
	return None

def find_state(fsm, state_id):
	for s in fsm:
		if "id" in s.attrib and s.attrib["id"]==state_id: return s
	return None
	
def graph_gen_edges(xml, node, graph, elem, ids, fsm=None):
	#print 'proc-edge',node.tag, node.attrib['name'] if 'name' in node.attrib else ""
	if node.tag in ['scxml']:
		for chnode in node:
			graph_gen_edges(xml, chnode, graph, elem, ids, node)
	if node.tag in ['parallel']:
		for chnode in node:
			graph_gen_edges(xml, chnode, graph, elem, ids, fsm)
	if node.tag in ['task', 'tsk']:
		for chnode in node.findall('plan'):
			src_state_id = node.attrib["id"]
			dst_state_id = chnode.attrib['id']
			src = src_state_id
			dst = dst_state_id
			gr_edge = pydot.Edge(ids[src], ids[dst])
			#gr_edge.set_lhead(ids[dst_state_id])
			#gr_edge.set_ltail(ids[src_state_id])
			graph.add_edge(gr_edge)		
		for chnode in node:
			graph_gen_edges(xml, chnode, graph, elem, ids, fsm)
	if node.tag in ['state']:
		has_init_state = 'initialstate' in node.attrib
		if has_init_state:
			src_state_id = ids[node.attrib["id"]]+'start'
			dst_state_id = node.attrib["id"]+"/"+node.attrib['initialstate']
			dst_state = find_state(node, dst_state_id)
			dst = None
			if dst_state == None:
				print "ERROR: dst_state == None", dst_state_id
			else:
				dst = find_simple_node(dst_state)
				gr_edge = pydot.Edge(src_state_id, ids[dst])
				if dst!=dst_state: gr_edge.set_lhead('cluster_'+ids[dst_state_id])
				#gr_edge.set_ltail(src_state_id)
				gr_edge.set_fontsize("8")
				graph.add_edge(gr_edge)	
		for t in [ t for t in node.findall('transition') if 'target' in t.attrib ]:
			src_state_id = node.attrib["id"]
			dst_state_id = t.attrib['target']
			src = find_simple_node(node)
			dst_state = find_state(fsm, dst_state_id)
			dst = None
			if dst_state == None:
				print "ERROR: dst_state == None"
			else:
				dst = find_simple_node(dst_state)
				gr_edge = pydot.Edge(ids[src], ids[dst])
				gr_edge.set_label(t.attrib["event"])
				if dst!=dst_state_id: gr_edge.set_lhead('cluster_'+ids[dst_state_id])
				if src!=src_state_id: gr_edge.set_ltail('cluster_'+ids[src_state_id])
				gr_edge.set_fontsize("8")
				graph.add_edge(gr_edge)		
		for chnode in node:
			graph_gen_edges(xml, chnode, graph, elem, ids, node)
	if node.tag in ['plan', 'par', 'seq', 'sel', 'dec', 'pln']:
		for chnode in node:
			src_state_id = node.attrib["id"]
			dst_state_id = chnode.attrib['id']
			src = find_simple_node(node)
			dst_state = chnode
			dst = find_simple_node(dst_state)
			gr_edge = pydot.Edge(ids[src], ids[dst])
			if dst!=dst_state_id: gr_edge.set_lhead('cluster_'+ids[dst_state_id])
			#gr_edge.set_ltail(ids[src_state_id])
			graph.add_edge(gr_edge)		
			graph_gen_edges(xml, chnode, graph, elem, ids, fsm)
	
				

if __name__ == '__main__':

	parser = OptionParser()
	parser.add_option("-v", "--verbose", dest="verbose",
	                  help="verbose printouts", nargs=0)
	parser.add_option("-q", "--quiet", dest="quite",
	                  help="quiet printouts", nargs=0)
	parser.add_option("-i", "--info", dest="info",
	                  help="printouts just locations info", nargs=0)
	
	(options, args) = parser.parse_args()
	
	if len(sys.argv) < 2:
		sys.exit('Usage: %s projectName destinationFolder [...xmlFiles]' % sys.argv[0])

	if options.verbose is () or options.info is ():
		print "  -- project location is",args[0]
		print "  -- share folder is",args[1]
	if options.verbose is ():
		print "  -- FILES: ",'\n'.join(args[2:])
	print "-- Start decision making xml parsing"
	try:
		for fileXML in args[2].split(";"):
			fileName = path_leaf(fileXML)
			filetype = fileXML[-len("XXxml"):]
			if options.verbose is ():
				print fileXML, fileName,filetype
			xml = ET.parse(fileXML).getroot()
			graph = pydot.Dot(graph_type='digraph', compound='true')
			map_ids = map_all_ids(xml)
			if options.verbose is ():
				for k,v in map_ids.items(): print k,":",v
			graph_gen_nodes(xml, xml, graph, graph, map_ids)
			graph_gen_edges(xml, xml, graph, graph, map_ids)
			graph.write_raw(args[1] + os.sep + fileName[:-len("XXxml")] + "dot")
	except:
		print "  -- Unexpected error (",fileXML,"):", sys.exc_info()
		import traceback
		traceback.print_exc()
	print "-- End decision making xml parsing"
