import logging
import uuid
from NodeGraphQt import BaseNode

# === 1. GROUP CLASS (proxy node) ===
class RosGroupNode(BaseNode):
    __identifier__ = 'ros.nodes'
    NODE_NAME = 'RosGroup'
    
    def __init__(self):
        super(RosGroupNode, self).__init__()
        self.set_color(40, 40, 40)
        self.add_text_input('node_name', 'Group Name', text='MySubGraph')
        
        # id used to link things on save/load
        self.create_property('group_uid', '') 
        
        # parent group id (for nesting)
        self.create_property('parent_group_id', '')
        
        # list of inner node ids
        self.create_property('internal_nodes', [])
        
        # connection map for the compiler
        self.create_property('port_links', {})
        
        self.create_property('is_expanded', False)
        
        # nesting level (for warnings)
        self.create_property('depth_level', 0)

    def on_mouse_double_clicked(self, pos):
        """Enter the subgraph on double click."""
        graph = self.graph
        viewer = graph.viewer()
        main_win = None
        parent = viewer.parent()
        while parent:
            if hasattr(parent, 'ui'):
                main_win = parent
                break
            parent = parent.parent()
        
        if main_win:
            main_win.enter_subgraph(self)
        super(RosGroupNode, self).on_mouse_double_clicked(pos)

    def get_template_data(self):
        return None 

# === 1.1. TERMINAL CLASSES (border ports) ===
class SubGraphInputNode(BaseNode):
    """Inner terminal for data COMING INTO the subgraph."""
    __identifier__ = 'ros.nodes.internal'
    NODE_NAME = 'Input Terminal'

    def __init__(self):
        super(SubGraphInputNode, self).__init__()
        self.set_color(100, 100, 255)
        self.add_output('out')
        self.create_property('linked_port', '') 
        self.create_property('group_uid', '')
        self.create_property('parent_group_id', '')

class SubGraphOutputNode(BaseNode):
    """Inner terminal for data GOING OUT of the subgraph."""
    __identifier__ = 'ros.nodes.internal'
    NODE_NAME = 'Output Terminal'

    def __init__(self):
        super(SubGraphOutputNode, self).__init__()
        self.set_color(255, 100, 100)
        self.add_input('in')
        self.create_property('linked_port', '')
        self.create_property('group_uid', '')
        self.create_property('parent_group_id', '')
# === 2. CREATING A GROUP ===
def create_group_from_selection(graph, name="SubGraph", color=(50, 50, 50)):
    selected_nodes = graph.selected_nodes()
    if not selected_nodes:
        return None, "Select nodes first!"

    # 1. Centre and creation
    xs = [n.pos()[0] for n in selected_nodes]
    ys = [n.pos()[1] for n in selected_nodes]
    cx = sum(xs) / len(selected_nodes)
    cy = sum(ys) / len(selected_nodes)

    node_type = 'ros.nodes.RosGroupNode' 
    if node_type not in graph.registered_nodes():
        node_type = 'ros.nodes.RosGroup'

    try:
        group_node = graph.create_node(node_type, pos=[cx, cy])
    except Exception as e:
        return None, f"Error creating group node ({node_type}): {e}"

    group_node.set_name(name)
    group_node.set_property('node_name', name)
    group_node.set_color(color[0], color[1], color[2])
    
    gid = str(uuid.uuid4())
    group_node.set_property('group_uid', gid)
    
    parent_gid = ""
    depth = 0
    for n in selected_nodes:
        if n.has_property('group_uid') and n.get_property('group_uid'):
            parent_gid = n.get_property('group_uid')
            all_nodes = graph.all_nodes()
            parent_node = next((an for an in all_nodes if an.has_property('group_uid') and an.get_property('group_uid') == parent_gid and an.type_ in ['ros.nodes.RosGroup', 'ros.nodes.RosGroupNode']), None)
            if parent_node:
                depth = parent_node.get_property('depth_level') + 1
            break

    group_node.set_property('parent_group_id', parent_gid)
    group_node.set_property('depth_level', depth)

    if depth > 3:
        logging.warning(f"Subgraph depth is {depth}. Deep nesting may affect performance.")

    internal_ids = []
    for n in selected_nodes:
        internal_ids.append(n.id)
        # The child refers to the group via parent_group_id.
        # The child's group_uid is NOT touched: that field means "I am a group with this id".
        if not n.has_property('parent_group_id'):
            try:
                n.create_property('parent_group_id', gid)
            except Exception:
                pass
        n.set_property('parent_group_id', gid)

    # 3. REWIRING WITH TERMINALS
    port_links = {} 

    # --- INCOMING (External -> Internal) ---
    for node in selected_nodes:
        for in_port in node.input_ports():
            connected = list(in_port.connected_ports())
            for source_port in connected:
                source_node = source_port.node()
                if source_node not in selected_nodes:
                    pname_ext = f"In_{source_port.name()}"
                    
                    if not group_node.get_input(pname_ext): 
                        group_node.add_input(pname_ext, color=source_port.color)
                        # create a terminal inside
                        term = graph.create_node('ros.nodes.internal.SubGraphInputNode', pos=[node.pos()[0]-200, node.pos()[1]])
                        term.set_name(pname_ext)
                        term.set_property('group_uid', gid)
                        term.set_property('parent_group_id', gid) 
                        term.set_property('linked_port', pname_ext)
                        term.view.setVisible(False)
                        internal_ids.append(term.id)
                    
                    g_in = group_node.get_input(pname_ext)
                    # find the terminal for this port
                    term = next(n for n in graph.all_nodes() if n.type_ == 'ros.nodes.internal.SubGraphInputNode' and n.get_property('linked_port') == pname_ext and n.get_property('group_uid') == gid)
                    
                    in_port.disconnect_from(source_port)
                    source_port.connect_to(g_in)
                    term.get_output('out').connect_to(in_port)
                    # remember the link external port -> inner terminal
                    port_links[pname_ext] = {
                        "direction": "in",
                        "terminal_id": term.id,
                        "internal_node_id": node.id,
                        "internal_port": in_port.name(),
                    }

    # --- OUTGOING (Internal -> External) ---
    for node in selected_nodes:
        for out_port in node.output_ports():
            connected = list(out_port.connected_ports())
            for target_port in connected:
                target_node = target_port.node()
                if target_node not in selected_nodes:
                    pname_ext = f"Out_{target_port.name()}"
                    
                    if not group_node.get_output(pname_ext): 
                        group_node.add_output(pname_ext, color=target_port.color)
                        # create a terminal inside
                        term = graph.create_node('ros.nodes.internal.SubGraphOutputNode', pos=[node.pos()[0]+200, node.pos()[1]])
                        term.set_name(pname_ext)
                        term.set_property('group_uid', gid)
                        term.set_property('parent_group_id', gid)  
                        term.set_property('linked_port', pname_ext)
                        term.view.setVisible(False)  
                        internal_ids.append(term.id)
                    
                    g_out = group_node.get_output(pname_ext)
                    # find the terminal
                    term = next(n for n in graph.all_nodes() if n.type_ == 'ros.nodes.internal.SubGraphOutputNode' and n.get_property('linked_port') == pname_ext and n.get_property('group_uid') == gid)
                    
                    target_port.disconnect_from(out_port)
                    out_port.connect_to(term.get_input('in'))
                    g_out.connect_to(target_port)
                    # remember the link inner terminal -> external port
                    port_links[pname_ext] = {
                        "direction": "out",
                        "terminal_id": term.id,
                        "internal_node_id": node.id,
                        "internal_port": out_port.name(),
                    }

    group_node.set_property('internal_nodes', internal_ids)
    group_node.set_property('port_links', port_links)

    # 4. Hide the inner nodes
    for nid in internal_ids:
        node = next((n for n in graph.all_nodes() if n.id == nid), None)
        if node and hasattr(node, 'view'):
            node.view.setVisible(False)

    graph.clear_selection()
    group_node.set_selected(True)
    
    return group_node, "OK"

# === 3. RESTORE AFTER LOAD/PASTE ===
def fix_group_after_load(graph, loaded_nodes):
    """
    Called after a paste or after loading a template.
    Finds groups and their children by 'group_uid', updates ids and hides the children.
    """
    groups = [n for n in loaded_nodes if n.type_ in ['ros.nodes.RosGroup', 'ros.nodes.RosGroupNode']]
    if not groups: return

    # go over all groups
    for grp in groups:
        gid = grp.get_property('group_uid')
        if not gid: continue
        
        # find the children among the loaded nodes with the same group_uid
        children = []
        children_ids = []
        
        for n in loaded_nodes:
            if n == grp: continue
            # children now refer to the group via parent_group_id
            if n.has_property('parent_group_id') and n.get_property('parent_group_id') == gid:
                children.append(n)
                children_ids.append(n.id)
        
        # update the id list in the group (now it knows its children's new ids)
        grp.set_property('internal_nodes', children_ids)
        
        # hide the children (so the group's insides are not exposed)
        grp.set_property('is_expanded', False)
        for child in children:
            if hasattr(child, 'view'):
                child.view.setVisible(False)
        
        # update the colour
        grp.set_color(40, 40, 40)

# === 4. VISIBILITY TOGGLE ===
def toggle_group_visibility(graph, group_node):
    internal_ids = group_node.get_property('internal_nodes')
    if not internal_ids: return
    
    new_state = not group_node.get_property('is_expanded')
    group_node.set_property('is_expanded', new_state)
    
    # find nodes by id
    all_nodes = {n.id: n for n in graph.all_nodes()}
    
    for nid in internal_ids:
        if nid in all_nodes:
            node = all_nodes[nid]
            if hasattr(node, 'view'):
                node.view.setVisible(new_state)
            
    group_node.set_color(80, 80, 80) if new_state else group_node.set_color(40, 40, 40)

def dissolve_group(graph, group_node):
    """
    Dissolves a group: plain children go back to the canvas (parent_group_id cleared),
    terminals are deleted. Call it BEFORE deleting the group itself.
    """
    gid = group_node.get_property('group_uid')
    if not gid:
        return

    terminals_to_delete = []
    for n in list(graph.all_nodes()):
        if n == group_node:
            continue
        # this group's terminals: to be deleted
        if n.type_ in ('ros.nodes.internal.SubGraphInputNode',
                       'ros.nodes.internal.SubGraphOutputNode') \
                and n.has_property('group_uid') and n.get_property('group_uid') == gid:
            terminals_to_delete.append(n)
            continue
        # plain children: release and show them
        if n.has_property('parent_group_id') and n.get_property('parent_group_id') == gid:
            n.set_property('parent_group_id', '')
            if hasattr(n, 'view'):
                n.view.setVisible(True)

    for term in terminals_to_delete:
        try:
            graph.delete_node(term)
        except Exception:
            pass