from ..base import RosNodeBase


class NoteNode(RosNodeBase):
    __identifier__ = 'ros.nodes.meta'
    NODE_NAME = 'Note'

    def __init__(self):
        super(NoteNode, self).__init__()
        self.add_text_input('title', 'Title', text='README')
        self.set_color(230, 180, 60)
        if not self.has_property('code_content'):
            self.create_property('code_content',
                                 value='# Note\n\nDouble-click to edit.',
                                 widget_type=0)


class FileNode(RosNodeBase):
    """
    A project file on the canvas (green mark in the Files tab).
    No ports; not part of code or launch generation (type 'meta'). The file content is
    code_content; on Save, an edit made in Studio is written into the file itself (ProjectManager.sync_file_nodes).
    """
    __identifier__ = 'ros.nodes.meta'
    NODE_NAME = 'File'

    def __init__(self):
        super(FileNode, self).__init__()
        self.set_color(60, 140, 70)
        if not self.has_property('code_content'):
            self.create_property('code_content', value='', widget_type=0)
