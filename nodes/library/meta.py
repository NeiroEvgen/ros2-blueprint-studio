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