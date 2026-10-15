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
    Файл проекта на рабочем столе (зелёная отметка во вкладке Files).
    Без портов; в генерацию кода и launch не попадает (тип 'meta'). Содержимое файла —
    code_content; при Save правка в Studio пишется в сам файл (ProjectManager.sync_file_nodes).
    """
    __identifier__ = 'ros.nodes.meta'
    NODE_NAME = 'File'

    def __init__(self):
        super(FileNode, self).__init__()
        self.set_color(60, 140, 70)
        if not self.has_property('code_content'):
            self.create_property('code_content', value='', widget_type=0)
