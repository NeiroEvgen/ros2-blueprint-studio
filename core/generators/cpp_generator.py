import os
from jinja2 import Environment, FileSystemLoader
from .base import BaseGenerator

class CppGenerator(BaseGenerator):
    def __init__(self, project_name: str):
        super().__init__(project_name)
        template_path = os.path.join(os.path.dirname(__file__), 'templates')
        self.env = Environment(loader=FileSystemLoader(template_path))

    def generate(self, node_data: dict) -> str:
        template = self.env.get_template('cpp_lifecycle_node.j2')
        context = {
            'class_name': self.safe_class_name(node_data.get('name', 'Node')),
            'node_name': self.safe_ros_name(node_data.get('name', 'node')),
            'namespace': node_data.get('namespace', ''),
            'publishers': node_data.get('publishers', []),
            'subscribers': node_data.get('subscribers', []),
            'parameters': self.extract_parameters(node_data)
        }
        return template.render(context)
