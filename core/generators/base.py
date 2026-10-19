import re
import logging

class BaseGenerator:
    def __init__(self, project_name: str):
        self.project_name: str = project_name
        # logging
        self.logger = logging.getLogger("[INFO] [GENERATOR]")

    def safe_ros_name(self, name: str) -> str:
        """Validates a node/topic name."""
        safe = re.sub(r'[^a-zA-Z0-9_]', '_', name)
        if safe and safe[0].isdigit():
            safe = f"n_{safe}"
        return safe.lower()

    def safe_class_name(self, name: str) -> str:
        """Validates a class name (CamelCase)."""
        safe = re.sub(r'[^a-zA-Z0-9_]', '_', name)
        parts = [p.capitalize() for p in safe.split('_') if p]
        return "".join(parts) if parts else "DefaultNode"

    def extract_parameters(self, node_dict: dict) -> list:
        """Extracts ROS 2 parameters from the node's custom properties."""
        params = []
        custom = node_dict.get('custom', {})
        for key, value in custom.items():
            # skip system fields
            if key not in ['node_name', 'source_file', 'code_content', 'type']:
                params.append({
                    'name': key,
                    'value': value,
                    'type': self._guess_type(value)
                })
        return params

    def _guess_type(self, val):
        val_str = str(val)
        if val_str.lower() in ['true', 'false']:
            return 'bool'
        if val_str.replace('.', '', 1).isdigit():
            return 'double' if '.' in val_str else 'int'
        return 'string'
