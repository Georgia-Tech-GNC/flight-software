import yaml
from robot.libraries.BuiltIn import BuiltIn


class ConfigLoader:

    def load_config(self, filename):
        with open(filename, "r") as f:
            config = yaml.safe_load(f)

        built_in = BuiltIn()

        for name, value in config.items():
            built_in.set_suite_variable(f"${{{name}}}", value)
