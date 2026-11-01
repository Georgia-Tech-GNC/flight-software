import yaml
from robot.libraries.BuiltIn import BuiltIn


class ConfigLoader:

    def load_config(self, filename):
        with open(filename, "r") as f:
            config = yaml.safe_load(f)

        built_in = BuiltIn()

        for name, value in config.items():
            self._set_variables(built_in, name, value)

    def _set_variables(self, built_in, name, value):
        if isinstance(value, dict):
            for child_name, child_value in value.items():
                self._set_variables(
                    built_in,
                    f"{name}.{child_name}",
                    child_value
                )
        else:
            built_in.set_suite_variable(
                f"${{{name}}}",
                value
            )
