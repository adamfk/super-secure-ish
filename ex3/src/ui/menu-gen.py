from pathlib import Path
import re
from typing import List, Optional

# THIS CODE IS JANKY!!! Quick and dirty proof of concept.

# special name to indicate that the menu item is an edit option
EDIT_KEY = "**EDIT**"

input = """\
MENU
    DISARM_SYSTEM
        **EDIT**
    ARM_SYSTEM
        **EDIT**
    CONFIG
        CHANGE_CODE
            **EDIT**
        ARMING_DELAY
            **EDIT**
        ALARM_DELAY
            **EDIT**
    DATA
        UPTIME
        SENSOR
"""



class MenuItem:
    def __init__(self) -> None:
        self.title: str = ""
        self.indent: float = 0
        self.parent: Optional['MenuItem'] = None
        self.children: List['MenuItem'] = []

    def has_children(self) -> bool:
        return len(self.children) > 0


class Parser:
    def process(self, text: str) -> List[MenuItem]:
        lines = text.splitlines()
        items: List[MenuItem] = []

        for element in lines:
            item = self.process_line(element)
            if item:
                items.append(item)

        return items

    def process_line(self, line: str) -> Optional[MenuItem]:
        import re
        line = line.rstrip()

        if not line.strip():
            return None

        regex = r"^(\s*)(.*)\s*$"
        match = re.match(regex, line)

        if match is None:
            raise ValueError("Invalid line: " + line)

        indent = match.group(1)
        title = match.group(2)

        tabs = indent.count('\t')
        spaces = indent.count(' ')

        menu_item = MenuItem()
        menu_item.title = title
        menu_item.indent = tabs + spaces / 4

        return menu_item


class TreeBuilder:
    def __init__(self) -> None:
        self.root: Optional[MenuItem] = None
        self.last: Optional[MenuItem] = None
        self.stack: List[MenuItem] = []
        self.items: List[MenuItem] = []

    def build(self, items: List[MenuItem]) -> None:
        self.items = items[:]
        item = self.items.pop(0)

        if item.indent != 0:
            print(item)
            raise ValueError("First item must have indent of 0.")

        self.root = item
        self.last = item
        self.stack.append(item)

        while self.items:
            self._process_next()

    def _process_next(self) -> None:
        if not self.items:
            return

        item = self.items.pop(0)
        parent = self.peak_stack()

        if item.indent == 0:
            raise ValueError("Multiple root items found.")
        else:
            indent_diff = item.indent - self.last.indent

            if indent_diff == 0:
                pass
            elif indent_diff == 1:
                parent = self.last
                self.stack.append(parent)
            elif indent_diff > 1:
                raise ValueError("Invalid indent.")
            else:
                for _ in range(int(-indent_diff)):
                    self.stack.pop()
                parent = self.peak_stack()

            parent.children.append(item)
            item.parent = parent

        self.last = item

    def peak_stack(self) -> MenuItem:
        return self.stack[-1]

class PlantUmlGen:
    def __init__(self) -> None:
        self.states_text: str = ""
        self.transitions_text: str = ""

    def _make_option_name(self, item: MenuItem) -> str:
        if item.has_children():
            return f"{item.parent.title}__{item.title}"
        return item.title

    def gen_state(self, state: MenuItem, indent: str = "") -> str:
        s = f"{indent}state {state.title}"
        if not state.has_children():
            s += "\n"
        else:
            s += self.maybe_output_child_state(state, indent)
        return s

    def maybe_output_child_state(self, state: MenuItem, indent: str) -> str:
        children = state.children
        initial_child = children[0]

        if initial_child.title == EDIT_KEY:
            return "\n"

        s = " {\n"
        inner_indent = indent + "    "
        s += f"{inner_indent}[*] -> {self._make_option_name(initial_child)}\n"

        for child in children:
            s += f"{inner_indent}state {self._make_option_name(child)}\n"

        needs_spacer = True
        for child in children:
            if child.children:
                if needs_spacer:
                    s += "\n"
                    needs_spacer = False
                s += self.gen_state(child, inner_indent)

        s += indent + "}\n"
        return s

    def gen_state_transitions(self, state: MenuItem) -> str:
        s = ""
        children = state.children

        if not children:
            return s

        s += f"\n' {state.title}\n"

        parent = state.parent
        if parent is not None:
            s += f"{self._make_option_name(state)} -right-> {state.title} : ENTER_PRESS\n"
            s += f"{state.title} -left-> {self._make_option_name(state)} : BACK_PRESS\n"

        for i in range(1, len(children)):
            a = children[i - 1]
            b = children[i]
            s += f"{self._make_option_name(a)} -down-> {self._make_option_name(b)} : DOWN_PRESS\n"

        for i in range(len(children) - 1, 0, -1):
            a = children[i]
            b = children[i - 1]
            s += f"{self._make_option_name(a)} -up-> {self._make_option_name(b)} : UP_PRESS\n"

        for c in children:
            s += self.gen_state_transitions(c)

        return s

    def gen_handlers(self, state: MenuItem) -> str:
        s = ""

        if state.title == EDIT_KEY:
            return s

        if state.parent is not None:
            if len(state.children) <= 1:
                s += f"{state.title}: enter / lcd->{state.title}();\n"
            if state.has_children():
                s += f"{self._make_option_name(state)}: enter / lcd->{self._make_option_name(state)}();\n"

        for c in state.children:
            s += self.gen_handlers(c)

        return s


def process(text: str) -> str:
    parser = Parser()
    items = parser.process(text)

    builder = TreeBuilder()
    builder.build(items)

    gen = PlantUmlGen()
    output = gen.gen_state(builder.root)
    output += gen.gen_state_transitions(builder.root)

    output += "\n\n'EVENT HANDLERS\n"
    output += gen.gen_handlers(builder.root)

    lines = output.splitlines()
    output = "\n".join("    " + line for line in lines)

    return output


######## Main code to run the parser and generate the PlantUML code ########

output = process(input)
print(output)

# read UiSm.plantuml file and update between `'<AUTO_GENERATED_CODE>` and `'</AUTO_GENERATED_CODE>`
this_dir = Path(__file__).parent
file_path = this_dir / "UiSm.plantuml"
file_contents = ""
with open(file_path, "r") as f:
    file_contents = f.read()
    # use regex to do substitution
    pattern = r"'<AUTO_GENERATED_CODE>[\s\S]*'</AUTO_GENERATED_CODE>"
    replacement = f"'<AUTO_GENERATED_CODE>\n{output}\n'</AUTO_GENERATED_CODE>"
    file_contents = re.sub(pattern, replacement, file_contents, flags=re.DOTALL)

with open(file_path, "w") as f:
    f.write(file_contents)

print("\nUpdated UiSm.plantuml")