
import os

# Functions
def list_subdirectories(path):
    """Lists all subdirectories in the given path."""

    subdirectories = []
    for entry in os.scandir(path):
        if entry.is_dir():
            subdirectories.append(entry.name)
    return subdirectories

def find_file_with_ending(directory, ending):
    files = []
    for file in os.listdir(directory):
        if file.endswith(ending):
            return os.path.join(directory, file)


def remove_bad_lines(file_path, bad_lines, lines_to_remove):
    with open(file_path, "r") as file_handler:
        file_text = file_handler.readlines()
        for i in range(0, len(file_text)-lines_to_remove*len(bad_lines)): # since len(file_text) changes, this prevents checking out of bounds
            for bh in bad_lines:
                if bh in file_text[i]:
                    for j in range(0, lines_to_remove): file_text.pop(i)

    with open(file_path, "w") as file_handler:
        for line in file_text:
            file_handler.write(line)


def comment_bad_lines(file_path, bad_lines, lines_to_comment):
    with open(file_path, "r") as file_handler:
        file_text = file_handler.readlines()
        for i in range(0, len(file_text)-lines_to_comment*len(bad_lines)): # since len(file_text) changes, this prevents checking out of bounds
            for bh in bad_lines:
                if bh in file_text[i]:
                    for j in range(i, i + lines_to_comment): file_text[j] = "//" + file_text[j]

    with open(file_path, "w") as file_handler:
        for line in file_text:
            file_handler.write(line)

# User input
list_subdir = list_subdirectories("MainMCU/")
print("Choose MCU/Nucleo (Select a number):")
for i in range(len(list_subdir)):
    print(str(i) + ": " + list_subdir[i])
mcu = list_subdir[int(input())]


bad_handlers = ["PendSV_Handler", "SysTick_Handler", "SVC_Handler"]

# Fix stm32[..]xx_it.c - Remove handlers defined by FreeRTOS
directory_to_search = "MainMCU/"+mcu+"/Core/Src/"
file_ending = 'xx_it.c'
file_path = find_file_with_ending(directory_to_search, file_ending)
lines_to_remove = 10
#remove_bad_lines(file_path, bad_handlers, lines_to_remove)
comment_bad_lines(file_path, bad_handlers, lines_to_remove)
 
# Fix stm32[..]xx_it.h - Remove handlers defined by FreeRTOS
directory_to_search = "MainMCU/"+mcu+"/Core/Inc/"
file_ending = 'xx_it.h'
file_path = find_file_with_ending(directory_to_search, file_ending)
lines_to_remove = 1
#remove_bad_lines(file_path, bad_handlers, lines_to_remove)
comment_bad_lines(file_path, bad_handlers, lines_to_remove)