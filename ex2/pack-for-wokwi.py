# Uploading/updating multiple files to Wokwi is a pain, so this script combines all the files into a single file
# so that you can easily copy and paste it into Wokwi's editor.

project_name = "ex2"
ino_file = f"{project_name}.ino"
output_file = f"../{project_name}-packed/{project_name}-packed.ino"

# ORDER matters! This is manually set
files_to_combine = [
    # IO FILES
    "src/io_config.hpp",    # no dependencies

    # AUDIO FILES
    "src/audio/Note.hpp",
    "src/audio/Song.hpp",   # must come after Note.hpp
    "src/audio/SongPlayer.hpp", # must come after Song.hpp
    "src/audio/songs.hpp",  # must come after Song.hpp
    "src/audio/songs.cpp",  # must come after songs.hpp
    "src/audio/Audio.hpp",  # must come after SongPlayer and songs

    "src/control/ControllerBase.hpp",
    "src/control/ControllerSm.hpp",
    "src/control/ControlData.hpp",
    "src/control/Controller.hpp",

    "src/ui/Display.hpp",
    "src/ui/UiSm.hpp",
    "src/ui/Ui.hpp",

    "src/buttons/ButtonSm.h",
    "src/buttons/ButtonSm.c",
    "src/buttons/MyArduinoButton.hpp",
    "src/buttons/Buttons.hpp",
    "src/buttons/Buttons.cpp",

    "src/control/ControllerSm.cpp",
    "src/ui/UiSm.cpp",

    # MAIN FILES
    "src/App.hpp",
    ino_file,  # should come after all other files
]

# foreach loop to read each file and append its content to a single string
combined_code = ""
for file in files_to_combine:
    with open(file, "r") as f:
        # add a comment with the file name
        separator = "//" + "/" * 80 + "\n"
        combined_code += separator
        combined_code += f"// FILE: {file}\n"
        combined_code += separator

        # read the file content and append it to the combined code
        combined_code += f.read()
        combined_code += "\n\n"

# remove pragmas and includes
combined_code = combined_code.replace("#pragma once", "// #pragma once")

# remove any includes that use quotes with a regex
combined_code = combined_code.replace('#include "', '// #include "')

# write the combined code to a new file
with open(output_file, "w") as f:
    f.write(combined_code)

print(f"File {output_file} written successfully")
