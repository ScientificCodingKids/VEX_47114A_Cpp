How to manage VEXcode Pro V5 files effectively using github and Visual Studio Code.

The note is taken from a class instructed by our team coach.

# Use Git

By default, VEXCode V5 Code ("VEXCode" in short) will create and save files directly in "c:\users\XYZ\Documents\VEXcode-projects". We do NOT use this folder directly. Rather, we use a subfolder called "VEX_47114A_Cpp" and store all files within. It is also the root folder for this git repo.

If we want to create sample projects or any other projects that we do NOT want to manage by git, we should keep them outside of "VEX_47114A_Cpp" folder. In this case, we can simply save them in "VEXcode-projects" folder.


# Use Visual Studio Code

It is one of best tools available for code reading. Please install C/C++ plugin. Also install the GitLens plugin. Once installed, please add "c:\users\XYZ\Documents\VEXcode-projects".

Note that our vex.h refers to some header files like v5.h, v5_vcs.h. We must provide their location so that Visual Studio Code knows how to analyze the symbols and we can actually jump to there.

Be sure settings.json (in C/C++ plugin) has the following snipplet.

```
"C_Cpp.default.includePath": ["C:\\Program Files (x86)\\VEX Robotics\\VEXcode Pro V5\\sdk\\vexv5\\include", "C:\\Program Files (x86)\\VEX Robotics\\VEXcode Pro V5\\sdk\\vexv5\\gcc\\include", "C:\\Program Files (x86)\\VEX Robotics\\VEXcode Pro V5\\sdk\\vexv5\\gcc\\include\\c++\\4.9.3", ", "C:\\Program Files (x86)\\VEX Robotics\\VEXcode Pro V5\\sdk\\vexv5\\clang\\8.0.0\\include"]
```

For older app (VEXCode V5 Text):

```
"C_Cpp.default.includePath": ["C:\\Users\\XYZ\\AppData\\Local\\VEX Coding Studio\\VEX Coding Studio\\sdk\\vexv5\\include"]
}
```
