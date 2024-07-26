REM alright put glut.h in C:/MinGW/GL
REM put glut32.dll in C:/WINDOWS/system32
REM put glut32.lib in C:/MinGW/lib

g++.exe -o WinFMS.exe fms.cpp -w -lglut32 -lglu32 -lopengl32

