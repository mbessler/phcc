@ECHO OFF
REM This is a batchfile for DOS/Windows
REM For the AN851 PIC Bootloader Downloader Program
REM Copyright (c) 2004 by Manuel Bessler
REM http://cockpit.varxec.net/electronics/PHCC.html

REM The hexfile has to exist, bark if it doesn't
IF NOT EXIST "%1" GOTO ERROR

an851host.exe --eraseflash --rows 504 --startaddr 0x0200
an851host.exe --writeflash --hexfile %1
an851host.exe --reset
GOTO END


:ERROR
ECHO ERROR: cannot find hexfile %1 !
ECHO        you have to supply a hexfile on the commandline
PAUSE
GOTO END

:END
