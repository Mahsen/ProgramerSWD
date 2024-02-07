@ECHO off
ECHO cd=%cd%
pushd %~dp0
python Tranfer.py main.bin COM48
pause