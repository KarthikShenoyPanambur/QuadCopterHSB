@echo off
set xv_path=C:\\Xilinx\\Vivado\\2014.4\\bin
call %xv_path%/xelab  -wto e858d18fa8024734b4a94b9f9282ab0f -m64 --debug typical --relax -L xil_defaultlib -L secureip --snapshot TBOneBitComparator_behav xil_defaultlib.TBOneBitComparator -log elaborate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0
