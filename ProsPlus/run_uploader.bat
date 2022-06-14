@echo ON

set CONDAPATH=C:\ProgramData\Anaconda3\condabin
rem Define here the name of the environment
set ENVNAME=py37pros

rem The following command activates the base environment.
if %ENVNAME%==base (set ENVPATH=%CONDAPATH%) else (set ENVPATH=%CONDAPATH%\envs\%ENVNAME%)

set PYTHONPATH=%PROS_CLI_HOME%

rem Activate the conda environment
rem Using call is required here, see: https://stackoverflow.com/questions/24678144/conda-environments-and-bat-files
call %CONDAPATH%\activate.bat %ENVNAME%

python %VEX_DEV_HOME%\ProsPlus\Uploader\uploader_wired.py --slot=%1 --pname=%2 --icon=%3 --bin_file=%4

rem Deactivate the environment
call conda deactivate
